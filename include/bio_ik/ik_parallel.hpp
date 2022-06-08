/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Philipp Sebastian Ruppel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <bio_ik/ik_base.hpp>
#include <bio_ik/ik_evolution_1.hpp>
#include <bio_ik/ik_evolution_2.hpp>
#include <bio_ik/ik_test.hpp>
#include <bio_ik/ik_gradient.hpp>

#include <boost/thread/barrier.hpp>
#include <optional>

namespace bio_ik
{

// executes a function in parallel on pre-allocated threads
class ParallelExecutor
{
    volatile bool exit;
    std::vector<std::thread> threads;
    std::function<void(size_t)> fun;
    boost::barrier barrier;
    double best_fitness_;

public:
    template <class FUN>
    ParallelExecutor(size_t thread_count_, const FUN& f)
        : exit(false)
        , threads(thread_count_)
        , fun(f)
        , barrier(static_cast<unsigned int>(thread_count_)) 
    {
        for(size_t i = 1; i < thread_count_; i++)
        {
            std::thread t([this, i]() {
                while(true)
                {
                    barrier.wait();
                    if(exit) break;
                    fun(i);
                    barrier.wait();
                    if(exit) break;
                }
            });
            std::swap(t, threads[i]);
        }
    }
    ~ParallelExecutor()
    {
        exit = true;
        barrier.wait();
        for(auto& t : threads)
            if(t.joinable()) t.join();
    }
    void run()
    {
        barrier.wait();
        fun(0);
        barrier.wait();
    }
};

std::optional<std::unique_ptr<IKSolver>> makeSolver(const IKParams& params) {
  if (auto evolution_1 = makeEvolution1Solver(params))
    return evolution_1;
  if (auto evolution_2 = makeEvolution2Solver(params))
    return evolution_2;
  if (auto gradient = makeGradientDecentSolver(params))
    return gradient;
  if (auto test = makeTestSolver(params))
    return test;
  return std::nullopt;
}

// runs ik on multiple threads until a stop criterion is met
struct IKParallel
{
    IKParams params_;
    std::vector<std::unique_ptr<IKSolver>> solvers_;
    std::vector<std::vector<double>> solver_solutions_;
    std::vector<std::vector<double>> solver_temps_;
    std::vector<int> solver_success_;
    std::vector<double> solver_fitness_;
    size_t thread_count_;
    // std::vector<RobotFK_Fast> fk; // TODO: remove
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> timeout_;
    bool success_;
    std::atomic<int> finished_;
    std::atomic<uint32_t> iteration_count_;
    std::vector<double> result_;
    std::unique_ptr<ParallelExecutor> par_;
    Problem problem_;
    bool enable_counter_;
    double best_fitness_;

    IKParallel(const IKParams& params)
        : params_(params)
    {
        // solver class name
        std::string name = params_.solver_class_name;

        enable_counter_ = params_.enable_counter;

        // create solvers
        auto create_solver = [&params]() {
            auto result = makeSolver(params);
            if (!result) {
                throw std::runtime_error("Invalid Solver Name: " +
                                        params.solver_class_name);
            }
            return std::move(*result);
        };
        solvers_.emplace_back(create_solver());
        thread_count_ = solvers_.front()->concurrency();
        if(params_.thread_count) {
            thread_count_ = params_.thread_count;
        }
        while(solvers_.size() < thread_count_)
            solvers_.emplace_back(create_solver());
        for(size_t i = 0; i < thread_count_; i++)
            solvers_[i]->thread_index_ = i;

        // while(fk.size() < thread_count_) fk.emplace_back(params.robot_model);

        // init buffers
        solver_solutions_.resize(thread_count_);
        solver_temps_.resize(thread_count_);
        solver_success_.resize(thread_count_);
        solver_fitness_.resize(thread_count_);

        // create parallel executor
        par_.reset(new ParallelExecutor(thread_count_, [this](size_t i) { solverthread(i); }));
    }

    void initialize(const Problem& problem)
    {
        problem_ = problem;
        // for(auto& f : fk) f.initialize(problem_.tip_link_indices);
    }

private:
    void solverthread(size_t i)
    {
        THREADPROFILER("thread", i);
        COUNTERPROFILER("solver threads");

        // initialize ik solvers
        {
            BLOCKPROFILER("ik solver init");
            solvers_[i]->initialize(problem_);
        }

        // run solver iterations until solution found or timeout
        for(size_t iteration = 0; (std::chrono::system_clock::now() < timeout_ && finished_ == 0) || (iteration == 0 && i == 0); iteration++)
        {
            if(finished_) break;

            // run solver for a few steps
            solvers_[i]->step();
            iteration_count_++;
            for(int it2 = 1; it2 < 4; it2++)
                if(std::chrono::system_clock::now() < timeout_ && finished_ == 0) solvers_[i]->step();

            if(finished_) break;

            // get solution and check stop criterion
            auto& result = solver_temps_[i];
            result = solvers_[i]->getSolution();
            auto& fk = solvers_[i]->model_;
            fk.applyConfiguration(result);
            bool success = solvers_[i]->checkSolution(result, fk.getTipFrames());
            if(success) finished_ = 1;
            solver_success_[i] = success;
            solver_solutions_[i] = result;
            solver_fitness_[i] = solvers_[i]->computeFitness(result, fk.getTipFrames());

            if(success) break;
        }

        finished_ = 1;

        for(auto& s : solvers_)
            s->canceled_ = true;
    }

public:
    void solve()
    {
        BLOCKPROFILER("solve mt");

        // prepare
        iteration_count_ = 0;
        result_ = problem_.initial_guess;
        timeout_ = problem_.timeout;
        success_ = false;
        finished_ = 0;
        for(auto& s : solver_solutions_)
            s = problem_.initial_guess;
        for(auto& s : solver_temps_)
            s = problem_.initial_guess;
        for(auto& s : solver_success_)
            s = 0;
        for(auto& f : solver_fitness_)
            f = DBL_MAX;
        for(auto& s : solvers_)
            s->canceled_ = false;

        // run solvers_
        {
            BLOCKPROFILER("solve mt 2");
            par_->run();
        }

        size_t best_index = 0;
        best_fitness_ = DBL_MAX;

        // if exact primary goal matches have been found ...
        for(size_t i = 0; i < thread_count_; i++)
        {
            if(solver_success_[i])
            {
                double fitness;
                if(solvers_[0]->problem_.secondary_goals.empty())
                {
                    // ... and if no secondary goals have been specified,
                    // select the best result_ according to primary goals
                    fitness = solver_fitness_[i];
                }
                else
                {
                    // ... and if secondary goals have been specified,
                    // select the result that best satisfies primary and secondary goals
                    fitness = solver_fitness_[i] + solvers_[0]->computeSecondaryFitnessAllVariables(solver_solutions_[i]);
                }
                if(fitness < best_fitness_)
                {
                    best_fitness_ = fitness;
                    best_index = i;
                }
            }
        }

        // if no exact primary goal matches have been found,
        // select best primary goal approximation
        if(best_fitness_ == DBL_MAX)
        {
            for(size_t i = 0; i < thread_count_; i++)
            {
                if(solver_fitness_[i] < best_fitness_)
                {
                    best_fitness_ = solver_fitness_[i];
                    best_index = i;
                }
            }
        }

        if(enable_counter_)
        {
            LOG("iterations", iteration_count_);
        }

        result_ = solver_solutions_[best_index];
        success_ = solver_success_[best_index];
    }

    double getSolutionFitness() const { return best_fitness_; }

    bool getSuccess() const { return success_; }

    const std::vector<double>& getSolution() const { return result_; }
};
}
