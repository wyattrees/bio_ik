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

namespace bio_ik
{

// pseudoinverse jacobian solver
// (mainly for testing RobotFK_Jacobian::computeJacobian)
template <class BASE> struct IKJacobianBase : BASE
{
    using BASE::modelInfo_;
    using BASE::model_;
    using BASE::params_;
    using BASE::computeFitness;
    using BASE::problem_;

    std::vector<Frame> tipObjectives;

    Eigen::VectorXd tip_diffs;
    Eigen::VectorXd joint_diffs;
    Eigen::MatrixXd jacobian;
    std::vector<Frame> tip_frames_temp;

    IKJacobianBase(const IKParams& p)
        : BASE(p)
    {
    }

    void initialize(const Problem& problem)
    {
        BASE::initialize(problem);
        tipObjectives.resize(problem.tip_link_indices.size());
        for(auto& goal : problem.goals)
            tipObjectives[goal.tip_index] = goal.frame;
    }

    void optimizeJacobian(std::vector<double>& solution)
    {
        FNPROFILER();

        int tip_count = static_cast<int>(problem_.tip_link_indices.size());
        tip_diffs.resize(tip_count * 6);
        joint_diffs.resize(static_cast<Eigen::Index>(problem_.active_variables.size()));

        // compute fk
        model_.applyConfiguration(solution);

        double translational_scale = 1;
        double rotational_scale = 1;

        // compute goal diffs
        tip_frames_temp = model_.getTipFrames();
        for(int itip = 0; itip < tip_count; itip++)
        {
            auto twist = frameTwist(tip_frames_temp[static_cast<size_t>(itip)], tipObjectives[static_cast<size_t>(itip)]);
            tip_diffs(itip * 6 + 0) = twist.vel.x() * translational_scale;
            tip_diffs(itip * 6 + 1) = twist.vel.y() * translational_scale;
            tip_diffs(itip * 6 + 2) = twist.vel.z() * translational_scale;
            tip_diffs(itip * 6 + 3) = twist.rot.x() * rotational_scale;
            tip_diffs(itip * 6 + 4) = twist.rot.y() * rotational_scale;
            tip_diffs(itip * 6 + 5) = twist.rot.z() * rotational_scale;
        }

        // compute jacobian
        {
            model_.computeJacobian(problem_.active_variables, jacobian);
            Eigen::Index icol = 0;
            for([[maybe_unused]] auto ivar : problem_.active_variables)
            {
                for(Eigen::Index itip = 0; itip < static_cast<Eigen::Index>(tip_count); itip++)
                {
                    jacobian(itip * 6 + 0, icol) *= translational_scale;
                    jacobian(itip * 6 + 1, icol) *= translational_scale;
                    jacobian(itip * 6 + 2, icol) *= translational_scale;
                    jacobian(itip * 6 + 3, icol) *= rotational_scale;
                    jacobian(itip * 6 + 4, icol) *= rotational_scale;
                    jacobian(itip * 6 + 5, icol) *= rotational_scale;
                }
            }
        }

        // get pseudoinverse and multiply
        joint_diffs = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(tip_diffs);
        // joint_diffs = (jacobian.transpose() * jacobian).ldlt().solve(jacobian.transpose() * tip_diffs);

        // apply joint deltas and clip
        {
            int icol = 0;
            for(auto ivar : problem_.active_variables)
            {
                auto v = solution[ivar] + joint_diffs(icol);
                if(!std::isfinite(v)) continue;
                v = modelInfo_.clip(v, ivar);
                solution[ivar] = v;
                icol++;
            }
        }
    }
};

// simple gradient descent
template <int if_stuck, size_t threads> struct IKGradientDescent : IKBase
{
    std::vector<double> solution, best_solution, gradient, temp;
    bool reset;

    IKGradientDescent(const IKParams& p)
        : IKBase(p)
    {
    }

    void initialize(const Problem& problem)
    {
        IKBase::initialize(problem);
        solution = problem.initial_guess;
        if(thread_index_ > 0)
            for(auto& vi : problem.active_variables)
                solution[vi] = random(modelInfo_.getMin(vi), modelInfo_.getMax(vi));
        best_solution = solution;
        reset = false;
    }

    const std::vector<double>& getSolution() const { return best_solution; }

    void step()
    {
        // random reset if stuck
        if(reset)
        {
            reset = false;
            for(auto& vi : problem_.active_variables)
                solution[vi] = random(modelInfo_.getMin(vi), modelInfo_.getMax(vi));
        }

        // compute gradient direction
        temp = solution;
        double jd = 0.0001;
        gradient.resize(solution.size(), 0);
        for(auto ivar : problem_.active_variables)
        {
            temp[ivar] = solution[ivar] - jd;
            double p1 = computeFitness(temp);

            temp[ivar] = solution[ivar] + jd;
            double p3 = computeFitness(temp);

            temp[ivar] = solution[ivar];

            gradient[ivar] = p3 - p1;
        }

        // normalize gradient direction
        double sum = 0.0001;
        for(auto ivar : problem_.active_variables)
            sum += fabs(gradient[ivar]);
        double f = 1.0 / sum * jd;
        for(auto ivar : problem_.active_variables)
            gradient[ivar] *= f;

        // initialize line search
        temp = solution;

        for(auto ivar : problem_.active_variables)
            temp[ivar] = solution[ivar] - gradient[ivar];
        double p1 = computeFitness(temp);

        // for(auto ivar : problem.active_variables) temp[ivar] = solution[ivar];
        // double p2 = computeFitness(temp);

        for(auto ivar : problem_.active_variables)
            temp[ivar] = solution[ivar] + gradient[ivar];
        double p3 = computeFitness(temp);

        double p2 = (p1 + p3) * 0.5;

        // linear step size estimation
        double cost_diff = (p3 - p1) * 0.5;
        double joint_diff = p2 / cost_diff;

        // in case cost_diff is 0
        if(!std::isfinite(joint_diff))
            joint_diff = 0.0;

        // apply optimization step
        // (move along gradient direction by estimated step size)
        for(auto ivar : problem_.active_variables)
            temp[ivar] = modelInfo_.clip(solution[ivar] - gradient[ivar] * joint_diff, ivar);

        if(if_stuck == 'c')
        {
            // always accept solution and continue
            solution = temp;
        }
        else
        {
            // has solution improved?
            if(computeFitness(temp) < computeFitness(solution))
            {
                // solution improved -> accept solution
                solution = temp;
            }
            else
            {
                if(if_stuck == 'r')
                {
                    // reset if stuck
                    reset = true;
                }
            }
        }

        // update best solution
        if(computeFitness(solution) < computeFitness(best_solution)) best_solution = solution;
    }

    size_t concurrency() const { return threads; }
};

// pseudoinverse jacobian only
template <size_t threads> struct IKJacobian : IKJacobianBase<IKBase>
{
    using IKBase::initialize;
    std::vector<double> solution;
    IKJacobian(const IKParams& p)
        : IKJacobianBase<IKBase>(p)
    {
    }
    void initialize(const Problem& problem)
    {
        IKJacobianBase<IKBase>::initialize(problem);
        solution = problem.initial_guess;
        if(thread_index_ > 0)
            for(auto& vi : problem.active_variables)
                solution[vi] = random(modelInfo_.getMin(vi), modelInfo_.getMax(vi));
    }
    const std::vector<double>& getSolution() const { return solution; }
    void step() { optimizeJacobian(solution); }
    size_t concurrency() const { return threads; }
};

std::optional<std::unique_ptr<IKSolver>> makeGradientDecentSolver(
    const IKParams& params) {
  const auto& name = params.solver_class_name;
  if (name == "gd")
    return std::make_unique<IKGradientDescent<' ', 1>>(params);
  else if (name == "gd_2")
    return std::make_unique<IKGradientDescent<' ', 2>>(params);
  else if (name == "gd_4")
    return std::make_unique<IKGradientDescent<' ', 4>>(params);
  else if (name == "gd_8")
    return std::make_unique<IKGradientDescent<' ', 8>>(params);
  else if (name == "gd_r")
    return std::make_unique<IKGradientDescent<'r', 1>>(params);
  else if (name == "gd_r_2")
    return std::make_unique<IKGradientDescent<'r', 2>>(params);
  else if (name == "gd_r_4")
    return std::make_unique<IKGradientDescent<'r', 4>>(params);
  else if (name == "gd_r_8")
    return std::make_unique<IKGradientDescent<'r', 8>>(params);
  else if (name == "gd_c")
    return std::make_unique<IKGradientDescent<'c', 1>>(params);
  else if (name == "gd_c_2")
    return std::make_unique<IKGradientDescent<'c', 2>>(params);
  else if (name == "gd_c_4")
    return std::make_unique<IKGradientDescent<'c', 4>>(params);
  else if (name == "gd_c_8")
    return std::make_unique<IKGradientDescent<'c', 8>>(params);
  else if (name == "jac")
    return std::make_unique<IKJacobian<1>>(params);
  else if (name == "jac_2")
    return std::make_unique<IKJacobian<2>>(params);
  else if (name == "jac_4")
    return std::make_unique<IKJacobian<4>>(params);
  else if (name == "jac_8")
    return std::make_unique<IKJacobian<8>>(params);
  else
    return std::nullopt;
}

}
