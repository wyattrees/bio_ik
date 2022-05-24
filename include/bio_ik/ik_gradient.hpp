// Copyright (c) 2016-2017, Philipp Sebastian Ruppel
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universität Hamburg nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <Eigen/Core>          // For NumTraits
#include <bio_ik/ik_base.hpp>  // for IKSolver
#include <bio_ik/problem.hpp>  // for Problem, Problem::GoalInfo
#include <bio_ik/utils.hpp>    // for FNPROFILER
#include <cmath>               // for isfinite
#include <cstddef>             // for size_t
#include <ext/alloc_traits.h>  // for __alloc_traits<>::value_type
#include <kdl/frames.hpp>      // for Twist, Vector
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>  // for vector, allocator

#include "bio_ik/frame.hpp"       // for Frame, frameTwist
#include "bio_ik/robot_info.hpp"  // for RobotInfo

namespace bio_ik {

std::optional<std::unique_ptr<IKSolver>> makeGradientDecentSolver(
    const IKParams& params);

const auto getGradientDecentModes = []() {
  return std::set<std::string>{
      "gd",     "gd_2",   "gd_4",  "gd_8",   "gd_r",   "gd_r_2",
      "gd_r_4", "gd_r_8", "gd_c",  "gd_c_2", "gd_c_4", "gd_c_8",
      "jac",    "jac_2",  "jac_4", "jac_8",
  };
};

}  // namespace bio_ik