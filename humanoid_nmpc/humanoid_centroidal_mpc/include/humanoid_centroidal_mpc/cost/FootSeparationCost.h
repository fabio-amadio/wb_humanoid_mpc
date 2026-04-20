/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

class FootSeparationCost final : public StateInputCostGaussNewtonAd {
 public:
  struct Config {
    bool enabled = false;
    scalar_t weight = 0.0;
    scalar_t minLateralSeparation = 0.16;
    scalar_t smoothingEpsilon = 0.01;
    std::string leftFootFrame = "foot_l_contact";
    std::string rightFootFrame = "foot_r_contact";
  };

  FootSeparationCost(Config config,
                     const PinocchioInterface& pinocchioInterface,
                     const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                     std::string costName,
                     const ModelSettings& modelSettings);

  ~FootSeparationCost() override = default;
  FootSeparationCost* clone() const override { return new FootSeparationCost(*this); }

  vector_t getParameters(scalar_t time, const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const override;

  static Config loadConfig(const std::string& taskFile, const std::string& prefix, bool verbose = false);

 private:
  FootSeparationCost(const FootSeparationCost& other);

  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) override;

  Config config_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  std::unique_ptr<MpcRobotModelBase<ad_scalar_t>> mpcRobotModelAdPtr_;
};

}  // namespace ocs2::humanoid
