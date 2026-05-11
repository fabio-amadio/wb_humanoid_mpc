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

#include <pinocchio/fwd.hpp>

#include <ocs2_core/constraint/StateConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/algorithm/frames.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

class ArmBodyCollisionConstraint final : public StateConstraintCppAd {
 public:
  struct Config {
    bool active = false;
    std::string leftElbowFrame;
    std::string rightElbowFrame;
    std::string leftForearmFrame;
    std::string rightForearmFrame;
    std::string leftHandFrame;
    std::string rightHandFrame;
    std::string leftUpperLegFrame;
    std::string rightUpperLegFrame;
    std::string leftKneeFrame;
    std::string rightKneeFrame;
    std::string leftLowerLegFrame;
    std::string rightLowerLegFrame;
    std::string torsoFrame;
    scalar_t elbowSphereRadius = 0.0;
    scalar_t forearmSphereRadius = 0.0;
    scalar_t handSphereRadius = 0.0;
    scalar_t upperLegSphereRadius = 0.0;
    scalar_t kneeSphereRadius = 0.0;
    scalar_t lowerLegSphereRadius = 0.0;
    scalar_t torsoSphereRadius = 0.0;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ArmBodyCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                             const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,
                             const Config& config,
                             std::string costName,
                             const ModelSettings& modelSettings);

  ~ArmBodyCollisionConstraint() override = default;
  ArmBodyCollisionConstraint* clone() const override { return new ArmBodyCollisionConstraint(*this); }

  bool isActive(scalar_t time) const override { return cfg_.active; }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; };

  vector_t getParameters(scalar_t time, const PreComputation& preComputation) const override {
    vector_t parameters(7);
    parameters << cfg_.elbowSphereRadius, cfg_.forearmSphereRadius, cfg_.handSphereRadius, cfg_.upperLegSphereRadius, cfg_.kneeSphereRadius,
        cfg_.lowerLegSphereRadius, cfg_.torsoSphereRadius;
    return parameters;
  };

  static Config loadConfig(const std::string& taskFile, const std::string& fieldName, bool verbose = false);

 private:
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override;

  ArmBodyCollisionConstraint(const ArmBodyCollisionConstraint& other);

  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  const MpcRobotModelBase<ad_scalar_t>* const mpcRobotModelPtr_;
  Config cfg_;

  const size_t numConstraints_ = 32;
};

}  // namespace ocs2::humanoid
