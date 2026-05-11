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

#include "humanoid_common_mpc/constraint/BaseHeightSoftConstraint.h"

namespace ocs2::humanoid {

BaseHeightSoftConstraint::BaseHeightSoftConstraint(
    scalar_t maxBaseHeight,
    ocs2::PieceWisePolynomialBarrierPenalty::Config barrierSettings,
    const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : penaltyPtr_(new ocs2::PieceWisePolynomialBarrierPenalty(barrierSettings)),
      mpcRobotModelPtr_(&mpcRobotModel),
      maxBaseHeight_(maxBaseHeight) {}

BaseHeightSoftConstraint::BaseHeightSoftConstraint(const BaseHeightSoftConstraint& rhs)
    : penaltyPtr_(rhs.penaltyPtr_->clone()),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      maxBaseHeight_(rhs.maxBaseHeight_),
      isActive_(rhs.isActive_) {}

scalar_t BaseHeightSoftConstraint::getValue(
    scalar_t,
    const vector_t& state,
    const ocs2::TargetTrajectories&,
    const ocs2::PreComputation&) const {
  return getValue(mpcRobotModelPtr_->getBasePose(state)(2));
}

ScalarFunctionQuadraticApproximation BaseHeightSoftConstraint::getQuadraticApproximation(
    scalar_t,
    const vector_t& state,
    const ocs2::TargetTrajectories&,
    const ocs2::PreComputation&) const {
  return getQuadraticApproximation(mpcRobotModelPtr_->getBasePose(state)(2));
}

scalar_t BaseHeightSoftConstraint::getValue(scalar_t baseHeight) const {
  const scalar_t offset = maxBaseHeight_ - baseHeight;
  return penaltyPtr_->getValue(0.0, offset);
}

ScalarFunctionQuadraticApproximation BaseHeightSoftConstraint::getQuadraticApproximation(scalar_t baseHeight) const {
  const scalar_t offset = maxBaseHeight_ - baseHeight;
  const size_t stateDim = mpcRobotModelPtr_->getStateDim();
  const size_t baseStartIndex = mpcRobotModelPtr_->getBaseStartindex();
  const size_t baseHeightIndex = baseStartIndex + 2;

  ScalarFunctionQuadraticApproximation cost;
  cost.f = penaltyPtr_->getValue(0.0, offset);
  cost.dfdx = vector_t::Zero(stateDim);
  cost.dfdx(baseHeightIndex) = -penaltyPtr_->getDerivative(0.0, offset);
  cost.dfdxx = matrix_t::Zero(stateDim, stateDim);
  cost.dfdxx(baseHeightIndex, baseHeightIndex) = penaltyPtr_->getSecondDerivative(0.0, offset);
  return cost;
}

void BaseHeightSoftConstraint::setGains(const scalar_t& mu, const scalar_t& delta) {
  penaltyPtr_->setConfig(ocs2::PieceWisePolynomialBarrierPenalty::Config(mu, delta));
}

void BaseHeightSoftConstraint::getGains(scalar_t& mu, scalar_t& delta) const {
  ocs2::PieceWisePolynomialBarrierPenalty::Config config;
  penaltyPtr_->getConfig(config);
  mu = config.mu;
  delta = config.delta;
}

}  // namespace ocs2::humanoid
