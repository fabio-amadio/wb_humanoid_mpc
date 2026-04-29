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

#include "humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h"

#include <boost/proto/proto_fwd.hpp>
#include <algorithm>
#include <cmath>

#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ocs2_core/misc/LoadData.h>
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2::humanoid {

CentroidalMpcTargetTrajectoriesCalculator::CentroidalMpcTargetTrajectoriesCalculator(const std::string& referenceFile,
                                                                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                                     PinocchioInterface pinocchioInterface,
                                                                                     const CentroidalModelInfo& info,
                                                                                     scalar_t mpcHorizon)
    : TargetTrajectoriesCalculatorBase(referenceFile, mpcRobotModel, mpcHorizon),
      pinocchioInterface_(pinocchioInterface),
      info_(info),
      mass_(pinocchio::computeTotalMass(pinocchioInterface.getModel())),
      waistYawJointIndex_(mpcRobotModel.getJointIndex("waist_yaw_joint")),
      waistRollJointIndex_(mpcRobotModel.getJointIndex("waist_roll_joint")),
      waistPitchJointIndex_(mpcRobotModel.getJointIndex("waist_pitch_joint")) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedPositionToTargetTrajectories(const vector4_t& commadLinePoseTarget,
                                                                                                    scalar_t initTime,
                                                                                                    const vector_t& initState) {
  vector_t currentPoseTarget = getCurrentBasePoseTarget(initState);

  const vector_t targetPose = getDeltaBaseTarget(commadLinePoseTarget, currentPoseTarget);

  scalar_t targetReachingTime = initTime + estimateTimeToTarget(targetPose - currentPoseTarget);

  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << vector_t::Zero(6), currentPoseTarget, targetJointState_;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, targetJointState_;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};

  return targetTrajectories;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories(const WalkingVelocityCommand& commandedVelocities,
                                                                                                    scalar_t initTime,
                                                                                                    const vector_t& initState) {
  // Smoothly transition from the current base motion reference to the commanded one over the first part of the horizon.

  vector_t currentPoseTarget = getCurrentBasePoseTarget(initState);
  vector_t targetJointState = targetJointState_;
  targetJointState(waistYawJointIndex_) = commandedVelocities.desired_waist_yaw;
  targetJointState(waistRollJointIndex_) = commandedVelocities.desired_waist_roll;
  targetJointState(waistPitchJointIndex_) = commandedVelocities.desired_waist_pitch;

  vector4_t commVelTargetGlobal = filterAndTransformVelCommandToLocal(commandedVelocities.toVector(), currentPoseTarget(3), 0.8);

  updateCentroidalDynamics(pinocchioInterface_, info_, mpcRobotModelPtr_->getGeneralizedCoordinates(initState));
  const Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>& A = getCentroidalMomentumMatrix(pinocchioInterface_);

  const Eigen::Matrix<scalar_t, 6, 6> Ab = A.leftCols<6>();
  const Eigen::Matrix<scalar_t, 6, 6> Ab_inv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  vector6_t baseVel = Ab_inv * initState.head(6);
  vector4_t currentVelTarget;
  currentVelTarget << baseVel[0], baseVel[1], currentPoseTarget[2], baseVel[5];
  const scalar_t transitionDuration = std::min(std::max<scalar_t>(velocityTransitionDuration_, 0.0), mpcHorizon_);

  std::vector<scalar_t> knotTimes{0.0};
  const scalar_t halfTransition = 0.5 * transitionDuration;
  if (halfTransition > 1e-9 && halfTransition < mpcHorizon_ - 1e-9) {
    knotTimes.push_back(halfTransition);
  }
  if (transitionDuration > 1e-9 && transitionDuration < mpcHorizon_ - 1e-9) {
    knotTimes.push_back(transitionDuration);
  }
  if (mpcHorizon_ > knotTimes.back()) {
    knotTimes.push_back(mpcHorizon_);
  }

  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
  timeTrajectory.reserve(knotTimes.size());
  stateTrajectory.reserve(knotTimes.size());
  inputTrajectory.reserve(knotTimes.size());

  vector6_t poseAtKnot = currentPoseTarget;
  vector4_t previousVelocityRef = currentVelTarget;
  scalar_t previousRelativeTime = 0.0;

  for (const scalar_t relativeTime : knotTimes) {
    const scalar_t alpha = transitionDuration > 1e-9 ? std::clamp(relativeTime / transitionDuration, scalar_t(0.0), scalar_t(1.0)) : 1.0;
    const vector4_t velocityRef = (1.0 - alpha) * currentVelTarget + alpha * commVelTargetGlobal;

    if (relativeTime > previousRelativeTime) {
      const vector3_t averageVel((previousVelocityRef[0] + velocityRef[0]) * 0.5, (previousVelocityRef[1] + velocityRef[1]) * 0.5,
                                 (previousVelocityRef[3] + velocityRef[3]) * 0.5);
      poseAtKnot = integrateTargetBasePose(poseAtKnot, averageVel, velocityRef[2], relativeTime - previousRelativeTime);
    } else {
      poseAtKnot[2] = velocityRef[2];
    }

    vector6_t momentumRef;
    momentumRef << velocityRef[0], velocityRef[1], 0.0, 0.0, 0.0, velocityRef[3] / mass_;

    timeTrajectory.push_back(initTime + relativeTime);
    stateTrajectory.emplace_back(vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
    stateTrajectory.back() << momentumRef, poseAtKnot, targetJointState;
    inputTrajectory.emplace_back(vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

    previousVelocityRef = velocityRef;
    previousRelativeTime = relativeTime;
  }

  return TargetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};
}

}  // namespace ocs2::humanoid
