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

#include "humanoid_common_mpc/constraint/ArmBodyCollisionConstraint.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2::humanoid {

ArmBodyCollisionConstraint::ArmBodyCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                       const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,
                                                       const Config& config,
                                                       std::string costName,
                                                       const ModelSettings& modelSettings)
    : StateConstraintCppAd(ConstraintOrder::Linear),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelPtr_(&mpcRobotModel),
      cfg_(config) {
  initialize(mpcRobotModelPtr_->getStateDim(), 7, costName, modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
             modelSettings.verboseCppAd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ArmBodyCollisionConstraint::ArmBodyCollisionConstraint(const ArmBodyCollisionConstraint& other)
    : StateConstraintCppAd(other),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_),
      cfg_(other.cfg_),
      numConstraints_(other.numConstraints_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ArmBodyCollisionConstraint::constraintFunction(ad_scalar_t time,
                                                           const ad_vector_t& state,
                                                           const ad_vector_t& parameters) const {
  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto data = pinocchioInterfaceCppAd_.getData();

  const ad_vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(state);
  pinocchio::forwardKinematics(model, data, q);

  const ad_vector3_t leftElbowPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftElbowFrame)).translation();
  const ad_vector3_t rightElbowPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightElbowFrame)).translation();
  const ad_vector3_t leftForearmPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftForearmFrame)).translation();
  const ad_vector3_t rightForearmPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightForearmFrame)).translation();
  const ad_vector3_t leftHandPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftHandFrame)).translation();
  const ad_vector3_t rightHandPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightHandFrame)).translation();
  const ad_vector3_t leftUpperLegPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftUpperLegFrame)).translation();
  const ad_vector3_t rightUpperLegPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightUpperLegFrame)).translation();
  const ad_vector3_t leftKneePosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftKneeFrame)).translation();
  const ad_vector3_t rightKneePosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightKneeFrame)).translation();
  const ad_vector3_t leftLowerLegPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftLowerLegFrame)).translation();
  const ad_vector3_t rightLowerLegPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightLowerLegFrame)).translation();
  const ad_vector3_t torsoPosition = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.torsoFrame)).translation();

  const ad_scalar_t minElbowTorsoDistance = parameters[0] + parameters[6];
  const ad_scalar_t minForearmForearmDistance = 2.0 * parameters[1];
  const ad_scalar_t minForearmUpperLegDistance = parameters[1] + parameters[3];
  const ad_scalar_t minForearmKneeDistance = parameters[1] + parameters[4];
  const ad_scalar_t minForearmLowerLegDistance = parameters[1] + parameters[5];
  const ad_scalar_t minForearmTorsoDistance = parameters[1] + parameters[6];
  const ad_scalar_t minHandHandDistance = 2.0 * parameters[2];
  const ad_scalar_t minHandUpperLegDistance = parameters[2] + parameters[3];
  const ad_scalar_t minHandKneeDistance = parameters[2] + parameters[4];
  const ad_scalar_t minHandLowerLegDistance = parameters[2] + parameters[5];
  const ad_scalar_t minHandTorsoDistance = parameters[2] + parameters[6];

  ad_vector_t constraintValues(numConstraints_);
  constraintValues[0] = (leftElbowPosition - torsoPosition).norm() - minElbowTorsoDistance;
  constraintValues[1] = (rightElbowPosition - torsoPosition).norm() - minElbowTorsoDistance;
  constraintValues[2] = (leftHandPosition - rightHandPosition).norm() - minHandHandDistance;
  constraintValues[3] = (leftHandPosition - torsoPosition).norm() - minHandTorsoDistance;
  constraintValues[4] = (rightHandPosition - torsoPosition).norm() - minHandTorsoDistance;
  constraintValues[5] = (leftHandPosition - leftKneePosition).norm() - minHandKneeDistance;
  constraintValues[6] = (leftHandPosition - rightKneePosition).norm() - minHandKneeDistance;
  constraintValues[7] = (rightHandPosition - leftKneePosition).norm() - minHandKneeDistance;
  constraintValues[8] = (rightHandPosition - rightKneePosition).norm() - minHandKneeDistance;
  constraintValues[9] = (leftHandPosition - leftUpperLegPosition).norm() - minHandUpperLegDistance;
  constraintValues[10] = (leftHandPosition - rightUpperLegPosition).norm() - minHandUpperLegDistance;
  constraintValues[11] = (rightHandPosition - leftUpperLegPosition).norm() - minHandUpperLegDistance;
  constraintValues[12] = (rightHandPosition - rightUpperLegPosition).norm() - minHandUpperLegDistance;
  constraintValues[13] = (leftHandPosition - leftLowerLegPosition).norm() - minHandLowerLegDistance;
  constraintValues[14] = (leftHandPosition - rightLowerLegPosition).norm() - minHandLowerLegDistance;
  constraintValues[15] = (rightHandPosition - leftLowerLegPosition).norm() - minHandLowerLegDistance;
  constraintValues[16] = (rightHandPosition - rightLowerLegPosition).norm() - minHandLowerLegDistance;
  constraintValues[17] = (leftForearmPosition - torsoPosition).norm() - minForearmTorsoDistance;
  constraintValues[18] = (rightForearmPosition - torsoPosition).norm() - minForearmTorsoDistance;
  constraintValues[19] = (leftForearmPosition - leftUpperLegPosition).norm() - minForearmUpperLegDistance;
  constraintValues[20] = (leftForearmPosition - rightUpperLegPosition).norm() - minForearmUpperLegDistance;
  constraintValues[21] = (rightForearmPosition - leftUpperLegPosition).norm() - minForearmUpperLegDistance;
  constraintValues[22] = (rightForearmPosition - rightUpperLegPosition).norm() - minForearmUpperLegDistance;
  constraintValues[23] = (leftForearmPosition - leftKneePosition).norm() - minForearmKneeDistance;
  constraintValues[24] = (leftForearmPosition - rightKneePosition).norm() - minForearmKneeDistance;
  constraintValues[25] = (rightForearmPosition - leftKneePosition).norm() - minForearmKneeDistance;
  constraintValues[26] = (rightForearmPosition - rightKneePosition).norm() - minForearmKneeDistance;
  constraintValues[27] = (leftForearmPosition - leftLowerLegPosition).norm() - minForearmLowerLegDistance;
  constraintValues[28] = (leftForearmPosition - rightLowerLegPosition).norm() - minForearmLowerLegDistance;
  constraintValues[29] = (rightForearmPosition - leftLowerLegPosition).norm() - minForearmLowerLegDistance;
  constraintValues[30] = (rightForearmPosition - rightLowerLegPosition).norm() - minForearmLowerLegDistance;
  constraintValues[31] = (leftForearmPosition - rightForearmPosition).norm() - minForearmForearmDistance;
  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ArmBodyCollisionConstraint::Config ArmBodyCollisionConstraint::loadConfig(const std::string& taskFile,
                                                                          const std::string& fieldName,
                                                                          bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  Config config;
  if (const auto active = pt.get_optional<bool>(fieldName + "active")) {
    config.active = *active;
  }
  if (!config.active) {
    return config;
  }

  loadData::loadPtreeValue(pt, config.leftElbowFrame, fieldName + "leftElbowFrame", verbose);
  loadData::loadPtreeValue(pt, config.rightElbowFrame, fieldName + "rightElbowFrame", verbose);
  loadData::loadPtreeValue(pt, config.leftForearmFrame, fieldName + "leftForearmFrame", verbose);
  loadData::loadPtreeValue(pt, config.rightForearmFrame, fieldName + "rightForearmFrame", verbose);
  loadData::loadPtreeValue(pt, config.leftHandFrame, fieldName + "leftHandFrame", verbose);
  loadData::loadPtreeValue(pt, config.rightHandFrame, fieldName + "rightHandFrame", verbose);
  loadData::loadPtreeValue(pt, config.leftUpperLegFrame, fieldName + "leftUpperLegFrame", verbose);
  loadData::loadPtreeValue(pt, config.rightUpperLegFrame, fieldName + "rightUpperLegFrame", verbose);
  loadData::loadPtreeValue(pt, config.leftKneeFrame, fieldName + "leftKneeFrame", verbose);
  loadData::loadPtreeValue(pt, config.rightKneeFrame, fieldName + "rightKneeFrame", verbose);
  loadData::loadPtreeValue(pt, config.leftLowerLegFrame, fieldName + "leftLowerLegFrame", verbose);
  loadData::loadPtreeValue(pt, config.rightLowerLegFrame, fieldName + "rightLowerLegFrame", verbose);
  loadData::loadPtreeValue(pt, config.torsoFrame, fieldName + "torsoFrame", verbose);
  loadData::loadPtreeValue(pt, config.elbowSphereRadius, fieldName + "elbowSphereRadius", verbose);
  loadData::loadPtreeValue(pt, config.forearmSphereRadius, fieldName + "forearmSphereRadius", verbose);
  loadData::loadPtreeValue(pt, config.handSphereRadius, fieldName + "handSphereRadius", verbose);
  loadData::loadPtreeValue(pt, config.upperLegSphereRadius, fieldName + "upperLegSphereRadius", verbose);
  loadData::loadPtreeValue(pt, config.kneeSphereRadius, fieldName + "kneeSphereRadius", verbose);
  loadData::loadPtreeValue(pt, config.lowerLegSphereRadius, fieldName + "lowerLegSphereRadius", verbose);
  loadData::loadPtreeValue(pt, config.torsoSphereRadius, fieldName + "torsoSphereRadius", verbose);
  return config;
}

}  // namespace ocs2::humanoid
