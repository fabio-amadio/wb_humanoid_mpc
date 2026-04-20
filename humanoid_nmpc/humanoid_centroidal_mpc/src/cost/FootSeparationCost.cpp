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

#include <pinocchio/fwd.hpp>

#include "humanoid_centroidal_mpc/cost/FootSeparationCost.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace ocs2::humanoid {

namespace {

template <typename T>
void loadOptional(const boost::property_tree::ptree& pt, const std::string& key, T& value) {
  if (const auto loaded = pt.get_optional<T>(key)) {
    value = loaded.get();
  }
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootSeparationCost::FootSeparationCost(Config config,
                                       const PinocchioInterface& pinocchioInterface,
                                       const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                       std::string costName,
                                       const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      config_(std::move(config)),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelAdPtr_(mpcRobotModelAD.clone()) {
  const auto& model = pinocchioInterface.getModel();
  if (model.getFrameId(config_.leftFootFrame) == static_cast<pinocchio::FrameIndex>(model.nframes)) {
    throw std::runtime_error("Foot separation cost leftFootFrame not found: " + config_.leftFootFrame);
  }
  if (model.getFrameId(config_.rightFootFrame) == static_cast<pinocchio::FrameIndex>(model.nframes)) {
    throw std::runtime_error("Foot separation cost rightFootFrame not found: " + config_.rightFootFrame);
  }

  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), 3, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd);
  std::cout << "Initialized FootSeparationCost with min lateral separation " << config_.minLateralSeparation << " and weight "
            << config_.weight << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootSeparationCost::FootSeparationCost(const FootSeparationCost& other)
    : StateInputCostGaussNewtonAd(other),
      config_(other.config_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelAdPtr_(other.mpcRobotModelAdPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t FootSeparationCost::costVectorFunction(ad_scalar_t time,
                                                   const ad_vector_t& state,
                                                   const ad_vector_t& input,
                                                   const ad_vector_t& parameters) {
  (void)time;
  (void)input;

  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto& data = pinocchioInterfaceCppAd_.getData();

  const ad_vector_t q = mpcRobotModelAdPtr_->getGeneralizedCoordinates(state);
  pinocchio::forwardKinematics(model, data, q);

  const ad_vector3_t leftFootPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(config_.leftFootFrame)).translation();
  const ad_vector3_t rightFootPosition =
      pinocchio::updateFramePlacement(model, data, model.getFrameId(config_.rightFootFrame)).translation();

  const ad_vector6_t basePose = mpcRobotModelAdPtr_->getBasePose(state);
  const ad_scalar_t yaw = basePose[3];
  const ad_vector3_t leftToRight = leftFootPosition - rightFootPosition;
  const ad_scalar_t lateralSeparationInPelvis = -CppAD::sin(yaw) * leftToRight[0] + CppAD::cos(yaw) * leftToRight[1];

  const ad_scalar_t sqrtWeight = parameters[0];
  const ad_scalar_t minSeparation = parameters[1];
  const ad_scalar_t epsilon = parameters[2];
  const ad_scalar_t violation = minSeparation - lateralSeparationInPelvis;
  const ad_scalar_t smoothPositivePart = 0.5 * (violation + CppAD::sqrt(violation * violation + epsilon * epsilon));

  ad_vector_t cost(1);
  cost[0] = sqrtWeight * smoothPositivePart;
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t FootSeparationCost::getParameters(scalar_t time,
                                           const TargetTrajectories& targetTrajectories,
                                           const PreComputation& preComputation) const {
  (void)time;
  (void)targetTrajectories;
  (void)preComputation;

  vector_t parameters(3);
  parameters << std::sqrt(config_.weight), config_.minLateralSeparation, config_.smoothingEpsilon;
  return parameters;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootSeparationCost::Config FootSeparationCost::loadConfig(const std::string& taskFile, const std::string& prefix, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  Config config;
  loadOptional(pt, prefix + "enabled", config.enabled);
  loadOptional(pt, prefix + "weight", config.weight);
  loadOptional(pt, prefix + "minLateralSeparation", config.minLateralSeparation);
  loadOptional(pt, prefix + "smoothingEpsilon", config.smoothingEpsilon);
  loadOptional(pt, prefix + "leftFootFrame", config.leftFootFrame);
  loadOptional(pt, prefix + "rightFootFrame", config.rightFootFrame);

  if (config.weight < 0.0) {
    throw std::runtime_error("Foot separation cost weight must be non-negative.");
  }
  if (config.minLateralSeparation < 0.0) {
    throw std::runtime_error("Foot separation cost minLateralSeparation must be non-negative.");
  }
  if (config.smoothingEpsilon <= 0.0) {
    throw std::runtime_error("Foot separation cost smoothingEpsilon must be positive.");
  }

  if (verbose) {
    std::cerr << "\n #### Foot Separation Cost Config: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << " #### 'enabled'............................................................" << config.enabled << "\n";
    std::cerr << " #### 'weight'............................................................." << config.weight << "\n";
    std::cerr << " #### 'minLateralSeparation'..............................................." << config.minLateralSeparation << "\n";
    std::cerr << " #### 'smoothingEpsilon'..................................................." << config.smoothingEpsilon << "\n";
    std::cerr << " #### 'leftFootFrame'......................................................" << config.leftFootFrame << "\n";
    std::cerr << " #### 'rightFootFrame'....................................................." << config.rightFootFrame << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return config;
}

}  // namespace ocs2::humanoid
