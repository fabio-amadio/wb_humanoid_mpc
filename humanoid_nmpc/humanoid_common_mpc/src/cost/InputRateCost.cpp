/******************************************************************************
Copyright (c) 2026, Manuel Yves Galliker. All rights reserved.

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

#include "humanoid_common_mpc/cost/InputRateCost.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LoadData.h>

#include <stdexcept>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void InputRateReference::preSolverRun(scalar_t initTime,
                                      scalar_t finalTime,
                                      const vector_t& initState,
                                      const ReferenceManagerInterface& referenceManager) {
  (void)initTime;
  (void)finalTime;
  (void)initState;
  (void)referenceManager;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void InputRateReference::postSolverRun(const PrimalSolution& primalSolution) {
  if (primalSolution.timeTrajectory_.empty() || primalSolution.inputTrajectory_.empty()) {
    return;
  }

  const size_t inputDim = primalSolution.inputTrajectory_.front().size();
  if (inputDim == 0 || primalSolution.timeTrajectory_.size() != primalSolution.inputTrajectory_.size()) {
    return;
  }
  for (const auto& input : primalSolution.inputTrajectory_) {
    if (static_cast<size_t>(input.size()) != inputDim) {
      return;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  timeTrajectory_ = primalSolution.timeTrajectory_;
  inputTrajectory_ = primalSolution.inputTrajectory_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool InputRateReference::getReferenceInput(scalar_t time, size_t inputDim, vector_t& referenceInput) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (timeTrajectory_.empty() || inputTrajectory_.empty() || inputTrajectory_.front().size() != static_cast<Eigen::Index>(inputDim)) {
    return false;
  }
  referenceInput = LinearInterpolation::interpolate(time, timeTrajectory_, inputTrajectory_);
  return referenceInput.size() == static_cast<Eigen::Index>(inputDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

InputRateCost::InputRateCost(matrix_t weights, std::shared_ptr<const InputRateReference> referencePtr)
    : weights_(std::move(weights)), referencePtr_(std::move(referencePtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

InputRateCost::Config InputRateCost::loadConfig(const std::string& taskFile,
                                                const std::string& prefix,
                                                size_t inputDim,
                                                bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  Config config;
  config.weights = matrix_t::Zero(inputDim, inputDim);
  loadData::loadPtreeValue(pt, config.active, prefix + "active", verbose);
  if (!config.active) {
    return config;
  }

  loadData::loadEigenMatrix(taskFile, prefix + "weights", config.weights);
  if (config.weights.rows() != static_cast<Eigen::Index>(inputDim) || config.weights.cols() != static_cast<Eigen::Index>(inputDim)) {
    throw std::runtime_error("[InputRateCost] Invalid weight matrix size in " + taskFile + ": expected " + std::to_string(inputDim) + "x" +
                             std::to_string(inputDim) + ", got " + std::to_string(config.weights.rows()) + "x" +
                             std::to_string(config.weights.cols()) + ".");
  }
  return config;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool InputRateCost::isActive(scalar_t time) const {
  (void)time;
  return referencePtr_ != nullptr && weights_.squaredNorm() > 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t InputRateCost::getValue(scalar_t time,
                                 const vector_t& state,
                                 const vector_t& input,
                                 const TargetTrajectories& targetTrajectories,
                                 const PreComputation& preComp) const {
  (void)state;
  (void)targetTrajectories;
  (void)preComp;

  vector_t referenceInput;
  if (referencePtr_ == nullptr || !referencePtr_->getReferenceInput(time, input.size(), referenceInput)) {
    return 0.0;
  }

  const vector_t inputDeviation = input - referenceInput;
  return 0.5 * inputDeviation.dot(weights_ * inputDeviation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ScalarFunctionQuadraticApproximation InputRateCost::getQuadraticApproximation(scalar_t time,
                                                                              const vector_t& state,
                                                                              const vector_t& input,
                                                                              const TargetTrajectories& targetTrajectories,
                                                                              const PreComputation& preComp) const {
  (void)targetTrajectories;
  (void)preComp;

  ScalarFunctionQuadraticApproximation L = ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());

  vector_t referenceInput;
  if (referencePtr_ == nullptr || !referencePtr_->getReferenceInput(time, input.size(), referenceInput)) {
    return L;
  }

  const vector_t inputDeviation = input - referenceInput;
  L.dfduu = weights_;
  L.dfdu.noalias() = weights_ * inputDeviation;
  L.f = 0.5 * inputDeviation.dot(L.dfdu);
  return L;
}

}  // namespace ocs2::humanoid
