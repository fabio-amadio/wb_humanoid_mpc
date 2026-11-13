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

#include "humanoid_common_mpc/cost/JointAccelerationCost.h"

#include <ocs2_core/misc/LoadData.h>
#include <Eigen/Eigenvalues>
#include <boost/property_tree/info_parser.hpp>
#include <stdexcept>
#include <vector>

namespace ocs2::humanoid {

JointAccelerationCost::Config JointAccelerationCost::loadConfig(const std::string& taskFile, const std::string& prefix, size_t inputDim, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  Config config;
  config.weights = matrix_t::Zero(inputDim, inputDim);
  loadData::loadPtreeValue(pt, config.active, prefix + "active", verbose);
  if (config.active) {
    loadData::loadEigenMatrix(taskFile, prefix + "weights", config.weights);
    if (config.weights.rows() != static_cast<Eigen::Index>(inputDim) || config.weights.cols() != static_cast<Eigen::Index>(inputDim)) {
      throw std::invalid_argument("[JointAccelerationCost] Weight matrix must match the physical input dimension.");
    }
    createDefinition(config.weights);
  }
  return config;
}

std::shared_ptr<LoopshapingDefinition> JointAccelerationCost::createDefinition(const matrix_t& weights) {
  if (weights.rows() != weights.cols() || !weights.allFinite() || !weights.isApprox(weights.transpose())) {
    throw std::invalid_argument("[JointAccelerationCost] Weights must be finite, square and symmetric.");
  }
  if (weights.size() == 0 || weights.isZero(0.0)) {
    return nullptr;
  }
  Eigen::SelfAdjointEigenSolver<matrix_t> eigenvalues(weights);
  if (eigenvalues.info() != Eigen::Success || eigenvalues.eigenvalues().minCoeff() < -1e-12 * weights.norm()) {
    throw std::invalid_argument("[JointAccelerationCost] Weights must be positive semidefinite.");
  }
  std::vector<Eigen::Index> selected;
  for (Eigen::Index i = 0; i < weights.rows(); ++i) {
    if (!weights.row(i).isZero(0.0)) {
      selected.push_back(i);
    }
  }
  const auto n = static_cast<Eigen::Index>(selected.size());
  const auto m = weights.rows();
  matrix_t selectedWeights(n, n);
  for (Eigen::Index i = 0; i < n; ++i) {
    for (Eigen::Index j = 0; j < n; ++j) {
      selectedWeights(i, j) = weights(selected[i], selected[j]);
    }
  }
  if (Eigen::LLT<matrix_t>(selectedWeights).info() != Eigen::Success) {
    throw std::invalid_argument("[JointAccelerationCost] The nonzero weight block must be positive definite.");
  }
  matrix_t A = matrix_t::Zero(n, n);
  matrix_t B = matrix_t::Zero(n, m);
  matrix_t C = matrix_t::Zero(m, n);
  matrix_t D = matrix_t::Identity(m, m);
  for (Eigen::Index j = 0; j < n; ++j) {
    const auto i = selected[j];
    B(j, i) = 1.0;  // z_dot[j] = v[i]
    C(i, j) = 1.0;  // physical u[i] = z[j]
    D(i, i) = 0.0;
  }
  return std::make_shared<LoopshapingDefinition>(LoopshapingType::eliminatepattern,
                                                 Filter(std::move(A), std::move(B), std::move(C), std::move(D)), weights);
}

}  // namespace ocs2::humanoid
