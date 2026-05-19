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

#pragma once

#include <memory>
#include <mutex>

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2::humanoid {

class InputRateReference final : public SolverSynchronizedModule {
 public:
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override;

  bool getReferenceInput(scalar_t time, size_t inputDim, vector_t& referenceInput) const;

 private:
  mutable std::mutex mutex_;
  scalar_array_t timeTrajectory_;
  vector_array_t inputTrajectory_;
};

class InputRateCost final : public StateInputCost {
 public:
  struct Config {
    bool active = false;
    matrix_t weights;
  };

  InputRateCost(matrix_t weights, std::shared_ptr<const InputRateReference> referencePtr);

  ~InputRateCost() override = default;
  InputRateCost* clone() const override { return new InputRateCost(*this); }

  static Config loadConfig(const std::string& taskFile, const std::string& prefix, size_t inputDim, bool verbose);

  bool isActive(scalar_t time) const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time,
                                                                 const vector_t& state,
                                                                 const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  InputRateCost(const InputRateCost& rhs) = default;

  matrix_t weights_;
  std::shared_ptr<const InputRateReference> referencePtr_;
};

}  // namespace ocs2::humanoid
