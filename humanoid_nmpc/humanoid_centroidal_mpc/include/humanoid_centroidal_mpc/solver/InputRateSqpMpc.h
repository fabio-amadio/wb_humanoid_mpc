// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_sqp/SqpSettings.h>

namespace ocs2::humanoid {

/** Initialize new horizon segments with zero rates, retaining the current physical input states.
 * Unlike the generic loopshaping initializer, this does not try to invert the singular integrator feedthrough matrix.
 */
class InputRateInitializer final : public Initializer {
 public:
  InputRateInitializer(const Initializer& physicalInitializer, std::shared_ptr<LoopshapingDefinition> definition);
  InputRateInitializer(const InputRateInitializer& other);
  InputRateInitializer* clone() const override { return new InputRateInitializer(*this); }
  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  std::unique_ptr<Initializer> physicalInitializer_;
  std::shared_ptr<LoopshapingDefinition> definition_;
};

/** Exact input-rate states internally, physical state/input trajectories externally.
 * Selected inputs start at the initializer value, then continue from the previous policy at each MPC update.
 * Policies are feedforward; feedback on internal input states requires an augmented-state observer.
 */
class InputRateSqpMpc final : public MPC_BASE {
 public:
  InputRateSqpMpc(mpc::Settings mpcSettings,
                  sqp::Settings sqpSettings,
                  const OptimalControlProblem& problem,
                  const Initializer& initializer,
                  const matrix_t& rateWeights);
  ~InputRateSqpMpc() override;
  SolverBase* getSolverPtr() override { return solver_.get(); }
  const SolverBase* getSolverPtr() const override { return solver_.get(); }

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, size_t initMode, scalar_t finalTime) override;

 private:
  std::unique_ptr<SolverBase> solver_;
};

}  // namespace ocs2::humanoid
