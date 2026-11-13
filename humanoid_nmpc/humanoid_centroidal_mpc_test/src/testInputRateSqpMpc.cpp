// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_oc/oc_problem/LoopshapingOptimalControlProblem.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "humanoid_centroidal_mpc/solver/InputRateSqpMpc.h"
#include "humanoid_common_mpc/cost/JointAccelerationCost.h"

using namespace ocs2;
using namespace ocs2::humanoid;

namespace {
OptimalControlProblem trackingProblem() {
  OptimalControlProblem problem;
  problem.dynamicsPtr = std::make_unique<LinearSystemDynamics>(matrix_t::Zero(1, 1), matrix_t::Identity(1, 1));
  problem.costPtr->add("trackInput", std::make_unique<QuadraticStateInputCost>(matrix_t::Zero(1, 1), matrix_t::Identity(1, 1)));
  return problem;
}

sqp::Settings solverSettings(scalar_t dt = 0.02) {
  sqp::Settings settings;
  settings.dt = dt;
  settings.nThreads = 1;
  settings.useFeedbackPolicy = false;
  settings.enableLogging = false;
  return settings;
}

void setInputTarget(MPC_BASE& mpc, scalar_t target) {
  mpc.getSolverPtr()->getReferenceManager().setTargetTrajectories(
      TargetTrajectories({0.0}, {vector_t::Zero(1)}, {vector_t::Constant(1, target)}));
}

scalar_t rateEnergy(const PrimalSolution& solution) {
  scalar_t energy = 0.0;
  for (size_t k = 1; k < solution.timeTrajectory_.size(); ++k) {
    const scalar_t dt = solution.timeTrajectory_[k] - solution.timeTrajectory_[k - 1];
    if (dt > 1e-9) {
      energy += (solution.inputTrajectory_[k] - solution.inputTrajectory_[k - 1]).squaredNorm() / dt;
    }
  }
  return energy;
}

struct CountingModule final : SolverSynchronizedModule {
  int before = 0, after = 0;
  void preSolverRun(scalar_t, scalar_t, const vector_t& state, const ReferenceManagerInterface&) override {
    ++before;
    EXPECT_EQ(state.size(), 1);
  }
  void postSolverRun(const PrimalSolution& solution) override {
    ++after;
    EXPECT_EQ(solution.stateTrajectory_.front().size(), 1);
    EXPECT_EQ(solution.inputTrajectory_.front().size(), 1);
  }
};
}  // namespace

TEST(JointAccelerationCost, SelectedInputDynamicsAndCostAreExact) {
  matrix_t weights = matrix_t::Zero(3, 3);
  weights(1, 1) = 2.0;
  weights(2, 2) = 3.0;
  const auto definition = JointAccelerationCost::createDefinition(weights);
  ASSERT_NE(definition, nullptr);
  const vector_t state = (vector_t(3) << 0.0, 0.4, -0.3).finished();
  const vector_t control = (vector_t(3) << 7.0, 2.0, -4.0).finished();
  EXPECT_TRUE(definition->getSystemInput(state, control).isApprox((vector_t(3) << 7.0, 0.4, -0.3).finished()));
  EXPECT_TRUE(definition->filterFlowMap(state.tail(2), control).isApprox((vector_t(2) << 2.0, -4.0).finished()));
  EXPECT_DOUBLE_EQ(definition->loopshapingCost(control), 28.0);
  vector_t constantControl = control;
  constantControl.tail(2).setZero();
  EXPECT_DOUBLE_EQ(definition->loopshapingCost(constantControl), 0.0);
}

TEST(JointAccelerationCost, InvalidWeightsAreRejectedAndZeroDisablesAugmentation) {
  EXPECT_EQ(JointAccelerationCost::createDefinition(matrix_t::Zero(3, 3)), nullptr);
  EXPECT_THROW(JointAccelerationCost::createDefinition(-matrix_t::Identity(2, 2)), std::invalid_argument);
  EXPECT_THROW(JointAccelerationCost::createDefinition(matrix_t::Ones(2, 3)), std::invalid_argument);
  EXPECT_THROW(JointAccelerationCost::createDefinition(matrix_t::Ones(2, 2)), std::invalid_argument);
  matrix_t invalid = matrix_t::Identity(2, 2);
  invalid(0, 1) = 0.2;
  EXPECT_THROW(JointAccelerationCost::createDefinition(invalid), std::invalid_argument);
  invalid(0, 0) = std::numeric_limits<scalar_t>::quiet_NaN();
  EXPECT_THROW(JointAccelerationCost::createDefinition(invalid), std::invalid_argument);
}

TEST(JointAccelerationCost, InitializerPreservesNonzeroInputState) {
  const auto definition = JointAccelerationCost::createDefinition(matrix_t::Identity(1, 1));
  DefaultInitializer physicalInitializer(1);
  InputRateInitializer initializer(physicalInitializer, definition);
  const vector_t state = (vector_t(2) << 0.1, 0.2).finished();
  vector_t input, next;
  initializer.compute(0.25, state, 0.27, input, next);
  EXPECT_TRUE(input.allFinite());
  EXPECT_TRUE(next.allFinite());
  EXPECT_NEAR(input[0], 0.0, 1e-10);
  EXPECT_NEAR(next[1], state[1], 1e-10);
}

TEST(InputRateSqpMpc, FirstSolveMatchesContinuousRateOptimalSolutionAndIncreasingWeightSmoothsInput) {
  // Minimize integral 0.5*((u-1)^2 + w*u_dot^2), u(0)=0, free u(T).
  // Analytic u(t) = 1 - cosh((T-t)/sqrt(w))/cosh(T/sqrt(w)).
  scalar_t lowWeightEnergy = 0.0;
  for (const scalar_t weight : {0.01, 1.0}) {
    const auto problem = trackingProblem();
    DefaultInitializer initializer(1);
    mpc::Settings settings;
    settings.timeHorizon_ = 1.0;
    InputRateSqpMpc mpc(settings, solverSettings(), problem, initializer, matrix_t::Constant(1, 1, weight));
    setInputTarget(mpc, 1.0);
    ASSERT_TRUE(mpc.run(0.0, vector_t::Zero(1), 0));
    const auto solution = mpc.getSolverPtr()->primalSolution(1.0);
    for (size_t k = 0; k < solution.timeTrajectory_.size(); ++k) {
      const scalar_t time = solution.timeTrajectory_[k];
      const scalar_t expected = 1.0 - std::cosh((1.0 - time) / std::sqrt(weight)) / std::cosh(1.0 / std::sqrt(weight));
      ASSERT_EQ(solution.stateTrajectory_[k].size(), 1);
      ASSERT_EQ(solution.inputTrajectory_[k].size(), 1);
      EXPECT_NEAR(solution.inputTrajectory_[k][0], expected, 0.03);
      EXPECT_NEAR(solution.controllerPtr_->computeInput(time, solution.stateTrajectory_[k])[0], solution.inputTrajectory_[k][0], 1e-9);
    }
    if (weight < 1.0) {
      lowWeightEnergy = rateEnergy(solution);
    } else {
      EXPECT_LT(rateEnergy(solution), 0.5 * lowWeightEnergy);
    }
  }
}

class InputRateMpcRestart : public ::testing::TestWithParam<bool> {};

TEST_P(InputRateMpcRestart, ReplanningPreservesInputAndResetRestoresInitialInput) {
  const auto problem = trackingProblem();
  DefaultInitializer initializer(1);
  mpc::Settings settings;
  settings.timeHorizon_ = 1.0;
  settings.coldStart_ = GetParam();
  InputRateSqpMpc mpc(settings, solverSettings(), problem, initializer, matrix_t::Identity(1, 1));
  auto module = std::make_shared<CountingModule>();
  mpc.getSolverPtr()->addSynchronizedModule(module);
  setInputTarget(mpc, 1.0);
  ASSERT_NO_THROW({ EXPECT_TRUE(mpc.run(0.0, vector_t::Zero(1), 0)); });
  const auto previous = mpc.getSolverPtr()->primalSolution(1.0);
  const vector_t state = LinearInterpolation::interpolate(0.25, previous.timeTrajectory_, previous.stateTrajectory_);
  const vector_t input = LinearInterpolation::interpolate(0.25, previous.timeTrajectory_, previous.inputTrajectory_);
  ASSERT_TRUE(state.allFinite());
  ASSERT_TRUE(input.allFinite());
  setInputTarget(mpc, -1.0);
  ASSERT_NO_THROW({ EXPECT_TRUE(mpc.run(0.25, state, 0)); });
  const auto next = mpc.getSolverPtr()->primalSolution(1.25);
  EXPECT_NEAR(next.inputTrajectory_.front()[0], input[0], 1e-6);
  EXPECT_EQ(module->before, 2);
  EXPECT_EQ(module->after, 2);
  mpc.reset();
  ASSERT_NO_THROW({ EXPECT_TRUE(mpc.run(0.0, vector_t::Zero(1), 0)); });
  EXPECT_NEAR(mpc.getSolverPtr()->primalSolution(1.0).inputTrajectory_.front()[0], 0.0, 1e-6);
}

INSTANTIATE_TEST_SUITE_P(WarmAndColdStart, InputRateMpcRestart, ::testing::Bool());
