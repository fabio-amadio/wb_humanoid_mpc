// SPDX-License-Identifier: BSD-3-Clause
#include "humanoid_centroidal_mpc/solver/InputRateSqpMpc.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_oc/oc_problem/LoopshapingOptimalControlProblem.h>
#include <ocs2_sqp/SqpSolver.h>
#include "humanoid_common_mpc/cost/JointAccelerationCost.h"

namespace ocs2::humanoid {

InputRateInitializer::InputRateInitializer(const Initializer& physicalInitializer, std::shared_ptr<LoopshapingDefinition> definition)
    : physicalInitializer_(physicalInitializer.clone()), definition_(std::move(definition)) {}

InputRateInitializer::InputRateInitializer(const InputRateInitializer& other)
    : physicalInitializer_(other.physicalInitializer_->clone()), definition_(other.definition_) {}

void InputRateInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  vector_t physicalInput, physicalNextState;
  physicalInitializer_->compute(time, definition_->getSystemState(state), nextTime, physicalInput, physicalNextState);
  input = definition_->getInputFilter().getD() * physicalInput;
  nextState = definition_->concatenateSystemAndFilterState(physicalNextState, definition_->getFilterState(state));
}

namespace {

/** Outer SolverBase updates references/modules in physical coordinates exactly once.
 * Inner SQP receives a snapshot and solves the augmented problem.
 */
class InputRateSolver final : public SolverBase {
 public:
  InputRateSolver(
      sqp::Settings settings, const OptimalControlProblem& problem, const Initializer& initializer, const matrix_t& weights, bool coldStart)
      : problem_(problem), initializer_(initializer.clone()), definition_(JointAccelerationCost::createDefinition(weights)), coldStart_(coldStart) {
    if (!definition_) {
      throw std::invalid_argument("[InputRateSqpMpc] At least one input-rate weight must be nonzero.");
    }
    if (settings.useFeedbackPolicy) {
      throw std::invalid_argument("[InputRateSqpMpc] Input-rate augmentation requires useFeedbackPolicy=false.");
    }
    const auto augmentedProblem = LoopshapingOptimalControlProblem::create(problem_, definition_);
    InputRateInitializer augmentedInitializer(initializer, definition_);
    solver_ = std::make_unique<SqpSolver>(std::move(settings), augmentedProblem, augmentedInitializer);
  }

  void reset() override {
    solver_->reset();
    previousSolution_.clear();
  }

  const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }
  const PerformanceIndex& getPerformanceIndeces() const override { return solver_->getPerformanceIndeces(); }
  size_t getNumIterations() const override { return solver_->getNumIterations(); }
  const std::vector<PerformanceIndex>& getIterationsLog() const override { return solver_->getIterationsLog(); }
  scalar_t getFinalTime() const override { return solver_->getFinalTime(); }
  const ProblemMetrics& getSolutionMetrics() const override { return solver_->getSolutionMetrics(); }

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* solution) const override {
    solver_->getPrimalSolution(finalTime, solution);
    for (size_t i = 0; i < solution->stateTrajectory_.size(); ++i) {
      const vector_t input = definition_->getSystemInput(solution->stateTrajectory_[i], solution->inputTrajectory_[i]);
      const vector_t state = definition_->getSystemState(solution->stateTrajectory_[i]);
      solution->inputTrajectory_[i] = input;
      solution->stateTrajectory_[i] = state;
    }
    solution->controllerPtr_ = std::make_unique<FeedforwardController>(solution->timeTrajectory_, solution->inputTrajectory_);
  }

  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override {
    const auto augmentedState = definition_->concatenateSystemAndFilterState(state, filterStateAt(time));
    const auto augmentedValue = solver_->getValueFunction(time, augmentedState);
    auto value = ScalarFunctionQuadraticApproximation::Zero(state.size(), 0);
    value.f = augmentedValue.f;
    value.dfdx = augmentedValue.dfdx.head(state.size());
    value.dfdxx = augmentedValue.dfdxx.topLeftCorner(state.size(), state.size());
    return value;
  }

  ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t, const vector_t&, const vector_t&) override {
    throw std::runtime_error("[InputRateSqpMpc] Hamiltonian queries are not supported.");
  }
  vector_t getStateInputEqualityConstraintLagrangian(scalar_t, const vector_t&) const override {
    throw std::runtime_error("[InputRateSqpMpc] Equality multiplier queries are not supported.");
  }
  MultiplierCollection getIntermediateDualSolution(scalar_t) const override {
    throw std::runtime_error("[InputRateSqpMpc] Dual solution queries are not supported.");
  }

 private:
  vector_t filterStateAt(scalar_t time) const {
    if (previousSolution_.timeTrajectory_.empty() || time < previousSolution_.timeTrajectory_.front() ||
        time > previousSolution_.timeTrajectory_.back()) {
      throw std::runtime_error("[InputRateSqpMpc] No input-state trajectory at the requested time; reset the MPC before restarting.");
    }
    const vector_t state = LinearInterpolation::interpolate(time, previousSolution_.timeTrajectory_, previousSolution_.stateTrajectory_);
    return definition_->getFilterState(state);
  }

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    vector_t filterState;
    if (previousSolution_.timeTrajectory_.empty()) {
      vector_t physicalInput, nextState, filterInput;
      initializer_->compute(initTime, initState, finalTime, physicalInput, nextState);
      definition_->getFilterEquilibrium(physicalInput, filterState, filterInput);
    } else {
      filterState = filterStateAt(initTime);
    }
    if (coldStart_) {
      solver_->reset();  // Retain continuous input states even when discarding the optimization warm start.
    }
    auto& references = solver_->getReferenceManager();
    references.setTargetTrajectories(getReferenceManager().getTargetTrajectories());
    references.setModeSchedule(getReferenceManager().getModeSchedule());
    solver_->run(initTime, definition_->concatenateSystemAndFilterState(initState, filterState),
                 getReferenceManager().getModeSchedule().modeAtTime(initTime), finalTime);
    solver_->getPrimalSolution(finalTime, &previousSolution_);
  }

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* controller) override {
    if (controller) {
      throw std::invalid_argument("[InputRateSqpMpc] External controllers require augmented coordinates and are not supported.");
    }
    runImpl(initTime, initState, finalTime);
  }
  void runImpl(scalar_t, const vector_t&, scalar_t, const PrimalSolution&) override {
    throw std::invalid_argument("[InputRateSqpMpc] External warm starts require augmented coordinates and are not supported.");
  }

  OptimalControlProblem problem_;
  std::unique_ptr<Initializer> initializer_;
  std::shared_ptr<LoopshapingDefinition> definition_;
  std::unique_ptr<SqpSolver> solver_;
  PrimalSolution previousSolution_;
  bool coldStart_;
};

}  // namespace

InputRateSqpMpc::InputRateSqpMpc(mpc::Settings mpcSettings,
                                 sqp::Settings sqpSettings,
                                 const OptimalControlProblem& problem,
                                 const Initializer& initializer,
                                 const matrix_t& weights)
    : MPC_BASE(mpcSettings),
      solver_(std::make_unique<InputRateSolver>(std::move(sqpSettings), problem, initializer, weights, mpcSettings.coldStart_)) {}

InputRateSqpMpc::~InputRateSqpMpc() = default;

void InputRateSqpMpc::calculateController(scalar_t initTime, const vector_t& initState, size_t initMode, scalar_t finalTime) {
  solver_->run(initTime, initState, initMode, finalTime);
}

}  // namespace ocs2::humanoid
