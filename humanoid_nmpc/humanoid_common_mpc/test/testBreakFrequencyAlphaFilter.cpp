#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

#include "humanoid_common_mpc/reference_manager/BreakFrequencyAlphaFilter.h"

using namespace ocs2;
using namespace ocs2::humanoid;
using namespace std::chrono;

namespace {
const BreakFrequencyAlphaFilter::Clock::time_point start{};
constexpr double frequency = 5.0;
const double tau = 1.0 / (2.0 * M_PI * frequency);
const double alpha = 0.02 / (0.02 + tau);
}  // namespace

TEST(CommandFilter, ConstantCommandPreservesInitialStandingHeight) {
  vector_t standing(4);
  standing << 0.0, 0.0, 0.76, 0.0;
  BreakFrequencyAlphaFilter filter(frequency, standing, start);
  for (int sample = 0; sample <= 100; ++sample) {
    EXPECT_TRUE(filter.getFilteredVector(standing, start + milliseconds(20 * sample)).isApprox(standing, 1e-12));
  }
}

TEST(CommandFilter, StepResponseAccumulatesPreviousOutputs) {
  BreakFrequencyAlphaFilter filter(frequency, vector_t::Zero(1), start);
  for (int sample = 1; sample <= 20; ++sample) {
    const auto output = filter.getFilteredVector(vector_t::Ones(1), start + milliseconds(20 * sample));
    EXPECT_NEAR(output[0], 1.0 - std::pow(1.0 - alpha, sample), 1e-12);
    EXPECT_GT(output[0], 0.0);
    EXPECT_LT(output[0], 1.0);
  }
}

TEST(CommandFilter, StopCommandDecaysFromPreviousOutput) {
  BreakFrequencyAlphaFilter filter(frequency, vector_t::Zero(2), start);
  vector_t command(2);
  command << 1.0, -2.0;
  const auto moving = filter.getFilteredVector(command, start + milliseconds(20));
  const auto stopped = filter.getFilteredVector(vector_t::Zero(2), start + milliseconds(40));
  EXPECT_TRUE(stopped.isApprox((1.0 - alpha) * moving, 1e-12));
  EXPECT_GT(stopped[0], 0.0);
  EXPECT_LT(stopped[1], 0.0);
}

TEST(CommandFilter, GainUsesLatestIntervalAfterLongUptime) {
  BreakFrequencyAlphaFilter filter(frequency, vector_t::Ones(1), start);
  filter.getFilteredVector(vector_t::Ones(1), start + seconds(10));
  const auto output = filter.getFilteredVector(vector_t::Constant(1, 2.0), start + seconds(10) + milliseconds(20));
  EXPECT_NEAR(output[0], 1.0 + alpha, 1e-12);
}

TEST(CommandFilter, RepeatedTimestampDoesNotAdvanceFilter) {
  BreakFrequencyAlphaFilter filter(frequency, vector_t::Zero(1), start);
  const auto first = filter.getFilteredVector(vector_t::Ones(1), start + milliseconds(20));
  EXPECT_TRUE(filter.getFilteredVector(vector_t::Zero(1), start + milliseconds(20)).isApprox(first, 1e-12));
  EXPECT_NEAR(filter.getFilteredVector(vector_t::Ones(1), start + milliseconds(40))[0], 1.0 - std::pow(1.0 - alpha, 2), 1e-12);
}

TEST(CommandFilter, HigherCutoffRespondsFaster) {
  BreakFrequencyAlphaFilter slow(2.0, vector_t::Zero(1), start);
  BreakFrequencyAlphaFilter fast(10.0, vector_t::Zero(1), start);
  const auto slowOutput = slow.getFilteredVector(vector_t::Ones(1), start + milliseconds(20));
  const auto fastOutput = fast.getFilteredVector(vector_t::Ones(1), start + milliseconds(20));
  EXPECT_GT(fastOutput[0], slowOutput[0]);
  EXPECT_GT(slowOutput[0], 0.0);
  EXPECT_LT(fastOutput[0], 1.0);
}
