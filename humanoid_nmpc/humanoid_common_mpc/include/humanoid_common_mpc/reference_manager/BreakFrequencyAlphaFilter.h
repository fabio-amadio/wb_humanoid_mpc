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

#pragma once

#include <humanoid_common_mpc/common/Types.h>
#include <cassert>
#include <chrono>
#include <cmath>

namespace ocs2::humanoid {

class BreakFrequencyAlphaFilter final {
 public:
  using Clock = std::chrono::steady_clock;

  /**
   * Constructor
   *
   * @param [in] breakFrequency: Break frequency (cut-off frequency) in Hz.
   * @param [in] now: Time of the initial sample; defaults to the monotonic clock.
   */
  BreakFrequencyAlphaFilter(scalar_t breakFrequency, const vector_t& y_init, Clock::time_point now = Clock::now())
      : breakDeltaT_(1 / (2 * M_PI * breakFrequency)), y_last_(y_init), lastTimeFilterCalled(now) {}

  /** Update from the previous output and elapsed sample time. Explicit timestamps must be nondecreasing. */
  vector_t getFilteredVector(const vector_t& x, Clock::time_point now = Clock::now()) {
    assert(x.size() == y_last_.size());
    assert(now >= lastTimeFilterCalled);
    const scalar_t alpha = computeAlpha(now);
    y_last_ = alpha * x + (1 - alpha) * y_last_;
    lastTimeFilterCalled = now;
    return y_last_;
  }

 private:
  scalar_t computeAlpha(Clock::time_point now) const {
    const std::chrono::duration<double> durationInSeconds = now - lastTimeFilterCalled;
    scalar_t delta_t = durationInSeconds.count();
    return (delta_t / (delta_t + breakDeltaT_));
  }

  scalar_t breakDeltaT_;
  vector_t y_last_;
  Clock::time_point lastTimeFilterCalled;
};

}  // namespace ocs2::humanoid
