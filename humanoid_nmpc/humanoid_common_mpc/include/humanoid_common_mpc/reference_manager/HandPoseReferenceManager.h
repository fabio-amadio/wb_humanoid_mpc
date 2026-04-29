/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.
******************************************************************************/

#pragma once

#include <algorithm>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

struct HandPoseReference {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector3_t positionInReferenceFrame = vector3_t::Zero();
  quaternion_t orientationReferenceToHand = quaternion_t::Identity();
};

class HandPoseReferenceManager {
 public:
  void setReference(const std::string& name, const HandPoseReference& reference, scalar_t transitionDuration = 0.0);

  std::optional<HandPoseReference> getReference(const std::string& name) const;
  std::optional<HandPoseReference> getReference(const std::string& name, scalar_t time) const;

 private:
  struct TimedHandPoseReference {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    HandPoseReference startReference;
    HandPoseReference goalReference;
    scalar_t transitionStartTime = 0.0;
    scalar_t transitionDuration = 0.0;
    scalar_t lastQueryTime = 0.0;
    bool transitionInitialized = false;
    bool hasLastQueryTime = false;
  };

  static HandPoseReference interpolateReference(const TimedHandPoseReference& reference, scalar_t time);

  mutable std::mutex mutex_;
  mutable std::unordered_map<std::string, TimedHandPoseReference> references_;
};

}  // namespace ocs2::humanoid
