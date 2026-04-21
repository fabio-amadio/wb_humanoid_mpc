/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.
******************************************************************************/

#pragma once

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
  void setReference(const std::string& name, const HandPoseReference& reference);

  std::optional<HandPoseReference> getReference(const std::string& name) const;

 private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, HandPoseReference> references_;
};

}  // namespace ocs2::humanoid
