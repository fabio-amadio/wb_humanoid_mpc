/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.
******************************************************************************/

#include "humanoid_common_mpc/reference_manager/HandPoseReferenceManager.h"

namespace ocs2::humanoid {

void HandPoseReferenceManager::setReference(const std::string& name, const HandPoseReference& reference) {
  std::lock_guard<std::mutex> lock(mutex_);
  references_[name] = reference;
}

std::optional<HandPoseReference> HandPoseReferenceManager::getReference(const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = references_.find(name);
  if (it == references_.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace ocs2::humanoid
