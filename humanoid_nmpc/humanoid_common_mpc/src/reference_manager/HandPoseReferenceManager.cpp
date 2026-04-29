/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.
******************************************************************************/

#include "humanoid_common_mpc/reference_manager/HandPoseReferenceManager.h"

namespace ocs2::humanoid {

HandPoseReference HandPoseReferenceManager::interpolateReference(const TimedHandPoseReference& reference, scalar_t time) {
  if (reference.transitionDuration <= 0.0) {
    return reference.goalReference;
  }

  const scalar_t alpha = std::clamp((time - reference.transitionStartTime) / reference.transitionDuration, scalar_t(0.0), scalar_t(1.0));

  HandPoseReference interpolatedReference;
  interpolatedReference.positionInReferenceFrame =
      (1.0 - alpha) * reference.startReference.positionInReferenceFrame + alpha * reference.goalReference.positionInReferenceFrame;
  interpolatedReference.orientationReferenceToHand =
      reference.startReference.orientationReferenceToHand.slerp(alpha, reference.goalReference.orientationReferenceToHand).normalized();
  return interpolatedReference;
}

void HandPoseReferenceManager::setReference(const std::string& name, const HandPoseReference& reference, scalar_t transitionDuration) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto [it, inserted] = references_.try_emplace(name);
  auto& timedReference = it->second;
  const HandPoseReference startReference =
      inserted ? reference
               : (timedReference.hasLastQueryTime ? interpolateReference(timedReference, timedReference.lastQueryTime)
                                                  : timedReference.goalReference);

  timedReference.startReference = startReference;
  timedReference.goalReference = reference;
  timedReference.transitionDuration = std::max<scalar_t>(transitionDuration, 0.0);
  timedReference.transitionInitialized = timedReference.transitionDuration <= 0.0;

  if (timedReference.transitionInitialized) {
    timedReference.transitionStartTime = timedReference.hasLastQueryTime ? timedReference.lastQueryTime : 0.0;
    timedReference.startReference = reference;
  }
}

std::optional<HandPoseReference> HandPoseReferenceManager::getReference(const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = references_.find(name);
  if (it == references_.end()) {
    return std::nullopt;
  }
  if (it->second.hasLastQueryTime) {
    return interpolateReference(it->second, it->second.lastQueryTime);
  }
  return it->second.goalReference;
}

std::optional<HandPoseReference> HandPoseReferenceManager::getReference(const std::string& name, scalar_t time) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = references_.find(name);
  if (it == references_.end()) {
    return std::nullopt;
  }

  auto& timedReference = it->second;
  if (!timedReference.transitionInitialized) {
    timedReference.transitionStartTime = time;
    timedReference.transitionInitialized = true;
  }

  const auto reference = interpolateReference(timedReference, time);
  timedReference.lastQueryTime = time;
  timedReference.hasLastQueryTime = true;
  return reference;
}

}  // namespace ocs2::humanoid
