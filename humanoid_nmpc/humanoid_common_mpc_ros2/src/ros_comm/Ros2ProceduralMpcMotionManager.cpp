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

#include <rclcpp/rclcpp.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <functional>
#include <mutex>
#include <utility>

#include "humanoid_common_mpc_ros2/ros_comm/Ros2ProceduralMpcMotionManager.h"

namespace ocs2::humanoid {

Ros2ProceduralMpcMotionManager::Ros2ProceduralMpcMotionManager(
    const std::string& gaitFile,
    const std::string& referenceFile,
    std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
    const MpcRobotModelBase<scalar_t>& mpcRobotModel,
    VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories,
    std::string robotName)
    : ProceduralMpcMotionManager(
          gaitFile, referenceFile, switchedModelReferenceManagerPtr, mpcRobotModel, velocityTargetToTargetTrajectories),
      robotName_(std::move(robotName)) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(referenceFile, pt);
  if (const auto handReferenceTransitionDuration = pt.get_optional<scalar_t>("handReferenceTransitionDuration")) {
    handReferenceTransitionDuration_ = std::max<scalar_t>(*handReferenceTransitionDuration, 0.0);
  }
  if (const auto waistReferenceTransitionDuration = pt.get_optional<scalar_t>("waistReferenceTransitionDuration")) {
    waistReferenceTransitionDuration_ = std::max<scalar_t>(*waistReferenceTransitionDuration, 0.0);
  }
  if (const auto waistYawMin = pt.get_optional<scalar_t>("waistYawBounds.min")) {
    waistYawBounds_.min = *waistYawMin;
  }
  if (const auto waistYawMax = pt.get_optional<scalar_t>("waistYawBounds.max")) {
    waistYawBounds_.max = *waistYawMax;
  }
  for (const std::string& handName : {"left_hand", "right_hand"}) {
    HandPositionBounds bounds;
    const std::string fieldPrefix = "handPositionBounds." + handName;
    if (const auto optionalMinX = pt.get_optional<scalar_t>(fieldPrefix + ".x_min")) {
      bounds.min.x() = *optionalMinX;
    }
    if (const auto optionalMaxX = pt.get_optional<scalar_t>(fieldPrefix + ".x_max")) {
      bounds.max.x() = *optionalMaxX;
    }
    if (const auto optionalMinY = pt.get_optional<scalar_t>(fieldPrefix + ".y_min")) {
      bounds.min.y() = *optionalMinY;
    }
    if (const auto optionalMaxY = pt.get_optional<scalar_t>(fieldPrefix + ".y_max")) {
      bounds.max.y() = *optionalMaxY;
    }
    if (const auto optionalMinZ = pt.get_optional<scalar_t>(fieldPrefix + ".z_min")) {
      bounds.min.z() = *optionalMinZ;
    }
    if (const auto optionalMaxZ = pt.get_optional<scalar_t>(fieldPrefix + ".z_max")) {
      bounds.max.z() = *optionalMaxZ;
    }
    handPositionBounds_.emplace(handName, bounds);
  }
}

void Ros2ProceduralMpcMotionManager::setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) {
  std::lock_guard<std::mutex> lock(walkingVelCommandMutex_);
  velocityCommand_ = scaleWalkingVelocityCommand(rawVelocityCommand);
  const scalar_t clampedWaistYaw = clampWaistYaw(rawVelocityCommand.desired_waist_yaw);
  if (!waistYawReferenceInitialized_) {
    waistYawReferenceInitialized_ = true;
    waistYawTransitionStartTime_ = currentSolverTime_;
    waistYawStartReference_ = clampedWaistYaw;
    waistYawGoalReference_ = clampedWaistYaw;
  } else {
    waistYawStartReference_ = interpolateWaistYaw(currentSolverTime_);
    waistYawGoalReference_ = clampedWaistYaw;
    waistYawTransitionStartTime_ = currentSolverTime_;
  }
  velocityCommand_.desired_waist_yaw = clampedWaistYaw;
}

void Ros2ProceduralMpcMotionManager::subscribe(rclcpp::Node::SharedPtr nodeHandle, const rclcpp::QoS& qos) {
  // ModeSchedule

  // TargetTrajectories
  auto walkingVelocityCallback = [this](const humanoid_mpc_msgs::msg::WalkingVelocityCommand::SharedPtr msg) {
    this->setAndScaleVelocityCommand(getWalkingVelocityCommandFromMsg(*msg));
  };
  velCommandSubscriber_ = nodeHandle->create_subscription<humanoid_mpc_msgs::msg::WalkingVelocityCommand>(
      "humanoid/walking_velocity_command", qos, walkingVelocityCallback);

  leftHandPoseSubscriber_ = nodeHandle->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/" + robotName_ + "/left_hand_pose_reference", qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->setHandPoseReference("left_hand", *msg); });
  rightHandPoseSubscriber_ = nodeHandle->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/" + robotName_ + "/right_hand_pose_reference", qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->setHandPoseReference("right_hand", *msg); });
}

void Ros2ProceduralMpcMotionManager::setHandPoseReference(const std::string& referenceName, const geometry_msgs::msg::PoseStamped& msg) {
  HandPoseReference reference;
  reference.positionInReferenceFrame << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  reference.positionInReferenceFrame = clampHandPosition(referenceName, reference.positionInReferenceFrame);
  reference.orientationReferenceToHand =
      quaternion_t(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z).normalized();
  switchedModelReferenceManagerPtr_->getHandPoseReferenceManagerPtr()->setReference(referenceName, reference, handReferenceTransitionDuration_);
}

vector3_t Ros2ProceduralMpcMotionManager::clampHandPosition(const std::string& referenceName,
                                                            const vector3_t& positionInReferenceFrame) const {
  const auto it = handPositionBounds_.find(referenceName);
  if (it == handPositionBounds_.end()) {
    return positionInReferenceFrame;
  }

  vector3_t clampedPosition = positionInReferenceFrame;
  clampedPosition.x() = std::clamp(clampedPosition.x(), it->second.min.x(), it->second.max.x());
  clampedPosition.y() = std::clamp(clampedPosition.y(), it->second.min.y(), it->second.max.y());
  clampedPosition.z() = std::clamp(clampedPosition.z(), it->second.min.z(), it->second.max.z());
  if (!clampedPosition.isApprox(positionInReferenceFrame)) {
    auto logger = rclcpp::get_logger("Ros2ProceduralMpcMotionManager");
    rclcpp::Clock steadyClock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(logger, steadyClock, 1000,
                         "Clamped %s hand reference from [%.3f %.3f %.3f] to [%.3f %.3f %.3f] in torso frame.",
                         referenceName.c_str(), positionInReferenceFrame.x(), positionInReferenceFrame.y(), positionInReferenceFrame.z(),
                         clampedPosition.x(), clampedPosition.y(), clampedPosition.z());
  }
  return clampedPosition;
}

scalar_t Ros2ProceduralMpcMotionManager::clampWaistYaw(scalar_t desiredWaistYaw) const {
  constexpr scalar_t kRadToDeg = 180.0 / 3.14159265358979323846;
  constexpr scalar_t kClampTolerance = 1e-6;
  const scalar_t clampedWaistYaw = std::clamp(desiredWaistYaw, waistYawBounds_.min, waistYawBounds_.max);
  if (std::abs(clampedWaistYaw - desiredWaistYaw) > kClampTolerance) {
    auto logger = rclcpp::get_logger("Ros2ProceduralMpcMotionManager");
    rclcpp::Clock steadyClock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(logger, steadyClock, 1000,
                         "Clamped waist yaw reference from %.1f deg to %.1f deg.",
                         desiredWaistYaw * kRadToDeg, clampedWaistYaw * kRadToDeg);
  }
  return clampedWaistYaw;
}

scalar_t Ros2ProceduralMpcMotionManager::interpolateWaistYaw(scalar_t time) const {
  if (!waistYawReferenceInitialized_ || waistReferenceTransitionDuration_ <= 1e-9) {
    return waistYawGoalReference_;
  }

  const scalar_t alpha = std::clamp((time - waistYawTransitionStartTime_) / waistReferenceTransitionDuration_, scalar_t(0.0), scalar_t(1.0));
  return (1.0 - alpha) * waistYawStartReference_ + alpha * waistYawGoalReference_;
}

WalkingVelocityCommand Ros2ProceduralMpcMotionManager::getScaledWalkingVelocityCommand(scalar_t time) {
  std::lock_guard<std::mutex> lock(walkingVelCommandMutex_);
  currentSolverTime_ = time;
  if (!waistYawReferenceInitialized_) {
    waistYawReferenceInitialized_ = true;
    waistYawTransitionStartTime_ = time;
    waistYawStartReference_ = velocityCommand_.desired_waist_yaw;
    waistYawGoalReference_ = velocityCommand_.desired_waist_yaw;
  }
  WalkingVelocityCommand command = velocityCommand_;
  command.desired_waist_yaw = interpolateWaistYaw(time);
  return command;
}

}  // namespace ocs2::humanoid
