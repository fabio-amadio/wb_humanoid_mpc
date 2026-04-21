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
      robotName_(std::move(robotName)) {}

void Ros2ProceduralMpcMotionManager::setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) {
  std::lock_guard<std::mutex> lock(walkingVelCommandMutex_);
  velocityCommand_ = scaleWalkingVelocityCommand(rawVelocityCommand);
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
  reference.orientationReferenceToHand =
      quaternion_t(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z).normalized();
  switchedModelReferenceManagerPtr_->getHandPoseReferenceManagerPtr()->setReference(referenceName, reference);
}

WalkingVelocityCommand Ros2ProceduralMpcMotionManager::getScaledWalkingVelocityCommand() {
  std::lock_guard<std::mutex> lock(walkingVelCommandMutex_);
  return velocityCommand_;
}

}  // namespace ocs2::humanoid
