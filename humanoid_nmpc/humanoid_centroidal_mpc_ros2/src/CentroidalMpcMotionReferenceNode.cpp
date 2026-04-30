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
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros2_interfaces/common/RosMsgConversions.h>
#include <humanoid_mpc_msgs/msg/mpc_motion_reference.hpp>
#include <ocs2_ros2_msgs/msg/mpc_observation.hpp>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>

#include "humanoid_common_mpc_ros2/ros_comm/MRTPolicySubscriber.h"

using namespace ocs2;
using namespace ocs2::humanoid;

namespace {

constexpr scalar_t kPolicyEvaluationLeadTime = 0.005;
constexpr scalar_t kVelocityFiniteDifferenceTime = 0.02;

template <typename Container>
typename Container::value_type sampleTrajectory(scalar_t time, const scalar_array_t& timeTrajectory, const Container& trajectory) {
  if (timeTrajectory.empty() || trajectory.empty()) {
    throw std::runtime_error("Cannot sample an empty MPC policy trajectory.");
  }
  if (time <= timeTrajectory.front()) {
    return trajectory.front();
  }
  if (time >= timeTrajectory.back()) {
    return trajectory.back();
  }
  return LinearInterpolation::interpolate(time, timeTrajectory, trajectory);
}

Eigen::Quaternion<scalar_t> basePoseToQuaternion(const vector6_t& basePose) {
  const vector3_t eulerZyx = basePose.tail<3>();
  return getQuaternionFromEulerAnglesZyx(eulerZyx);
}

vector3_t angularVelocityFromBasePoseDifference(const vector6_t& startBasePose, const vector6_t& endBasePose, scalar_t duration) {
  const auto startOrientation = basePoseToQuaternion(startBasePose);
  const auto endOrientation = basePoseToQuaternion(endBasePose);
  const auto deltaOrientation = endOrientation * startOrientation.inverse();
  Eigen::AngleAxis<scalar_t> angleAxis(deltaOrientation);
  return angleAxis.axis() * angleAxis.angle() / duration;
}

class CentroidalMpcMotionReferencePublisher {
 public:
  CentroidalMpcMotionReferencePublisher(
      std::string taskFile, std::string urdfFile, std::string referenceFile, std::string topicPrefix, rclcpp::Node::SharedPtr nodeHandle)
      : interface_(taskFile, urdfFile, referenceFile, false),
        nodeHandle_(std::move(nodeHandle)),
        topicPrefix_(std::move(topicPrefix)),
        observationTopic_(topicPrefix_ + "/mpc_observation"),
        motionReferenceTopic_(topicPrefix_ + "/mpc_motion_reference"),
        policySubscriber_(topicPrefix_) {
    const auto& modelSettings = interface_.modelSettings();
    fullJointNames_ = modelSettings.fullJointNames;

    std::unordered_map<std::string, size_t> fullJointIndexByName;
    fullJointIndexByName.reserve(fullJointNames_.size());
    for (size_t i = 0; i < fullJointNames_.size(); ++i) {
      fullJointIndexByName.emplace(fullJointNames_[i], i);
    }

    activeToFullJointIndices_.reserve(modelSettings.mpcModelJointNames.size());
    for (const auto& jointName : modelSettings.mpcModelJointNames) {
      const auto it = fullJointIndexByName.find(jointName);
      if (it == fullJointIndexByName.end()) {
        throw std::runtime_error("Joint " + jointName + " is not part of the full model joint list.");
      }
      activeToFullJointIndices_.push_back(it->second);
    }

    defaultFullJointAngles_ = expandToFullJointVector(interface_.getMpcRobotModel().getJointAngles(interface_.getInitialState()),
                                                      vector_t::Zero(modelSettings.full_joint_dim));

    auto qos = rclcpp::QoS(10);
    qos.best_effort();

    motionReferencePublisher_ = nodeHandle_->create_publisher<humanoid_mpc_msgs::msg::MpcMotionReference>(motionReferenceTopic_, qos);
    policySubscriber_.launchNodes(nodeHandle_, qos);
    observationSubscriber_ = nodeHandle_->create_subscription<ocs2_ros2_msgs::msg::MpcObservation>(
        observationTopic_, qos, std::bind(&CentroidalMpcMotionReferencePublisher::mpcObservationCallback, this, std::placeholders::_1));

    RCLCPP_INFO(nodeHandle_->get_logger(), "Publishing centroidal MPC motion references on %s", motionReferenceTopic_.c_str());
  }

 private:
  vector_t expandToFullJointVector(const vector_t& mpcJointVector, const vector_t& defaultFullJointVector) const {
    vector_t fullJointVector(defaultFullJointVector);
    if (mpcJointVector.size() != static_cast<Eigen::Index>(activeToFullJointIndices_.size())) {
      throw std::runtime_error("MPC joint vector has unexpected dimension.");
    }

    for (size_t i = 0; i < activeToFullJointIndices_.size(); ++i) {
      fullJointVector[activeToFullJointIndices_[i]] = mpcJointVector[i];
    }

    return fullJointVector;
  }

  void mpcObservationCallback(const ocs2_ros2_msgs::msg::MpcObservation::SharedPtr msg) {
    if (!policySubscriber_.initialPolicyReceived()) {
      return;
    }

    const auto observation = ros_msg_conversions::readObservationMsg(*msg);
    activePolicyInitialized_ = policySubscriber_.updatePolicy() || activePolicyInitialized_;
    if (!activePolicyInitialized_) {
      return;
    }

    const auto& policy = policySubscriber_.getPolicy();
    if (policy.timeTrajectory_.empty() || policy.stateTrajectory_.empty() || policy.inputTrajectory_.empty()) {
      return;
    }

    const scalar_t queryTime =
        std::clamp(observation.time + kPolicyEvaluationLeadTime, policy.timeTrajectory_.front(), policy.timeTrajectory_.back());
    const auto policyState = sampleTrajectory(queryTime, policy.timeTrajectory_, policy.stateTrajectory_);
    const auto policyInput = sampleTrajectory(queryTime, policy.timeTrajectory_, policy.inputTrajectory_);

    publishReference(policy, queryTime, policyState, policyInput);
  }

  void publishReference(const PrimalSolution& policy, scalar_t queryTime, const vector_t& policyState, const vector_t& policyInput) {
    const auto& mpcRobotModel = interface_.getMpcRobotModel();

    const auto mpcJointAngles = mpcRobotModel.getJointAngles(policyState);
    const auto mpcJointVelocities = mpcRobotModel.getJointVelocities(policyState, policyInput);
    const auto fullJointAngles = expandToFullJointVector(mpcJointAngles, defaultFullJointAngles_);
    const auto fullJointVelocities = expandToFullJointVector(mpcJointVelocities, vector_t::Zero(interface_.modelSettings().full_joint_dim));

    const vector6_t basePose = mpcRobotModel.getBasePose(policyState);
    const auto orientationBaseToWorld = basePoseToQuaternion(basePose);

    vector3_t rootLinearVelocityWorld = vector3_t::Zero();
    vector3_t rootAngularVelocityWorld = vector3_t::Zero();

    scalar_t nextTime = std::min(queryTime + kVelocityFiniteDifferenceTime, policy.timeTrajectory_.back());
    scalar_t previousTime = std::max(queryTime - kVelocityFiniteDifferenceTime, policy.timeTrajectory_.front());
    if (nextTime > queryTime + 1e-6) {
      const auto nextState = sampleTrajectory(nextTime, policy.timeTrajectory_, policy.stateTrajectory_);
      const vector6_t nextBasePose = mpcRobotModel.getBasePose(nextState);
      const scalar_t dt = nextTime - queryTime;
      rootLinearVelocityWorld = (nextBasePose.head<3>() - basePose.head<3>()) / dt;
      rootAngularVelocityWorld = angularVelocityFromBasePoseDifference(basePose, nextBasePose, dt);
    } else if (queryTime > previousTime + 1e-6) {
      const auto previousState = sampleTrajectory(previousTime, policy.timeTrajectory_, policy.stateTrajectory_);
      const vector6_t previousBasePose = mpcRobotModel.getBasePose(previousState);
      const scalar_t dt = queryTime - previousTime;
      rootLinearVelocityWorld = (basePose.head<3>() - previousBasePose.head<3>()) / dt;
      rootAngularVelocityWorld = angularVelocityFromBasePoseDifference(previousBasePose, basePose, dt);
    }

    const vector3_t rootLinearVelocityBase = orientationBaseToWorld.inverse() * rootLinearVelocityWorld;
    const vector3_t rootAngularVelocityBase = orientationBaseToWorld.inverse() * rootAngularVelocityWorld;

    humanoid_mpc_msgs::msg::MpcMotionReference referenceMsg;
    referenceMsg.header.stamp = nodeHandle_->now();
    referenceMsg.header.frame_id = "world";

    referenceMsg.joint_names = fullJointNames_;
    referenceMsg.joint_pos.assign(fullJointAngles.data(), fullJointAngles.data() + fullJointAngles.size());
    referenceMsg.joint_vel.assign(fullJointVelocities.data(), fullJointVelocities.data() + fullJointVelocities.size());

    referenceMsg.root_pose_w.position.x = basePose[0];
    referenceMsg.root_pose_w.position.y = basePose[1];
    referenceMsg.root_pose_w.position.z = basePose[2];
    referenceMsg.root_pose_w.orientation.x = orientationBaseToWorld.x();
    referenceMsg.root_pose_w.orientation.y = orientationBaseToWorld.y();
    referenceMsg.root_pose_w.orientation.z = orientationBaseToWorld.z();
    referenceMsg.root_pose_w.orientation.w = orientationBaseToWorld.w();

    referenceMsg.root_twist_w.linear.x = rootLinearVelocityWorld.x();
    referenceMsg.root_twist_w.linear.y = rootLinearVelocityWorld.y();
    referenceMsg.root_twist_w.linear.z = rootLinearVelocityWorld.z();
    referenceMsg.root_twist_w.angular.x = rootAngularVelocityWorld.x();
    referenceMsg.root_twist_w.angular.y = rootAngularVelocityWorld.y();
    referenceMsg.root_twist_w.angular.z = rootAngularVelocityWorld.z();

    referenceMsg.motion_cmd.reserve(2 * fullJointAngles.size() + 6);
    for (Eigen::Index i = 0; i < fullJointAngles.size(); ++i) {
      referenceMsg.motion_cmd.push_back(static_cast<float>(fullJointAngles[i]));
    }
    for (Eigen::Index i = 0; i < fullJointVelocities.size(); ++i) {
      referenceMsg.motion_cmd.push_back(static_cast<float>(fullJointVelocities[i]));
    }
    referenceMsg.motion_cmd.push_back(static_cast<float>(rootLinearVelocityBase.x()));
    referenceMsg.motion_cmd.push_back(static_cast<float>(rootLinearVelocityBase.y()));
    referenceMsg.motion_cmd.push_back(static_cast<float>(rootAngularVelocityBase.z()));
    referenceMsg.motion_cmd.push_back(static_cast<float>(basePose[2]));
    referenceMsg.motion_cmd.push_back(static_cast<float>(basePose[5]));
    referenceMsg.motion_cmd.push_back(static_cast<float>(basePose[4]));

    motionReferencePublisher_->publish(referenceMsg);
  }

  CentroidalMpcInterface interface_;
  rclcpp::Node::SharedPtr nodeHandle_;
  std::string topicPrefix_;
  std::string observationTopic_;
  std::string motionReferenceTopic_;
  MRTPolicySubscriber policySubscriber_;
  rclcpp::Publisher<humanoid_mpc_msgs::msg::MpcMotionReference>::SharedPtr motionReferencePublisher_;
  rclcpp::Subscription<ocs2_ros2_msgs::msg::MpcObservation>::SharedPtr observationSubscriber_;

  std::vector<std::string> fullJointNames_;
  std::vector<size_t> activeToFullJointIndices_;
  vector_t defaultFullJointAngles_;
  bool activePolicyInitialized_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 5) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);
  const std::string taskFile(argv[2]);
  const std::string referenceFile(argv[3]);
  const std::string urdfFile(argv[4]);

  rclcpp::init(argc, argv);

  auto nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_mpc_motion_reference_publisher");
  CentroidalMpcMotionReferencePublisher referencePublisher(taskFile, urdfFile, referenceFile, robotName, nodeHandle);

  rclcpp::spin(nodeHandle);
  rclcpp::shutdown();
  return 0;
}
