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

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ocs2_ros2_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros2_msgs/msg/mpc_observation.hpp>
#include <ocs2_core/misc/LinearInterpolation.h>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>

#include "humanoid_common_mpc_ros2/ros_comm/MRTPolicySubscriber.h"

using namespace ocs2;
using namespace ocs2::humanoid;

namespace {

constexpr scalar_t kPolicyEvaluationLeadTime = 0.005;

class CentroidalReferenceJointStatePublisher {
 public:
  CentroidalReferenceJointStatePublisher(std::string taskFile,
                                         std::string urdfFile,
                                         std::string referenceFile,
                                         std::string topicPrefix,
                                         rclcpp::Node::SharedPtr nodeHandle)
      : interface_(taskFile, urdfFile, referenceFile, false),
        nodeHandle_(std::move(nodeHandle)),
        topicPrefix_(std::move(topicPrefix)),
        observationTopic_(topicPrefix_ + "/mpc_observation"),
        referenceJointStateTopic_(topicPrefix_ + "/reference_joint_states"),
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

    defaultFullJointAngles_ = vector_t::Zero(modelSettings.full_joint_dim);
    defaultFullJointAngles_ =
        expandToFullJointVector(interface_.getMpcRobotModel().getJointAngles(interface_.getInitialState()), defaultFullJointAngles_);

    auto qos = rclcpp::QoS(10);
    qos.best_effort();

    referenceJointStatePublisher_ = nodeHandle_->create_publisher<sensor_msgs::msg::JointState>(referenceJointStateTopic_, qos);
    policySubscriber_.launchNodes(nodeHandle_, qos);
    observationSubscriber_ = nodeHandle_->create_subscription<ocs2_ros2_msgs::msg::MpcObservation>(
        observationTopic_, qos,
        std::bind(&CentroidalReferenceJointStatePublisher::mpcObservationCallback, this, std::placeholders::_1));

    RCLCPP_INFO(nodeHandle_->get_logger(), "Publishing centroidal references on %s using MPC topics %s and %s",
                referenceJointStateTopic_.c_str(), observationTopic_.c_str(), (topicPrefix_ + "/mpc_policy").c_str());
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
    const scalar_t queryTime = observation.time + kPolicyEvaluationLeadTime;

    vector_t policyState = queryTime <= policy.timeTrajectory_.back()
                               ? LinearInterpolation::interpolate(queryTime, policy.timeTrajectory_, policy.stateTrajectory_)
                               : policy.stateTrajectory_.back();
    vector_t policyInput = queryTime <= policy.timeTrajectory_.back()
                               ? LinearInterpolation::interpolate(queryTime, policy.timeTrajectory_, policy.inputTrajectory_)
                               : policy.inputTrajectory_.back();

    const auto mpcJointAngles = interface_.getMpcRobotModel().getJointAngles(policyState);
    const auto mpcJointVelocities = interface_.getMpcRobotModel().getJointVelocities(policyState, policyInput);

    const auto fullJointAngles = expandToFullJointVector(mpcJointAngles, defaultFullJointAngles_);
    const auto fullJointVelocities = expandToFullJointVector(
        mpcJointVelocities, vector_t::Zero(interface_.modelSettings().full_joint_dim));

    sensor_msgs::msg::JointState jointStateMsg;
    jointStateMsg.header.stamp = nodeHandle_->now();
    jointStateMsg.name = fullJointNames_;
    jointStateMsg.position.assign(fullJointAngles.data(), fullJointAngles.data() + fullJointAngles.size());
    jointStateMsg.velocity.assign(fullJointVelocities.data(), fullJointVelocities.data() + fullJointVelocities.size());

    referenceJointStatePublisher_->publish(jointStateMsg);
  }

  CentroidalMpcInterface interface_;
  rclcpp::Node::SharedPtr nodeHandle_;
  std::string topicPrefix_;
  std::string observationTopic_;
  std::string referenceJointStateTopic_;
  MRTPolicySubscriber policySubscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr referenceJointStatePublisher_;
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

  auto nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_reference_joint_state_publisher");
  CentroidalReferenceJointStatePublisher referencePublisher(taskFile, urdfFile, referenceFile, robotName, nodeHandle);

  rclcpp::spin(nodeHandle);
  rclcpp::shutdown();
  return 0;
}
