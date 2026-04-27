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

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>

#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizer.h"

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros2_interfaces/visualization/VisualizationHelpers.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ocs2::humanoid {

namespace {

constexpr const char* kMpcFramePrefix = "mpc/";

geometry_msgs::msg::Pose toPoseMsg(const vector3_t& position, const quaternion_t& orientation) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

visualization_msgs::msg::InteractiveMarkerControl makeAxisControl(const std::string& name,
                                                                  uint8_t interactionMode,
                                                                  double x,
                                                                  double y,
                                                                  double z) {
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.name = name;
  control.orientation.w = 1.0;
  control.orientation.x = x;
  control.orientation.y = y;
  control.orientation.z = z;
  control.interaction_mode = interactionMode;
  return control;
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidVisualizer::HumanoidVisualizer(const std::string& taskFile,
                                       PinocchioInterface pinocchioInterface,
                                       const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                       rclcpp::Node::SharedPtr nodeHandle,
                                       scalar_t maxUpdateFrequency)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      mpcRobotModelPtr_(&mpcRobotModel),
      contactVisualizer_(taskFile, pinocchioInterface_, mpcRobotModel),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency),
      base_link_name_(pinocchioInterface_.getModel()
                          .frames[((pinocchio::FrameIndex)2)]
                          .name),  // [0] universe frame, [1] root joint frame, [2] root link frame.
      node_handle_(std::move(nodeHandle)) {
  // launchSubscribers();
  createVisualizationPublishers();
  initializeHandInteractiveMarkers(taskFile);
  prevPolicyState = vector_t::Zero(mpcRobotModelPtr_->getStateDim());
  prevPolicyInput = vector_t::Zero(mpcRobotModelPtr_->getInputDim());

  collisionConfig_ = FootCollisionConstraint::loadFootCollisionConstraintConfig(taskFile);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::launchSubscribers() {
  auto qos = rclcpp::QoS(1);
  qos.best_effort();
  observationSubscriberPtr_ = node_handle_->create_subscription<ocs2_ros2_msgs::msg::MpcObservation>(
      "/humanoid/mpc_observation", qos, std::bind(&HumanoidVisualizer::mpcObservationCallback, this, std::placeholders::_1));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::mpcObservationCallback(const ocs2_ros2_msgs::msg::MpcObservation::SharedPtr msg) {
  const auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);
  update(currentObservation, PrimalSolution(), CommandData());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::createVisualizationPublishers() {
  tfBroadcasterPtr_.reset(new tf2_ros::TransformBroadcaster(node_handle_));
  jointPublisherPtr_ = node_handle_->create_publisher<sensor_msgs::msg::JointState>("mpc/joint_states", 1);
  terminalJointPublisherPtr_ =
      node_handle_->create_publisher<sensor_msgs::msg::JointState>("terminal_state/joint_states", 1);
  terminalJointTargetPublisherPtr_ =
      node_handle_->create_publisher<sensor_msgs::msg::JointState>("terminal_target/joint_states", 1);
  markerPublisherPtr_ = node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>("cartesian_markers", 1);
  collsisionMarkerPublisherPtr_ = node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>("collision_markers", 1);
  stateOptimizedPublisherPtr_ = node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>("optimized_state_markers", 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::initializeHandInteractiveMarkers(const std::string& taskFile) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  const auto taskSpaceCostsOptional = pt.get_child_optional("task_space_costs");
  if (!taskSpaceCostsOptional) {
    return;
  }

  const auto maybeAddHandMarker = [this, &taskSpaceCostsOptional](const std::string& referenceName, const std::string& colorName,
                                                                  const Color& color) {
    const auto handTaskOptional = taskSpaceCostsOptional->get_child_optional(referenceName);
    if (!handTaskOptional) {
      return;
    }

    const bool usePelvisFrameReference = handTaskOptional->get<bool>("pelvis_frame_pose_reference", false);
    const std::string linkName = handTaskOptional->get<std::string>("link_name", "");
    const std::string referenceFrameName = handTaskOptional->get<std::string>("reference_frame_link_name", base_link_name_);
    if (!usePelvisFrameReference || linkName.empty() || !pinocchioInterface_.getModel().existFrame(linkName)) {
      return;
    }

    if (!handInteractiveMarkerServerPtr_) {
      handInteractiveMarkerServerPtr_ =
          std::make_unique<interactive_markers::InteractiveMarkerServer>("hand_pose_markers", node_handle_);
    }

    HandInteractiveMarker handMarker;
    handMarker.referenceName = referenceName;
    handMarker.markerName = referenceName + "_interactive_marker";
    handMarker.linkName = linkName;
    handMarker.referenceFrameName = referenceFrameName;
    handMarker.visualReferenceFrameName = std::string(kMpcFramePrefix) + referenceFrameName;
    handMarker.publisher = node_handle_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/" + mpcRobotModelPtr_->modelSettings.robotName + "/" + referenceName + "_pose_reference", 1);

    handInteractiveMarkers_.push_back(std::move(handMarker));

    RCLCPP_INFO(node_handle_->get_logger(), "Enabled %s interactive marker for link `%s` on topic `%s`.", colorName.c_str(),
                linkName.c_str(), ("/" + mpcRobotModelPtr_->modelSettings.robotName + "/" + referenceName + "_pose_reference").c_str());
  };

  maybeAddHandMarker("left_hand", "left hand", Color::blue);
  maybeAddHandMarker("right_hand", "right hand", Color::orange);

  if (handInteractiveMarkerServerPtr_) {
    handInteractiveMarkerServerPtr_->applyChanges();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::updateHandInteractiveMarkers() {
  if (!handInteractiveMarkerServerPtr_) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  const auto& data = pinocchioInterface_.getData();
  std::scoped_lock lock(handInteractiveMarkersMutex_);
  bool didChange = false;

  for (auto& handMarker : handInteractiveMarkers_) {
    if (handMarker.initialized) {
      continue;
    }

    const pinocchio::FrameIndex referenceFrameIndex = model.getFrameId(handMarker.referenceFrameName);
    const pinocchio::FrameIndex handFrameIndex = model.getFrameId(handMarker.linkName);
    const pinocchio::SE3 referenceToHand = data.oMf[referenceFrameIndex].inverse() * data.oMf[handFrameIndex];
    const quaternion_t referenceOrientationToHand(referenceToHand.rotation());
    handMarker.poseInReferenceFrame = toPoseMsg(referenceToHand.translation(), referenceOrientationToHand.normalized());

    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = handMarker.visualReferenceFrameName;
    marker.name = handMarker.markerName;
    marker.description = handMarker.referenceName + " pose";
    marker.scale = 0.14;
    marker.pose = handMarker.poseInReferenceFrame;

    visualization_msgs::msg::InteractiveMarkerControl visualControl;
    visualControl.always_visible = true;
    visualControl.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
    visualization_msgs::msg::Marker sphere;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = 0.035;
    sphere.scale.y = 0.035;
    sphere.scale.z = 0.035;
    sphere.color = getColor(handMarker.referenceName == "left_hand" ? Color::blue : Color::orange);
    sphere.color.a = 0.85;
    visualControl.markers.push_back(sphere);
    marker.controls.push_back(visualControl);

    marker.controls.push_back(makeAxisControl("move_x", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, 1.0, 0.0, 0.0));
    marker.controls.push_back(
        makeAxisControl("move_y", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, 0.0, 0.0, 1.0));
    marker.controls.push_back(
        makeAxisControl("move_z", visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS, 0.0, 1.0, 0.0));
    marker.controls.push_back(
        makeAxisControl("rotate_x", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 1.0, 0.0, 0.0));
    marker.controls.push_back(
        makeAxisControl("rotate_y", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 0.0, 0.0, 1.0));
    marker.controls.push_back(
        makeAxisControl("rotate_z", visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS, 0.0, 1.0, 0.0));

    handInteractiveMarkerServerPtr_->insert(
        marker, [this, markerName = handMarker.markerName](
                    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
          this->processHandInteractiveMarkerFeedback(markerName, feedback);
        });

    handMarker.initialized = true;
    didChange = true;

    geometry_msgs::msg::PoseStamped referenceMsg;
    referenceMsg.header.stamp = node_handle_->now();
    referenceMsg.header.frame_id = handMarker.visualReferenceFrameName;
    referenceMsg.pose = handMarker.poseInReferenceFrame;
    handMarker.publisher->publish(referenceMsg);
  }

  if (didChange) {
    handInteractiveMarkerServerPtr_->applyChanges();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::processHandInteractiveMarkerFeedback(
    const std::string& markerName, const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  if (!feedback || (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE &&
                    feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)) {
    return;
  }

  std::scoped_lock lock(handInteractiveMarkersMutex_);
  for (auto& handMarker : handInteractiveMarkers_) {
    if (handMarker.markerName != markerName) {
      continue;
    }

    handMarker.poseInReferenceFrame = feedback->pose;
    handMarker.initialized = true;

    geometry_msgs::msg::PoseStamped referenceMsg;
    referenceMsg.header.stamp = feedback->header.stamp;
    referenceMsg.header.frame_id =
        feedback->header.frame_id.empty() ? handMarker.visualReferenceFrameName : feedback->header.frame_id;
    referenceMsg.pose = feedback->pose;
    handMarker.publisher->publish(referenceMsg);
    break;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::publishObservation(const SystemObservation& observation) const {
  vector_t state(observation.state);
  publishBaseTransform(mpcRobotModelPtr_->getBasePose(state), "mpc/");
  publishJointTransforms(mpcRobotModelPtr_->getJointAngles(state), jointPublisherPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::publishJointTransforms(const vector_t& jointAngles,
                                                rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisherPtr) const {
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = node_handle_->now();
  joint_state.name.resize(mpcRobotModelPtr_->getFullModelJointDim());
  joint_state.position.resize(mpcRobotModelPtr_->getFullModelJointDim());

  std::vector<std::string> jointNamesVector(mpcRobotModelPtr_->modelSettings.fullJointNames.begin(),
                                            mpcRobotModelPtr_->modelSettings.fullJointNames.end());
  vector_t fullJointPoisitions =
      mpcRobotModelPtr_->getFullModelJointAngles(jointAngles, vector_t::Zero(mpcRobotModelPtr_->modelSettings.full_joint_dim));
  std::vector<double> jointPositionsVector(fullJointPoisitions.data(), fullJointPoisitions.data() + fullJointPoisitions.size());
  joint_state.name = jointNamesVector;
  joint_state.position = jointPositionsVector;

  jointPublisherPtr->publish(joint_state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::publishBaseTransform(const vector6_t& basePose, const std::string prefix) const {
  const Eigen::Quaternion<scalar_t> quaternionBaseToWorld = getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_handle_->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = prefix + base_link_name_;
  transform.transform.translation.x = basePose[0];
  transform.transform.translation.y = basePose[1];
  transform.transform.translation.z = basePose[2];
  transform.transform.rotation.x = quaternionBaseToWorld.x();
  transform.transform.rotation.y = quaternionBaseToWorld.y();
  transform.transform.rotation.z = quaternionBaseToWorld.z();
  transform.transform.rotation.w = quaternionBaseToWorld.w();
  tfBroadcasterPtr_->sendTransform(transform);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::updatePinocchioFrames(const vector_t& state) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, mpcRobotModelPtr_->getGeneralizedCoordinates(state));
  // for (int i = 0; i < N_CONTACTS; i++) {
  //   pinocchio::updateFramePlacement(model, data, contactFrameIndices[i]);
  // }
  pinocchio::updateFramePlacements(model, data);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::publishCartesianMarkers(const contact_flag_t& contactFlags, const vector_t& state, const vector_t& input) const {
  // Reserve message

  // Reserve message
  const size_t numberOfCartesianMarkers = N_CONTACTS * N_CONTACT_POLYGON_POINTS + 3;
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCartesianMarkers);

  std::vector<vector3_t> copPositions = computeContactsCoP(input, pinocchioInterface_, contactFlags, *mpcRobotModelPtr_);
  std::vector<vector3_t> contactForces;
  contactForces.reserve(N_CONTACTS);

  // Feet positions and Forces
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    vector3_t contactForce = mpcRobotModelPtr_->getContactForce(input, i);
    // markerArray.markers.emplace_back(
    //     getFootMarker(contactPositions[i], contactFlags[i], feetColorMap_[i], footMarkerDiameter_, footAlphaWhenLifted_));
    markerArray.markers.emplace_back(getForceMarker(contactForce, copPositions[i], contactFlags[i], Color::green, forceScale_));
    contactForces.emplace_back(contactForce);
  }

  // Center of pressure
  markerArray.markers.emplace_back(getCenterOfPressureMarker(contactForces.begin(), contactForces.end(), copPositions.begin(),
                                                             contactFlags.begin(), Color::green, copMarkerDiameter_));

  // Add the visualization of the 4 fources equal to the contact wrench
  visualization_msgs::msg::MarkerArray wrenchVisualizationForcesMarkerArray(
      contactVisualizer_.generateContactVisualizationForceMarkers(input, contactFlags, forceScale_));
  for (auto& marker : wrenchVisualizationForcesMarkerArray.markers) {
    markerArray.markers.emplace_back(marker);
  }

  // Give markers an id and a frame
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg("world", node_handle_->now()));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  markerPublisherPtr_->publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::publishSelfCollisionMarkers(const contact_flag_t& contactFlags, const vector_t& state) const {
  std::vector<std::string> collisionFrameNames = {collisionConfig_.leftAnkleFrame,      collisionConfig_.rightAnkleFrame,
                                                  collisionConfig_.leftFootCenterFrame, collisionConfig_.rightFootCenterFrame,
                                                  collisionConfig_.leftFootFrame1,      collisionConfig_.rightFootFrame1,
                                                  collisionConfig_.leftFootFrame2,      collisionConfig_.rightFootFrame2,
                                                  collisionConfig_.leftKneeFrame,       collisionConfig_.rightKneeFrame};

  // Reserve message
  const size_t numberOfCollisionMarkers = collisionFrameNames.size();
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(numberOfCollisionMarkers);

  std::vector<scalar_t> collisionSphereRadius = {collisionConfig_.footCollisionSphereRadius, collisionConfig_.footCollisionSphereRadius,
                                                 collisionConfig_.footCollisionSphereRadius, collisionConfig_.footCollisionSphereRadius,
                                                 collisionConfig_.footCollisionSphereRadius, collisionConfig_.footCollisionSphereRadius,
                                                 collisionConfig_.footCollisionSphereRadius, collisionConfig_.footCollisionSphereRadius,
                                                 collisionConfig_.kneeCollisionSphereRadius, collisionConfig_.kneeCollisionSphereRadius};

  std::vector<vector3_t> collisionPositions = getFramePositions<scalar_t>(pinocchioInterface_, collisionFrameNames);

  for (size_t i = 0; i < numberOfCollisionMarkers; ++i) {
    markerArray.markers.emplace_back(getSphereMsg(collisionPositions[i], Color::red, 2 * collisionSphereRadius[i]));
    markerArray.markers.back().color.a = 0.5;
  }

  // Give markers an id and a frame
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg("world", node_handle_->now()));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  // Publish cartesian markers (minus the CoM Pose)
  collsisionMarkerPublisherPtr_->publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::publishOptimizedStateTrajectory(const scalar_array_t& mpcTimeTrajectory,
                                                         const vector_array_t& mpcStateTrajectory,
                                                         const ModeSchedule& modeSchedule,
                                                         const TargetTrajectories& targetTrajectories,
                                                         scalar_t groundHeight) const {
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
    return;  // Nothing to publish
  }

  PinocchioInterface stateTrajectoryPinocchioInterface = pinocchioInterface_;

  // Create name of all visualized frame position by checking if each of the frame name from a list of full frame names exist in the model
  // This is required to run the visualization on a model that does not contain all the frames
  static std::vector<std::string> frameNames = [&]() {
    std::vector<std::string> fullFrameNames = {"foot_l_contact", "foot_r_contact", "mid360_link", "left_rubber_hand",
                                               "right_rubber_hand"};  // ToDo: generalize this
    std::vector<std::string> frameNames;
    frameNames.reserve(2);  // Feet and should always be present
    for (std::string& frameName : fullFrameNames) {
      if (stateTrajectoryPinocchioInterface.getModel().existFrame(frameName)) frameNames.push_back(frameName);
    }
    return frameNames;
  }();

  // Reserve Feet msg
  std::vector<std::vector<geometry_msgs::msg::Point>> frameMsgs(frameNames.size());
  std::for_each(frameMsgs.begin(), frameMsgs.end(),
                [&](std::vector<geometry_msgs::msg::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

  // Reserve Base Msg
  std::vector<geometry_msgs::msg::Point> mpcBasePositionMsgs;
  mpcBasePositionMsgs.reserve(mpcStateTrajectory.size());

  // Reserve COM Msg
  std::vector<geometry_msgs::msg::Point> mpcCOMPositionMsgs;
  mpcCOMPositionMsgs.reserve(mpcStateTrajectory.size());

  // Extract Base and Feet from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    // Fill position msgs
    mpcBasePositionMsgs.push_back(getPointMsg(mpcRobotModelPtr_->getBasePose(state).head(3)));

    vector3_t comPosition =
        pinocchio::centerOfMass(stateTrajectoryPinocchioInterface.getModel(), stateTrajectoryPinocchioInterface.getData(),
                                mpcRobotModelPtr_->getGeneralizedCoordinates(state), false);
    comPosition[2] = groundHeight;

    mpcCOMPositionMsgs.push_back(getPointMsg(comPosition));

    const auto framePositions =
        computeFramePositions<scalar_t>(mpcRobotModelPtr_->getGeneralizedCoordinates(state), stateTrajectoryPinocchioInterface, frameNames);
    for (size_t i = 0; i < frameMsgs.size(); i++) {
      frameMsgs[i].push_back(getPointMsg(framePositions[i]));
    }
  });

  // Publish Terminal MPC State
  vector_t terminalState = mpcStateTrajectory.back();
  publishJointTransforms(mpcRobotModelPtr_->getJointAngles(terminalState), terminalJointPublisherPtr_);
  publishBaseTransform(mpcRobotModelPtr_->getBasePose(terminalState), "terminal_state/");

  // Publish Terminal Target State
  vector_t targetTerminalState = targetTrajectories.getDesiredState(mpcTimeTrajectory.back());
  publishJointTransforms(mpcRobotModelPtr_->getJointAngles(targetTerminalState), terminalJointTargetPublisherPtr_);
  publishBaseTransform(mpcRobotModelPtr_->getBasePose(targetTerminalState), "terminal_target/");

  // Convert feet msgs to Array message
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(frameMsgs.size() + 1);  // + 1 for the pelvis trajectory
  for (size_t i = 0; i < frameMsgs.size(); i++) {
    markerArray.markers.emplace_back(getLineMsg(std::move(frameMsgs[i]), contactColorMap_[i], trajectoryLineWidth_));
    markerArray.markers.back().ns = "Frame EE Trajectories";
  }

  markerArray.markers.emplace_back(getLineMsg(std::move(mpcBasePositionMsgs), Color::red, trajectoryLineWidth_));
  markerArray.markers.back().ns = "Base Trajectory";
  markerArray.markers.emplace_back(getLineMsg(std::move(mpcCOMPositionMsgs), Color::yellow, trajectoryLineWidth_));
  markerArray.markers.back().ns = "COM Trajectory";

  // Future footholds
  visualization_msgs::msg::Marker sphereList;
  sphereList.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  sphereList.scale.x = footMarkerDiameter_;
  sphereList.scale.y = footMarkerDiameter_;
  sphereList.scale.z = footMarkerDiameter_;
  sphereList.ns = "Future footholds";
  sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& subsystemSequence = modeSchedule.modeSequence;
  const auto tStart = mpcTimeTrajectory.front();
  const auto tEnd = mpcTimeTrajectory.back();
  for (size_t event = 0; event < eventTimes.size(); ++event) {
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
      const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]);
      const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
      const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);
      const auto feetPositions = computeContactPositions<scalar_t>(mpcRobotModelPtr_->getGeneralizedCoordinates(postEventState),
                                                                   stateTrajectoryPinocchioInterface, *mpcRobotModelPtr_);
      for (size_t i = 0; i < feetPositions.size(); i++) {
        if (!preEventContactFlags[i] && postEventContactFlags[i]) {  // If a foot lands, a marker is added at that location.
          sphereList.points.emplace_back(getPointMsg(feetPositions[i]));
          sphereList.colors.push_back(getColor(contactColorMap_[i]));
        }
      }
    }
  }
  markerArray.markers.push_back(std::move(sphereList));

  // Add headers and Id
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg("world", node_handle_->now()));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  stateOptimizedPublisherPtr_->publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizer::update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    updatePinocchioFrames(vector_t(observation.state));
    updateHandInteractiveMarkers();
    scalar_t groundHeight = getGroundHeightEstimate(pinocchioInterface_, *mpcRobotModelPtr_, observation.mode);
    publishObservation(observation);
    publishCartesianMarkers(modeNumber2StanceLeg(observation.mode), observation.state, observation.input);
    publishSelfCollisionMarkers(modeNumber2StanceLeg(observation.mode), observation.state);

    // Publish a new optimized trajectory if it is non-empty and not equal to the previous one
    if (!policy.timeTrajectory_.empty() && !policy.stateTrajectory_.empty() && !policy.inputTrajectory_.empty()) {
      if (policy.timeTrajectory_.front() != prevPolicyTime || !policy.stateTrajectory_.front().isApprox(prevPolicyState) ||
          !policy.inputTrajectory_.front().isApprox(prevPolicyInput)) {
        prevPolicyTime = policy.timeTrajectory_.front();
        prevPolicyState = policy.stateTrajectory_.front();
        prevPolicyInput = policy.inputTrajectory_.front();
        publishOptimizedStateTrajectory(policy.timeTrajectory_, policy.stateTrajectory_, policy.modeSchedule_,
                                        command.mpcTargetTrajectories_, groundHeight);
      }
    }

    lastTime_ = observation.time;
  }
}

}  // namespace ocs2::humanoid
