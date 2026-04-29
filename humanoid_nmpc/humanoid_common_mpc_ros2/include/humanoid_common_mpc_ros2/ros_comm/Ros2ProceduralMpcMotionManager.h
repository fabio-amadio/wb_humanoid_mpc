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

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <limits>
#include <mutex>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"
#include "humanoid_mpc_msgs/msg/walking_velocity_command.hpp"

namespace ocs2::humanoid {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class Ros2ProceduralMpcMotionManager : public ProceduralMpcMotionManager {
 public:
  Ros2ProceduralMpcMotionManager(const std::string& gaitFile,
                                 const std::string& referenceFile,
                                 std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                 VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories,
                                 std::string robotName);

  ~Ros2ProceduralMpcMotionManager() override = default;

  /** Disable copy / move */
  Ros2ProceduralMpcMotionManager& operator=(const Ros2ProceduralMpcMotionManager&) = delete;
  Ros2ProceduralMpcMotionManager(const Ros2ProceduralMpcMotionManager&) = delete;
  Ros2ProceduralMpcMotionManager& operator=(Ros2ProceduralMpcMotionManager&&) = delete;
  Ros2ProceduralMpcMotionManager(Ros2ProceduralMpcMotionManager&&) = delete;

  void setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) override;

  void subscribe(rclcpp::Node::SharedPtr nodeHandle, const rclcpp::QoS& qos);

 private:
  struct HandPositionBounds {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t min = vector3_t::Constant(-std::numeric_limits<scalar_t>::infinity());
    vector3_t max = vector3_t::Constant(std::numeric_limits<scalar_t>::infinity());
  };

  struct WaistOrientationBounds {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t min = vector3_t(-1.57079632679, -0.10471975512, -0.10471975512);
    vector3_t max = vector3_t(1.57079632679, 0.10471975512, 0.10471975512);
  };

  WalkingVelocityCommand getScaledWalkingVelocityCommand(scalar_t time) override;

  void setHandPoseReference(const std::string& referenceName, const geometry_msgs::msg::PoseStamped& msg);
  vector3_t clampHandPosition(const std::string& referenceName, const vector3_t& positionInReferenceFrame) const;
  vector3_t clampWaistOrientation(const vector3_t& desiredWaistOrientation) const;
  vector3_t interpolateWaistOrientation(scalar_t time) const;

  rclcpp::Subscription<humanoid_mpc_msgs::msg::WalkingVelocityCommand>::SharedPtr velCommandSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr leftHandPoseSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rightHandPoseSubscriber_;
  std::mutex walkingVelCommandMutex_;
  scalar_t handReferenceTransitionDuration_ = 0.3;
  scalar_t waistReferenceTransitionDuration_ = 0.3;
  std::unordered_map<std::string, HandPositionBounds> handPositionBounds_;
  WaistOrientationBounds waistOrientationBounds_;
  scalar_t currentSolverTime_ = 0.0;
  scalar_t waistOrientationTransitionStartTime_ = 0.0;
  vector3_t waistOrientationStartReference_ = vector3_t::Zero();
  vector3_t waistOrientationGoalReference_ = vector3_t::Zero();
  bool waistOrientationReferenceInitialized_ = false;
  std::string robotName_;
};

}  // namespace ocs2::humanoid
