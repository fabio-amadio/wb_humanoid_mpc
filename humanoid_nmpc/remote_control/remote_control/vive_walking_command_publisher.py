"""****************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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
****************************************************************************"""

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from humanoid_mpc_msgs.msg import WalkingVelocityCommand


@dataclass
class TrackpadCommand:
    x: float = 0.0
    y: float = 0.0
    pressed: bool = False


def _apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value


def _joint_value(msg: JointState, joint_name: str) -> float | None:
    try:
        index = msg.name.index(joint_name)
    except ValueError:
        return None

    if index < len(msg.position):
        return msg.position[index]
    return None


class ViveWalkingCommandPublisher(Node):
    def __init__(self):
        super().__init__("vive_walking_command_publisher")

        self.publisher_rate = self.declare_parameter("publisher_rate", 25.0).value
        self.default_base_height = self.declare_parameter(
            "default_base_height", 0.75
        ).value
        self.pressed_threshold = self.declare_parameter(
            "pressed_threshold", 0.5
        ).value
        self.trackpad_deadband = self.declare_parameter(
            "trackpad_deadband", 0.1
        ).value

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=25)

        self.publisher_ = self.create_publisher(
            WalkingVelocityCommand, "/humanoid/walking_velocity_command", qos_profile
        )
        self.left_subscription_ = self.create_subscription(
            JointState,
            "/vive/left/joint_states",
            self.left_callback,
            qos_profile,
        )
        self.right_subscription_ = self.create_subscription(
            JointState,
            "/vive/right/joint_states",
            self.right_callback,
            qos_profile,
        )
        self.timer = self.create_timer(1.0 / self.publisher_rate, self.timer_callback)

        self.left_trackpad = TrackpadCommand()
        self.right_trackpad = TrackpadCommand()

    def left_callback(self, msg: JointState):
        trackpad_x = _joint_value(msg, "trackpad_x")
        trackpad_pressed = _joint_value(msg, "trackpad_pressed")

        if trackpad_x is None or trackpad_pressed is None:
            self.get_logger().warn(
                "Missing left Vive joints 'trackpad_x' or 'trackpad_pressed'.",
                throttle_duration_sec=5.0,
            )
            return

        self.left_trackpad.x = float(trackpad_x)
        self.left_trackpad.pressed = float(trackpad_pressed) >= self.pressed_threshold

    def right_callback(self, msg: JointState):
        trackpad_x = _joint_value(msg, "trackpad_x")
        trackpad_y = _joint_value(msg, "trackpad_y")
        trackpad_pressed = _joint_value(msg, "trackpad_pressed")

        if trackpad_x is None or trackpad_y is None or trackpad_pressed is None:
            self.get_logger().warn(
                "Missing right Vive joints 'trackpad_x', 'trackpad_y', or 'trackpad_pressed'.",
                throttle_duration_sec=5.0,
            )
            return

        self.right_trackpad.x = float(trackpad_x)
        self.right_trackpad.y = float(trackpad_y)
        self.right_trackpad.pressed = (
            float(trackpad_pressed) >= self.pressed_threshold
        )

    def timer_callback(self):
        msg = WalkingVelocityCommand()

        if self.right_trackpad.pressed:
            msg.linear_velocity_x = _apply_deadband(
                self.right_trackpad.y, self.trackpad_deadband
            )
            msg.linear_velocity_y = _apply_deadband(
                -self.right_trackpad.x, self.trackpad_deadband
            )
        else:
            msg.linear_velocity_x = 0.0
            msg.linear_velocity_y = 0.0

        if self.left_trackpad.pressed:
            msg.angular_velocity_z = _apply_deadband(
                -self.left_trackpad.x, self.trackpad_deadband
            )
        else:
            msg.angular_velocity_z = 0.0

        msg.desired_pelvis_height = self.default_base_height
        msg.desired_waist_yaw = 0.0
        msg.desired_waist_roll = 0.0
        msg.desired_waist_pitch = 0.0

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ViveWalkingCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
