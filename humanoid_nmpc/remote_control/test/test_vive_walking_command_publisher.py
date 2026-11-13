from unittest.mock import Mock, patch

import pytest
import rclpy
from sensor_msgs.msg import JointState

from remote_control.vive_walking_command_publisher import ViveWalkingCommandPublisher


@pytest.fixture
def controller():
    rclpy.init()
    node = ViveWalkingCommandPublisher()
    node.publisher_ = Mock()
    with patch("remote_control.vive_walking_command_publisher.monotonic") as clock:
        clock.return_value = 10.0
        yield node, clock
    node.destroy_node()
    rclpy.shutdown()


def joint_state(x=0.8, y=0.7, grip=1.0):
    return JointState(
        name=["trackpad_x", "trackpad_y", "trackpad_pressed", "grip_button"],
        position=[x, y, 1.0, grip],
    )


def published(node):
    node.timer_callback()
    return node.publisher_.publish.call_args.args[0]


def test_disconnect_stops_velocity_and_holds_waist(controller):
    node, clock = controller
    node.right_callback(joint_state())
    first = published(node)
    assert first.linear_velocity_x == 0.7
    assert first.linear_velocity_y == -0.8
    assert first.desired_waist_yaw < 0.0
    clock.return_value = 10.0 + node.command_timeout_s
    stopped = published(node)
    assert stopped.linear_velocity_x == stopped.linear_velocity_y == 0.0
    assert stopped.desired_waist_yaw == first.desired_waist_yaw
    node.right_callback(joint_state(grip=0.0))
    assert published(node).linear_velocity_x == 0.7


def test_each_controller_expires_independently(controller):
    node, clock = controller
    node.left_callback(joint_state(grip=0.0))
    node.right_callback(joint_state(grip=0.0))
    clock.return_value = 10.4
    node.right_callback(joint_state(grip=0.0))
    clock.return_value = 10.6
    command = published(node)
    assert command.angular_velocity_z == 0.0
    assert command.linear_velocity_x == 0.7
    clock.return_value = 11.0
    node.left_callback(joint_state(grip=0.0))
    command = published(node)
    assert command.angular_velocity_z == -0.8
    assert command.linear_velocity_x == 0.0


@pytest.mark.parametrize("invalid", [JointState(), joint_state(x=float("nan"))])
def test_invalid_packet_clears_previous_command(controller, invalid):
    node, _ = controller
    node.left_callback(joint_state())
    node.right_callback(joint_state())
    node.left_callback(invalid)
    node.right_callback(invalid)
    command = published(node)
    assert (
        command.linear_velocity_x
        == command.linear_velocity_y
        == command.angular_velocity_z
        == 0.0
    )
    assert command.desired_waist_yaw == 0.0
