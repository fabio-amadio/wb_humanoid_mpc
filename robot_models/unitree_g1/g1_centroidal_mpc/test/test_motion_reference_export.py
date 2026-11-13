"""Exercise the installed exporter against synthetic ROS policies, without a solver."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
from time import monotonic

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

from humanoid_mpc_msgs.msg import (
    MpcFutureMotionJointPos,
    MpcFutureMotionJointState,
    MpcMotionJointPos,
    MpcMotionJointState,
)
from ocs2_ros2_msgs.msg import (
    MpcFlattenedController,
    MpcInput,
    MpcObservation,
    MpcState,
)


@pytest.mark.parametrize("joint_state", [False, True])
@pytest.mark.parametrize("future", [False, True])
def test_export_layout_expiry_and_recovery(joint_state, future):
    share = Path(get_package_share_directory("g1_centroidal_mpc"))
    description = Path(get_package_share_directory("g1_description"))
    executable = (
        Path(get_package_prefix("humanoid_centroidal_mpc_ros2"))
        / "lib/humanoid_centroidal_mpc_ros2/humanoid_centroidal_mpc_motion_reference_node"
    )
    prefix = f"review_{os.getpid()}_{int(joint_state)}_{int(future)}"
    args = [
        str(executable),
        prefix,
        str(share / "config/mpc/task_locomotion.info"),
        str(share / "config/command/reference.info"),
        str(description / "urdf/g1_29dof.urdf"),
        "--motion-reference-type",
        "joint_state" if joint_state else "joint_pos",
    ]
    if future:
        args.append("--publish-future-motion-ref")
    rclpy.init()
    node = rclpy.create_node(prefix + "_test")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    policy_pub = node.create_publisher(
        MpcFlattenedController, prefix + "/mpc_policy", qos
    )
    observation_pub = node.create_publisher(
        MpcObservation, prefix + "/mpc_observation", qos
    )
    message_type = (
        (MpcFutureMotionJointState if joint_state else MpcFutureMotionJointPos)
        if future
        else (MpcMotionJointState if joint_state else MpcMotionJointPos)
    )
    received = []
    node.create_subscription(
        message_type,
        prefix
        + ("/mpc_future_motion_reference" if future else "/mpc_motion_reference"),
        received.append,
        qos,
    )

    def spin_for(duration):
        deadline = monotonic() + duration
        while monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    def publish_policy(start):
        states = []
        for x in (0.0, 1.0):
            values = [0.0] * 12 + [0.25] * 23
            values[6], values[8] = x, 0.76
            states.append(MpcState(value=values))
        inputs = [MpcInput(value=[0.0] * 35) for _ in states]
        policy = MpcFlattenedController(
            time_trajectory=[start, start + 1.0],
            state_trajectory=states,
            input_trajectory=inputs,
        )
        policy.init_observation = MpcObservation(
            time=start, state=states[0], input=inputs[0], mode=bytes([3])
        )
        policy.mode_schedule.mode_sequence = [3]
        policy.plan_target_trajectories.time_trajectory = list(policy.time_trajectory)
        policy.plan_target_trajectories.state_trajectory = states
        policy.plan_target_trajectories.input_trajectory = inputs
        for _ in range(3):
            policy_pub.publish(policy)
            spin_for(0.05)

    def request_reference(time):
        received.clear()
        for _ in range(10):
            observation_pub.publish(MpcObservation(time=time))
            spin_for(0.05)
            if received:
                break

    with tempfile.TemporaryFile() as log:
        process = subprocess.Popen(args, stdout=log, stderr=log)
        try:
            deadline = monotonic() + 10.0
            while (
                policy_pub.get_subscription_count() == 0
                or observation_pub.get_subscription_count() == 0
            ):
                assert process.poll() is None, "Exporter exited during startup"
                assert monotonic() < deadline, "Exporter did not connect"
                spin_for(0.05)
            publish_policy(1.0)
            request_reference(1.25)
            assert received, "Valid policy did not produce a reference"
            message = received[-1]
            block_size = 64 if joint_state else 35
            assert len(message.motion_cmd) == block_size * (13 if future else 1)
            assert message.motion_cmd[0] == pytest.approx(0.25)
            assert message.motion_cmd[block_size - 6] == pytest.approx(1.0)
            assert message.motion_cmd[block_size - 3] == pytest.approx(0.76)
            if joint_state:
                assert list(message.motion_cmd[29:58]) == [0.0] * 29
            if future:
                assert list(message.steps) == list(range(0, 49, 4))
                assert message.dt == pytest.approx(0.02)
            spin_for(0.1)
            request_reference(3.25)
            assert not received, "Expired policy was republished"
            request_reference(0.0)
            assert not received, "Policy from before a time reset was republished"
            publish_policy(3.0)
            request_reference(3.25)
            assert received, "Export did not recover with a fresh policy"
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()
            if process.returncode != 0:
                log.seek(0)
                pytest.fail(log.read().decode(errors="replace"))
