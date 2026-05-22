"""Record MPC motion references to a CLAMP-compatible NPZ file."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from humanoid_mpc_msgs.msg import MpcMotionJointPos, MpcMotionJointState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


class MpcMotionReferenceRecorder(Node):
    def __init__(
        self,
        topic: str,
        output: Path,
        max_duration_s: float | None,
        message_type: str,
    ):
        super().__init__("mpc_motion_reference_recorder")
        self.output = output
        self.max_duration_s = max_duration_s
        self.start_time_s: float | None = None
        self.timestamps_s: list[float] = []
        self.motion_cmd: list[np.ndarray] = []

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=100)
        msg_cls = MpcMotionJointState if message_type == "joint_state" else MpcMotionJointPos
        self.subscription = self.create_subscription(
            msg_cls, topic, self.reference_callback, qos_profile
        )

        self.get_logger().info(
            f"Recording {message_type} MPC motion references from `{topic}` to `{output}`"
        )

    def reference_callback(self, msg) -> None:
        timestamp_s = float(msg.header.stamp.sec) + 1.0e-9 * float(
            msg.header.stamp.nanosec
        )
        if self.start_time_s is None:
            self.start_time_s = timestamp_s
        elapsed_s = timestamp_s - self.start_time_s

        if self.max_duration_s is not None and elapsed_s > self.max_duration_s:
            self.get_logger().info("Requested duration reached; saving NPZ.")
            self.save()
            rclpy.shutdown()
            return

        self.timestamps_s.append(elapsed_s)
        self.motion_cmd.append(np.asarray(msg.motion_cmd, dtype=np.float32))

        if len(self.timestamps_s) % 100 == 0:
            self.get_logger().info(
                f"Recorded {len(self.timestamps_s)} MPC reference frames."
            )

    def save(self) -> None:
        if not self.timestamps_s:
            self.get_logger().warn(
                "No MPC motion reference frames were recorded; skipping NPZ save."
            )
            return

        self.output.parent.mkdir(parents=True, exist_ok=True)
        timestamps = np.asarray(self.timestamps_s, dtype=np.float64)
        fps = (
            1.0 / float(np.median(np.diff(timestamps))) if timestamps.size > 1 else 0.0
        )

        np.savez(
            self.output,
            fps=np.asarray([fps], dtype=np.float64),
            timestamps_s=timestamps,
            motion_cmd=np.asarray(self.motion_cmd, dtype=np.float32),
        )
        self.get_logger().info(
            f"Saved {len(self.timestamps_s)} frames to `{self.output}` at estimated fps={fps:.3f}."
        )


def _build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record `/g1/mpc_motion_reference` to a CLAMP-compatible NPZ."
    )
    parser.add_argument(
        "--topic",
        default="/g1/mpc_motion_reference",
        help="MPC motion reference topic.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/g1_mpc_motion_reference.npz"),
        help="Output NPZ path.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional recording duration in seconds. Without this, press Ctrl-C to save.",
    )
    parser.add_argument(
        "--message-type",
        choices=("joint_pos", "joint_state"),
        default="joint_pos",
        help="Compact MPC motion reference message type.",
    )
    return parser


def main(args=None) -> None:
    parser = _build_argparser()
    cli_args = parser.parse_args(remove_ros_args(args=args)[1:])
    output = cli_args.output.expanduser().resolve()

    rclpy.init(args=args)
    recorder = MpcMotionReferenceRecorder(
        cli_args.topic, output, cli_args.duration, cli_args.message_type
    )
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            recorder.save()
            recorder.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
