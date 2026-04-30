"""Record MPC motion references to a CLAMP-compatible NPZ file."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from humanoid_mpc_msgs.msg import MpcMotionReference
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


class MpcMotionReferenceRecorder(Node):
    def __init__(self, topic: str, output: Path, max_duration_s: float | None):
        super().__init__("mpc_motion_reference_recorder")
        self.output = output
        self.max_duration_s = max_duration_s
        self.start_time_s: float | None = None
        self.timestamps_s: list[float] = []
        self.joint_names: tuple[str, ...] | None = None
        self.joint_pos: list[np.ndarray] = []
        self.joint_vel: list[np.ndarray] = []
        self.root_pos_w: list[np.ndarray] = []
        self.root_quat_w: list[np.ndarray] = []
        self.root_lin_vel_w: list[np.ndarray] = []
        self.root_ang_vel_w: list[np.ndarray] = []
        self.motion_cmd: list[np.ndarray] = []

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=100)
        self.subscription = self.create_subscription(
            MpcMotionReference, topic, self.reference_callback, qos_profile
        )

        self.get_logger().info(
            f"Recording MPC motion references from `{topic}` to `{output}`"
        )

    def reference_callback(self, msg: MpcMotionReference) -> None:
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

        joint_names = tuple(msg.joint_names)
        if self.joint_names is None:
            self.joint_names = joint_names
        elif joint_names != self.joint_names:
            raise RuntimeError(
                "Received MPC motion reference with changed joint_names."
            )

        joint_pos = np.asarray(msg.joint_pos, dtype=np.float64)
        joint_vel = np.asarray(msg.joint_vel, dtype=np.float64)
        if joint_pos.shape != joint_vel.shape:
            raise RuntimeError(
                f"joint_pos shape {joint_pos.shape} does not match joint_vel shape {joint_vel.shape}."
            )

        self.timestamps_s.append(elapsed_s)
        self.joint_pos.append(joint_pos)
        self.joint_vel.append(joint_vel)
        self.root_pos_w.append(
            np.asarray(
                [
                    msg.root_pose_w.position.x,
                    msg.root_pose_w.position.y,
                    msg.root_pose_w.position.z,
                ],
                dtype=np.float64,
            )
        )
        self.root_quat_w.append(
            np.asarray(
                [
                    msg.root_pose_w.orientation.w,
                    msg.root_pose_w.orientation.x,
                    msg.root_pose_w.orientation.y,
                    msg.root_pose_w.orientation.z,
                ],
                dtype=np.float64,
            )
        )
        self.root_lin_vel_w.append(
            np.asarray(
                [
                    msg.root_twist_w.linear.x,
                    msg.root_twist_w.linear.y,
                    msg.root_twist_w.linear.z,
                ],
                dtype=np.float64,
            )
        )
        self.root_ang_vel_w.append(
            np.asarray(
                [
                    msg.root_twist_w.angular.x,
                    msg.root_twist_w.angular.y,
                    msg.root_twist_w.angular.z,
                ],
                dtype=np.float64,
            )
        )
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

        body_pos_w = np.asarray(self.root_pos_w, dtype=np.float64)[:, None, :]
        body_quat_w = np.asarray(self.root_quat_w, dtype=np.float64)[:, None, :]
        body_lin_vel_w = np.asarray(self.root_lin_vel_w, dtype=np.float64)[:, None, :]
        body_ang_vel_w = np.asarray(self.root_ang_vel_w, dtype=np.float64)[:, None, :]

        np.savez(
            self.output,
            joint_pos=np.asarray(self.joint_pos, dtype=np.float64),
            joint_vel=np.asarray(self.joint_vel, dtype=np.float64),
            body_pos_w=body_pos_w,
            body_quat_w=body_quat_w,
            body_lin_vel_w=body_lin_vel_w,
            body_ang_vel_w=body_ang_vel_w,
            body_names=np.asarray(["pelvis"]),
            body_link_names=np.asarray(["pelvis"]),
            joint_names=np.asarray(self.joint_names),
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
    return parser


def main(args=None) -> None:
    parser = _build_argparser()
    cli_args = parser.parse_args(remove_ros_args(args=args)[1:])
    output = cli_args.output.expanduser().resolve()

    rclpy.init(args=args)
    recorder = MpcMotionReferenceRecorder(cli_args.topic, output, cli_args.duration)
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
