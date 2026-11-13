"""Check generator timestamps against the configured initial state and plotting errors."""

import importlib.util
from pathlib import Path
import re
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import numpy as np
import pytest


@pytest.fixture(scope="module")
def exports(tmp_path_factory):
    directory = tmp_path_factory.mktemp("aligned_motions")
    share = Path(get_package_share_directory("g1_centroidal_mpc"))
    prefix = Path(get_package_prefix("humanoid_centroidal_mpc_ros2"))
    workspace = prefix.parent if prefix.name == "install" else prefix.parent.parent
    task = share / "config/mpc/task_hand_pose.info"
    reference = share / "config/command/reference_random_hand_pose.info"
    executable = (
        prefix
        / "lib/humanoid_centroidal_mpc_ros2/humanoid_centroidal_mpc_random_hand_pose_generator"
    )
    motions = {}
    for fps in (25, 50):
        output = directory / f"motion_{fps}.npz"
        command = [
            str(executable),
            "--task-file",
            str(task),
            "--reference-file",
            str(reference),
            "--output",
            str(output),
            "--duration",
            "0.13",
            "--fps",
            str(fps),
            "--seed",
            "1",
            "--max-attempts-per-motion",
            "1",
            "--save-additional-info",
            "--no-base-command-filter",
            "--manipulation-probability",
            "1",
        ]
        result = subprocess.run(
            command, cwd=workspace, capture_output=True, text=True, timeout=540
        )
        assert result.returncode == 0, result.stdout + result.stderr
        motions[fps] = output
    return motions, task


@pytest.mark.parametrize("fps", [25, 50])
def test_commands_and_robot_start_at_zero_and_share_the_time_grid(exports, fps):
    paths, task = exports
    initial_block = re.search(r"initialState\s*\{([^}]+)\}", task.read_text()).group(1)
    initial = {
        int(index): float(value)
        for index, value in re.findall(r"\((\d+),0\)\s+([-+\deE.]+)", initial_block)
    }
    with np.load(paths[fps], allow_pickle=False) as motion:
        frames = int(np.floor(0.13 * fps)) + 1
        np.testing.assert_allclose(
            motion["time"], np.arange(frames) / fps, atol=1e-14, rtol=0
        )
        assert motion["time"][0] == 0
        assert motion["time"][-1] <= 0.13
        # This fails if row zero is a post-rollout state relabeled as t=0.
        np.testing.assert_allclose(
            motion["joint_pos"][0],
            [initial[i] for i in range(12, 41)],
            atol=1e-8,
            rtol=0,
        )
        pelvis = list(motion["body_names"].astype(str)).index("pelvis")
        np.testing.assert_allclose(
            motion["body_pos_w"][0, pelvis],
            [initial[i] for i in range(6, 9)],
            atol=1e-7,
            rtol=0,
        )
        for key in (
            "joint_pos",
            "joint_vel",
            "body_pos_w",
            "body_quat_w",
            "body_lin_vel_w",
            "body_ang_vel_w",
            "contact_wrench_w",
            "contact_flags",
            "base_command",
            "left_hand_target_pos",
            "right_hand_target_pos",
            "left_hand_target_quat",
            "right_hand_target_quat",
            "random_mode",
        ):
            assert motion[key].shape[0] == frames, key
            assert np.isfinite(motion[key]).all(), key
    with np.load(paths[25], allow_pickle=False) as coarse, np.load(
        paths[50], allow_pickle=False
    ) as fine:
        for key in (
            "base_command",
            "left_hand_target_pos",
            "right_hand_target_pos",
            "left_hand_target_quat",
            "right_hand_target_quat",
            "random_mode",
        ):
            np.testing.assert_array_equal(coarse[key], fine[key][::2])


def test_tracking_uses_equal_time_rows_and_requires_timestamps(exports, tmp_path):
    paths, _ = exports
    script = (
        Path(__file__).resolve().parents[4] / "scripts/plot_mpc_motion_comparison.py"
    )
    spec = importlib.util.spec_from_file_location("plot_mpc_motion_comparison", script)
    plotting = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(plotting)
    motion = plotting.load_motion(paths[50])
    urdf = Path(get_package_share_directory("g1_description")) / "urdf/g1_29dof.urdf"
    errors = plotting.hand_tracking(motion, urdf)
    for side in ("left", "right"):
        assert len(errors[side]) == len(motion["time"])
        # The initial hand command is the default pose; shifting targets by a row is incorrect.
        assert errors[side][0] < 1e-6
    without_time = tmp_path / "missing_time.npz"
    with np.load(paths[50], allow_pickle=False) as data:
        np.savez(
            without_time, **{key: data[key] for key in data.files if key != "time"}
        )
    with pytest.raises(ValueError, match="Missing required 'time' array"):
        plotting.load_motion(without_time)
