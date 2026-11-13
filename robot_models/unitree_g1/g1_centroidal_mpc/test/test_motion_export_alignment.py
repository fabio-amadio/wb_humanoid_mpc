"""Check generator timestamps against the configured initial state and plotting errors."""

import importlib.util
import hashlib
import json
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
        output = directory / f'motion_{fps}_"quoted".npz'
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
            "--num-motions",
            "2",
            "--max-attempts-per-motion",
            "3",
            "--save-additional-info",
            "--no-base-command-filter",
            "--manipulation-probability",
            "1",
        ]
        result = subprocess.run(
            command, cwd=workspace, capture_output=True, text=True, timeout=540
        )
        assert result.returncode == 0, result.stdout + result.stderr
        motions[fps] = output.with_name(output.stem + "_0000" + output.suffix)
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
        np.testing.assert_array_equal(
            motion["joint_pos"][0],
            np.array([initial[i] for i in range(12, 41)], dtype=np.float32),
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


@pytest.mark.parametrize("fps", [25, 50])
def test_one_manifest_per_generation_run(exports, fps):
    paths, task = exports
    motion = paths[fps]
    manifests = list(motion.parent.glob("generation_*.json"))
    # Two generator invocations, each producing two motions: two manifests, not four.
    assert len(manifests) == 2
    assert len(list(motion.parent.glob("*.npz"))) == 4
    assert not list(motion.parent.glob("*.npz.json"))
    metadata = next(
        data
        for data in [json.loads(path.read_text()) for path in manifests]
        if any(entry["file"] == motion.name for entry in data["motions"])
    )
    assert metadata["status"] == "completed"
    assert metadata["completed_motions"] == 2
    options = metadata["options"]
    assert options["numMotions"] == 2
    assert options["seed"] == 1
    assert options["fps"] == fps
    assert options["handPoseMode"] is True
    assert options["saveAdditionalInfo"] is True
    assert options["bypassBaseCommandFilter"] is True
    assert metadata["argv"][metadata["argv"].index("--output") + 1] == options["output"]
    assert len(metadata["motions"]) == 2
    for index, record in enumerate(metadata["motions"]):
        assert record["index"] == index
        assert (motion.parent / record["file"]).is_file()
        assert 1 <= record["attempts"] <= options["maxAttemptsPerMotion"]
        assert record["seed"] == (
            options["seed"]
            + index * options["maxAttemptsPerMotion"]
            + record["attempts"]
            - 1
        ) % (2 ** 32)
        assert set(record) == {"index", "file", "seed", "attempts"}
    assert metadata["inputs"]["task"]["contents"] == task.read_text()
    for entry in metadata["inputs"].values():
        assert entry["contents"].encode() == Path(entry["path"]).read_bytes()
        assert entry["sha256"] == hashlib.sha256(entry["contents"].encode()).hexdigest()
    assert metadata["conventions"]["quaternion_order"] == "wxyz"
    libraries = metadata["solver"]["libraries"]
    assert any("libblasfeo.so" in entry["path"] for entry in libraries)
    assert any("libhpipm.so" in entry["path"] for entry in libraries)
    assert all(len(entry["sha256"]) == 64 for entry in libraries)
    assert len(metadata["source_tree"]["generator_sha256"]) == 64
    assert not list(motion.parent.glob("*.tmp"))
