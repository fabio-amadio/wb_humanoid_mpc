#!/usr/bin/env python3
"""Compare generic and AVX2 QP libraries on exactly the same augmented-state walking MPC."""

import argparse
import hashlib
import json
import os
import re
import subprocess
from pathlib import Path
from time import perf_counter

import numpy as np
from ament_index_python.packages import get_package_prefix

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=ROOT / "generated_motions/fast_qp_comparison"
    )
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--velocity", type=float, default=0.3)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument(
        "--task-file",
        type=Path,
        default=ROOT
        / "robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_hand_pose.info",
    )
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if args.duration <= 0 or args.fps <= 0 or args.repeats < 1 or args.velocity <= 0:
        parser.error("Duration, fps, repeats and velocity must be positive")
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error("Output directory is not empty; choose a new directory")
    prefix = (
        Path(
            os.environ.get("WB_MPC_FAST_QP_ROOT", ROOT / ".docker_ws/optimized_qp/avx2")
        )
        / "install"
    )
    if not (prefix / "fast_qp_backend.txt").exists():
        parser.error(
            "Build the optimized backend first: bash scripts/build_fast_qp.bash"
        )
    output.mkdir(parents=True, exist_ok=True)
    (output / ".gitignore").write_text("*\n")
    task = output / "task.info"
    task.write_bytes(args.task_file.read_bytes())
    if not re.search(r"joint_acceleration_cost\s*\{\s*active\s+true", task.read_text()):
        parser.error(
            "This comparison requires the extended state and acceleration cost to remain active"
        )
    reference = output / "reference.info"
    reference.write_bytes(
        (
            ROOT
            / "robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference_random_hand_pose.info"
        ).read_bytes()
    )
    executable = (
        Path(get_package_prefix("humanoid_centroidal_mpc_ros2"))
        / "lib/humanoid_centroidal_mpc_ros2/humanoid_centroidal_mpc_random_hand_pose_generator"
    )
    generic_env = os.environ.copy()
    generic_env["LD_LIBRARY_PATH"] = ":".join(
        part
        for part in generic_env.get("LD_LIBRARY_PATH", "").split(":")
        if part != str(prefix / "lib")
    )
    fast_env = generic_env.copy()
    fast_env["LD_LIBRARY_PATH"] = (
        str(prefix / "lib") + ":" + generic_env["LD_LIBRARY_PATH"]
    )
    report = {
        "scenario": "walking with 17 extra input states and unchanged acceleration weights, horizon, time step and integrator",
        "task_sha256": hashlib.sha256(task.read_bytes()).hexdigest(),
        "arguments": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        "runs": [],
    }
    for label, env in [("generic", generic_env), ("avx2", fast_env)]:
        linkage = subprocess.check_output(["ldd", str(executable)], env=env, text=True)
        (output / f"{label}_linkage.txt").write_text(linkage)
        if label == "avx2" and not all(
            f"{prefix}/lib/lib{library}.so" in linkage
            for library in ["blasfeo", "hpipm"]
        ):
            raise RuntimeError(
                "Optimized libraries are not being loaded; inspect avx2_linkage.txt"
            )

    baseline = None
    for repeat in range(args.repeats):
        backends = [("generic", generic_env), ("avx2", fast_env)]
        if repeat % 2:
            backends.reverse()
        for label, env in backends:
            motion = output / f"{label}_{repeat}.npz"
            command = [
                str(executable),
                "--task-file",
                str(task),
                "--reference-file",
                str(reference),
                "--output",
                str(motion),
                "--duration",
                str(args.duration),
                "--fps",
                str(args.fps),
                "--seed",
                str(args.seed),
                "--max-attempts-per-motion",
                "1",
                "--save-additional-info",
                "--no-base-command-filter",
                "--basic-primitive",
                "walk_forward",
                "--vx-max",
                str(args.velocity),
                "--basic-primitive-min-speed-ratio",
                "1",
            ]
            print(f"Running {label}, repeat {repeat + 1}/{args.repeats}", flush=True)
            log = output / f"{label}_{repeat}.log"
            start = perf_counter()
            with log.open("w") as stream:
                result = subprocess.run(
                    command,
                    cwd=ROOT.parent.parent,
                    env=env,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    timeout=max(120, args.duration * 30),
                )
            elapsed = perf_counter() - start
            if result.returncode:
                raise RuntimeError(f"{label} failed; inspect {log}")
            motion = motion.with_name(motion.stem + "_walk_forward.npz")
            timings = dict(
                re.findall(
                    r"(LQ Approximation|Solve QP|Linesearch|Compute Controller)\s*:\s*([\d.eE+-]+)\s*\[ms\]",
                    log.read_text(),
                )
            )
            if len(timings) != 4:
                raise RuntimeError(f"Missing SQP timings in {log}")
            timings = {key: float(value) for key, value in timings.items()}
            with np.load(motion, allow_pickle=False) as data:
                arrays = {
                    key: data[key]
                    for key in [
                        "joint_pos",
                        "joint_vel",
                        "base_command",
                        "left_hand_target_pos",
                        "right_hand_target_pos",
                    ]
                }
            if not all(np.isfinite(value).all() for value in arrays.values()):
                raise RuntimeError(f"Non-finite motion data in {motion}")
            if baseline is None:
                baseline = arrays
            targets_equal = all(
                np.array_equal(arrays[key], baseline[key])
                for key in [
                    "base_command",
                    "left_hand_target_pos",
                    "right_hand_target_pos",
                ]
            )
            if not targets_equal:
                raise RuntimeError("Walking command targets differ")
            run = {
                "backend": label,
                "repeat": repeat,
                "wall_seconds": elapsed,
                "timings_ms": timings,
                "sqp_ms_per_iteration": sum(timings.values()),
                "command_targets_identical": targets_equal,
                "max_joint_position_difference_rad": float(
                    np.max(np.abs(arrays["joint_pos"] - baseline["joint_pos"]))
                ),
                "max_joint_velocity_difference_rad_s": float(
                    np.max(np.abs(arrays["joint_vel"] - baseline["joint_vel"]))
                ),
            }
            report["runs"].append(run)
            print(json.dumps(run), flush=True)
            (output / "benchmark.json").write_text(json.dumps(report, indent=2) + "\n")
    report["median_sqp_ms"] = {
        label: float(
            np.median(
                [
                    run["sqp_ms_per_iteration"]
                    for run in report["runs"]
                    if run["backend"] == label
                ]
            )
        )
        for label in ["generic", "avx2"]
    }
    (output / "benchmark.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["median_sqp_ms"], indent=2))


if __name__ == "__main__":
    main()
