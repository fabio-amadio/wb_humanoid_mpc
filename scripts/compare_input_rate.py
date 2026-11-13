#!/usr/bin/env python3
"""Generate matched hand-pose motions at several input-rate weights, then plot the results. Run inside Docker."""

import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
from pathlib import Path
from time import perf_counter

from ament_index_python.packages import get_package_prefix

from plot_mpc_motion_comparison import compare

ROOT = Path(__file__).resolve().parents[1]


def rate_task(source, scale):
    match = re.search(r"(?m)^joint_acceleration_cost\s*\{", source)
    if not match:
        raise ValueError("Task has no joint_acceleration_cost block")
    start = source.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    block = source[match.start() : end]
    scaling = re.search(r"(?m)^\s*scaling\s+([\deE.+-]+)", block)
    if not scaling:
        raise ValueError("Rate weight block has no scaling field")
    old_scaling = float(scaling[1])
    block = re.sub(
        r"(?m)^(\s*active\s+)(true|false)",
        lambda m: m[1] + ("true" if scale else "false"),
        block,
        count=1,
    )
    block = re.sub(
        r"(?m)^(\s*scaling\s+)[\deE.+-]+",
        lambda m: m[1] + str(old_scaling * scale),
        block,
        count=1,
    )
    return source[: match.start()] + block + source[end:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "generated_motions/input_rate_comparison",
    )
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--scales", type=float, nargs="+", default=[0.0, 1.0, 10.0])
    args = parser.parse_args()
    if (
        not math.isfinite(args.duration)
        or not math.isfinite(args.fps)
        or args.duration <= 0
        or args.fps <= 0
        or len(set(args.scales)) != len(args.scales)
        or any(not math.isfinite(scale) or scale < 0 for scale in args.scales)
    ):
        parser.error(
            "Duration/fps must be positive; scales must be unique and nonnegative"
        )
    output = args.output_dir.expanduser().resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error(
            "Output directory is not empty; choose a new directory to preserve previous results"
        )
    output.mkdir(parents=True, exist_ok=True)
    (output / ".gitignore").write_text("# Generated comparison artifacts\n*\n")
    config = output / "configs"
    config.mkdir()
    task = (
        ROOT
        / "robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_hand_pose.info"
    )
    reference = (
        ROOT
        / "robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference_random_hand_pose.info"
    )
    urdf = ROOT / "robot_models/unitree_g1/g1_description/urdf/g1_29dof.urdf"
    reference_copy = config / reference.name
    reference_copy.write_bytes(reference.read_bytes())
    executable = (
        Path(get_package_prefix("humanoid_centroidal_mpc_ros2"))
        / "lib/humanoid_centroidal_mpc_ros2/humanoid_centroidal_mpc_random_hand_pose_generator"
    )
    manifest = {
        "seed": args.seed,
        "duration": args.duration,
        "fps": args.fps,
        "scales": args.scales,
        "scenario": "standing manipulation, deterministic sampled commands, wall-clock base filter bypassed",
        "git_commit": subprocess.check_output(
            ["git", "-c", f"safe.directory={ROOT}", "rev-parse", "HEAD"],
            cwd=ROOT,
            text=True,
        ).strip(),
        "sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [
                task,
                reference,
                urdf,
                ROOT
                / "humanoid_nmpc/humanoid_centroidal_mpc/src/solver/InputRateSqpMpc.cpp",
                ROOT
                / "humanoid_nmpc/humanoid_centroidal_mpc_ros2/src/CentroidalMpcRandomReferenceGenerator.cpp",
            ]
        },
        "runs": [],
    }
    paths, labels = [], []
    for scale in args.scales:
        label = "rate_off" if scale == 0 else f"rate_{scale:g}x"
        task_copy = config / f"task_hand_pose_{label}.info"
        task_copy.write_text(rate_task(task.read_text(), scale))
        motion = output / f"{label}.npz"
        command = [
            str(executable),
            "--task-file",
            str(task_copy),
            "--reference-file",
            str(reference_copy),
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
            "--manipulation-probability",
            "1",
            "--heading-prob",
            "0",
            "--no-base-command-filter",
        ]
        print(
            f"Generating {label}: {args.duration:g}s at {args.fps:g} Hz, seed {args.seed}",
            flush=True,
        )
        start = perf_counter()
        log_path = output / f"{label}.log"
        with log_path.open("w") as log:
            result = subprocess.run(
                command,
                cwd=ROOT.parent.parent,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=max(120, args.duration * 30),
            )
        elapsed = perf_counter() - start
        if result.returncode:
            raise RuntimeError(
                f"{label} failed (exit {result.returncode}); inspect {log_path}"
            )
        text = log_path.read_text()
        timings = dict(
            re.findall(
                r"(LQ Approximation|Solve QP|Linesearch|Compute Controller)\s*:\s*([\d.eE+-]+)\s*\[ms\]",
                text,
            )
        )
        timings = {key: float(value) for key, value in timings.items()}
        if len(timings) != 4:
            raise RuntimeError(f"Incomplete SQP timings; inspect {log_path}")
        run = {
            "label": label,
            "scale": scale,
            "wall_seconds": elapsed,
            "sqp_ms_per_iteration": sum(timings.values()),
            "timings_ms": timings,
            "command": command,
        }
        manifest["runs"].append(run)
        (output / "benchmark.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(
            f"  wall {elapsed:.2f}s; SQP {run['sqp_ms_per_iteration']:.2f} ms/iteration",
            flush=True,
        )
        paths.append(motion)
        labels.append(label)
    report = compare(paths, labels, output, urdf=urdf, require_matched=True)
    for run, metrics in zip(manifest["runs"], report["motions"]):
        run["metrics"] = metrics
    (output / "benchmark.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Plots, NPZ files, task copies and benchmark metadata: {output}")


if __name__ == "__main__":
    main()
