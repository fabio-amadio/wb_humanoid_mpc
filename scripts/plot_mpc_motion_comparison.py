#!/usr/bin/env python3
"""Compare generated MPC NPZ motions: q, dq, finite-difference acceleration and hand tracking."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


COMMAND_KEYS = (
    "base_command",
    "left_hand_target_pos",
    "right_hand_target_pos",
    "left_hand_target_quat",
    "right_hand_target_quat",
    "random_mode",
)


def strings(values):
    return [
        value.decode() if isinstance(value, bytes) else str(value) for value in values
    ]


def load_motion(path):
    with np.load(path, allow_pickle=False) as data:
        motion = {key: data[key] for key in data.files}
    motion["names"] = strings(motion["joint_names"])
    motion["hz"] = float(motion["fps"].reshape(-1)[0])
    if not np.isfinite(motion["hz"]) or motion["hz"] <= 0:
        raise ValueError(f"Invalid fps in {path}")
    for key in ("joint_pos", "joint_vel"):
        motion[key] = np.asarray(motion[key], dtype=float)
        if not np.isfinite(motion[key]).all():
            raise ValueError(f"Non-finite {key} in {path}")
    if (
        motion["joint_pos"].shape != motion["joint_vel"].shape
        or len(motion["joint_pos"]) < 3
    ):
        raise ValueError(f"Joint position/velocity shapes are invalid in {path}")
    motion["time"] = np.arange(len(motion["joint_pos"])) / motion["hz"]
    motion["acceleration"] = np.diff(motion["joint_vel"], axis=0) * motion["hz"]
    return motion


def hand_tracking(motion, urdf):
    """The generator stores state k+1 with target k; compare state k+1 to target k+1."""
    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(urdf), pin.JointModelFreeFlyer())
    data = model.createData()
    torso_id = model.getFrameId("torso_link")
    hand_ids = [model.getFrameId(side + "_rubber_hand") for side in ("left", "right")]
    if any(frame_id >= model.nframes for frame_id in [torso_id] + hand_ids):
        raise ValueError("URDF is missing torso/hand frames")
    joint_indices = [model.idx_qs[model.getJointId(name)] for name in motion["names"]]
    pelvis = strings(motion["body_names"]).index("pelvis")
    actual = np.empty((len(motion["time"]), 2, 3))
    for k, joints in enumerate(motion["joint_pos"]):
        q = pin.neutral(model)
        q[:3] = motion["body_pos_w"][k, pelvis]
        wxyz = motion["body_quat_w"][k, pelvis]
        q[3:7] = wxyz[[1, 2, 3, 0]]
        q[joint_indices] = joints
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        for side, frame in enumerate(hand_ids):
            actual[k, side] = (
                data.oMf[torso_id].inverse() * data.oMf[frame]
            ).translation
    return {
        side: np.linalg.norm(
            actual[:-1, i] - motion[side + "_hand_target_pos"][1:], axis=1
        )
        for i, side in enumerate(("left", "right"))
    }


def compare(paths, labels, output, joints=None, urdf=None, require_matched=False):
    output.mkdir(parents=True, exist_ok=True)
    motions = [load_motion(path) for path in paths]
    names = motions[0]["names"]
    if any(motion["names"] != names for motion in motions):
        raise ValueError("Joint names/order differ across files")
    command_match = (
        all(
            key in motion
            and key in motions[0]
            and np.array_equal(motion[key], motions[0][key])
            for motion in motions[1:]
            for key in COMMAND_KEYS
        )
        if len(motions) > 1
        else None
    )
    if require_matched and not command_match:
        raise ValueError("Command targets differ: this is not a matched comparison")
    groups = {
        "waist": [i for i, name in enumerate(names) if "waist" in name],
        "arm": [
            i for i, name in enumerate(names) if "shoulder" in name or "elbow" in name
        ],
        "wrist": [i for i, name in enumerate(names) if "wrist" in name],
    }
    selected = sorted(set(sum(groups.values(), [])))
    baseline_rms = np.sqrt(np.mean(motions[0]["acceleration"] ** 2, axis=0))
    shown = (
        [names.index(name) for name in joints]
        if joints
        else [
            max(indices, key=lambda i: baseline_rms[i])
            for indices in groups.values()
            if indices
        ]
    )
    if not shown:
        raise ValueError("No matching joints to plot")
    fig, axes = plt.subplots(
        3, len(shown), figsize=(5 * len(shown), 9), sharex="col", squeeze=False
    )
    rows = [
        ("joint_pos", "Position [rad]"),
        ("joint_vel", "Velocity [rad/s]"),
        ("acceleration", "Acceleration [rad/s²]"),
    ]
    for column, index in enumerate(shown):
        axes[0, column].set_title(names[index].replace("_joint", ""))
        for row, (key, ylabel) in enumerate(rows):
            for motion, label in zip(motions, labels):
                time = motion["time"][1:] if key == "acceleration" else motion["time"]
                axes[row, column].plot(
                    time, motion[key][:, index], label=label, linewidth=1.2
                )
            axes[row, column].set_ylabel(ylabel)
            axes[row, column].grid(alpha=0.25)
        axes[2, column].set_xlabel("Time since first exported frame [s]")
    axes[0, 0].legend(fontsize=8)
    fig.suptitle(
        "MPC motion comparison — acceleration estimated from exported joint velocities"
    )
    fig.tight_layout()
    fig.savefig(output / "joint_comparison.png", dpi=160)
    fig.savefig(output / "joint_comparison.pdf")
    plt.close(fig)

    results = []
    fig, ax = plt.subplots(figsize=(12, 5))
    width = 0.8 / len(motions)
    x = np.arange(len(selected))
    for j, (motion, label) in enumerate(zip(motions, labels)):
        acceleration = motion["acceleration"][:, selected]
        rms_by_joint = np.sqrt(np.mean(acceleration ** 2, axis=0))
        result = {
            "label": label,
            "path": str(paths[j]),
            "frames": len(motion["time"]),
            "fps": motion["hz"],
            "upper_body_acceleration_rms_rad_s2": float(
                np.sqrt(np.mean(acceleration ** 2))
            ),
            "upper_body_acceleration_p99_rad_s2": float(
                np.percentile(np.abs(acceleration), 99)
            ),
            "upper_body_acceleration_max_rad_s2": float(np.max(np.abs(acceleration))),
            "upper_body_velocity_rms_rad_s": float(
                np.sqrt(np.mean(motion["joint_vel"][:, selected] ** 2))
            ),
        }
        for group, indices in groups.items():
            result[group + "_acceleration_rms_rad_s2"] = float(
                np.sqrt(np.mean(motion["acceleration"][:, indices] ** 2))
            )
        results.append(result)
        ax.bar(x + j * width, rms_by_joint, width=width, label=label)
    ax.set_xticks(x + width * (len(motions) - 1) / 2)
    ax.set_xticklabels(
        [names[i].replace("_joint", "") for i in selected],
        rotation=55,
        ha="right",
        fontsize=8,
    )
    ax.set_ylabel("RMS acceleration [rad/s²]")
    ax.set_title("Upper-body acceleration, all exported frames")
    ax.legend()
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output / "acceleration_rms.png", dpi=160)
    plt.close(fig)

    if urdf and all("left_hand_target_pos" in motion for motion in motions):
        fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        for motion, label, result in zip(motions, labels, results):
            errors = hand_tracking(motion, urdf)
            for axis, side in zip(axes, ("left", "right")):
                axis.plot(motion["time"][:-1], errors[side] * 100, label=label)
                axis.set_ylabel(side.capitalize() + " hand error [cm]")
                axis.grid(alpha=0.25)
                result[side + "_hand_position_rms_m"] = float(
                    np.sqrt(np.mean(errors[side] ** 2))
                )
        axes[0].legend()
        axes[1].set_xlabel("Time since first exported frame [s]")
        fig.suptitle(
            "Torso-frame hand position tracking — state/target timestamps aligned"
        )
        fig.tight_layout()
        fig.savefig(output / "hand_tracking.png", dpi=160)
        plt.close(fig)

    report = {
        "command_targets_identical": command_match,
        "shown_joints": [names[i] for i in shown],
        "motions": results,
    }
    (output / "metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    with (output / "metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    print(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motions", type=Path, nargs="+")
    parser.add_argument("--labels", nargs="+")
    parser.add_argument(
        "--joints",
        nargs="+",
        help="Exact joint names; default: most active waist, arm and wrist joints",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("/tmp/mpc_motion_plots")
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        help="Full G1 URDF, for Pinocchio hand-position tracking analysis",
    )
    parser.add_argument("--require-matched-targets", action="store_true")
    args = parser.parse_args()
    labels = args.labels or [path.stem for path in args.motions]
    if len(labels) != len(args.motions):
        parser.error("Provide one label per motion")
    compare(
        args.motions,
        labels,
        args.output_dir,
        args.joints,
        args.urdf,
        args.require_matched_targets,
    )


if __name__ == "__main__":
    main()
