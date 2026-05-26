# G1 Centroidal Dynamics MPC

This repository contains a centroidal-dynamics nonlinear MPC stack for humanoid loco-manipulation on the Unitree G1, built on top of an extended ROS 2 port of OCS2. A concise explanation of the OCS2 centroidal model is available here: [Sleiman et al., A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation](https://arxiv.org/abs/2103.00946).

This project derives from [`wb_humanoid_mpc`](https://github.com/manumerous/wb_humanoid_mpc) by [Manuel Yves Galliker](https://github.com/manumerous), who established the original software architecture and implementation.

Relative to the original Centroidal Dynamics MPC, this repository adds:

- integration of a hand-pose tracking task
- integration of a waist-DOF tracking task
- a motion-reference export pipeline for MPC+RL control with [**YAHMP**](https://github.com/hucebot/yahmp)

## Overview

The centroidal MPC optimizes whole-body kinematics together with centroidal dynamics, with support for locomotion, upper-body posture targets, Cartesian hand references, and downstream MPC motion-reference export.

The main user-facing workflows in this repo are:

- dummy simulation with RViz visualization
- MuJoCo simulation
- Cartesian hand reference control through RViz interactive markers and ROS topics
- Vive-based locomotion command input
- random NPZ motion reference generation

## Docker Workflow

### Build the Docker Image

From the repo root:

```bash
cd docker
./image_build.bash
```

This builds the `wb-humanoid-mpc:dev` image defined by [docker/Dockerfile](docker/Dockerfile).

### Launch the Container

From the same `docker` directory:

```bash
./launch_wb_mpc.bash
```

This script starts a container named `wb-mpc-dev`, mounts the repository into `/wb_humanoid_mpc_ws/src/wb_humanoid_mpc`, and keeps workspace artifacts on the host in `.docker_ws/`. If the container is already running, re-running the same script opens a new shell inside it with the ROS environment and workspace overlay sourced automatically.

## Build Inside Docker

Once inside the container:

```bash
make build-all
```

The first build can be heavy on RAM, especially when auto-differentiation code is generated for the first time.

`PARALLEL_JOBS=6` is used by default. You can override the build parallelism, for example:

```bash
make PARALLEL_JOBS=2 build-all
```

Recommended RAM by `PARALLEL_JOBS`:

| PARALLEL_JOBS | Required RAM |
|--------------:|-------------:|
| 2             | 16 GiB       |
| 4             | 32 GiB       |
| 6             | 64 GiB       |

## Run the MPC

There are two main runtime tasks in this repo:

- the **locomotion task**, which controls base velocity, root height, and waist joints. It uses the reduced G1 model with fixed wrist joints
- the **hand-pose task**, which adds Cartesian hand-pose tracking on top of the locomotion task. It uses the full 29-DOF G1 model

For tuning, start from:

- locomotion MPC task: [task_locomotion.info](robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_locomotion.info)
- hand-pose MPC task: [task_hand_pose.info](robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_hand_pose.info)
- runtime reference and command settings: [reference.info](robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference.info)

### Dummy Simulation

Launch the **locomotion task** in the dummy sim. This is a lightweight closed-loop rollout of the centroidal model driven by the current MPC policy, without the MuJoCo physics backend:

```bash
make launch-g1-dummy-sim-locomotion
```

This expects the base/height/waist command on:

```text
/humanoid/walking_velocity_command
```

Launch the **hand-pose task** in the dummy sim:

```bash
make launch-g1-dummy-sim-hand-pose
```

Besides the base/height/waist command, this expects torso-frame hand pose commands on:

```text
/g1/left_hand_pose_reference
/g1/right_hand_pose_reference
```

The first time you launch a given configuration, code generation may take several minutes.

### MuJoCo Simulation

Launch the **locomotion task** in the MuJoCo sim:

```bash
make launch-g1-sim-locomotion
```

Launch the **hand-pose task** in the MuJoCo sim:

```bash
make launch-g1-sim-hand-pose
```

## Hand Pose Control

The hand-pose launch exposes torso-frame hand references for both hands.

RViz includes interactive markers under `Hand Pose Markers`. Moving them publishes:

- `/g1/left_hand_pose_reference`
- `/g1/right_hand_pose_reference`

Both are expected in the `torso_link` frame.

## Vive Locomotion Control

To send commands to the MPC from an external Vive teleop source without the RViz interactive markers, use:

```bash
make launch-g1-dummy-sim-teleop
```

To use that teleop setup while also publishing `/g1/mpc_motion_reference` for the RL policy, use:

```bash
make launch-g1-dummy-sim-teleop-pub-mpc-motion-ref
```

The teleop node subscribes to:

- `/vive/right/joint_states`
- `/vive/left/joint_states`

It is launched by the teleop launch variants and publishes over:

- `/g1/left_hand_pose_reference` for the desired left hand pose
- `/g1/right_hand_pose_reference` for the desired right hand pose
- `/humanoid/walking_velocity_command` for the desired walking velocity

The walking-velocity mapping from the Vive controllers is:

- from `/vive/right/joint_states`: `trackpad_x` and `trackpad_y` control linear velocity `x/y` only when `trackpad_pressed == 1`
- from `/vive/left/joint_states`: `trackpad_y` controls yaw velocity only when `trackpad_pressed == 1`

## MPC Motion Reference Export

The centroidal MPC can publish compact motion references for downstream tracking policies such as [**YAHMP**](https://github.com/hucebot/yahmp). The payload is always a flattened `float32[] motion_cmd`; only the payload layout changes.

By default, the exported command contains joint positions and base motion:

```text
[joint_pos, base_vx_body, base_vy_body, base_yaw_rate_body, base_height, base_roll, base_pitch]
```

This uses:

- `humanoid_mpc_msgs/msg/MpcMotionJointPos` for the current reference
- `humanoid_mpc_msgs/msg/MpcFutureMotionJointPos` for the future reference

To switch from the default position-only message to the joint-state message, set:

```bash
MPC_MOTION_REFERENCE_TYPE=joint_state
```

for Make targets, or pass this launch argument directly:

```bash
mpc_motion_reference_type:=joint_state
```

The joint-state layout includes joint velocities:

```text
[joint_pos, joint_vel, base_vx_body, base_vy_body, base_yaw_rate_body, base_height, base_roll, base_pitch]
```

This uses the matching `MpcMotionJointState` and `MpcFutureMotionJointState` messages.

To switch back to the lightweight default, use `joint_pos`.

To publish the current MPC reference on `/g1/mpc_motion_reference`:

```bash
make launch-g1-dummy-sim-locomotion-pub-mpc-motion-ref
```

or with the hand-pose task:

```bash
make launch-g1-dummy-sim-hand-pose-pub-mpc-motion-ref
```

To publish the current reference plus future samples on `/g1/mpc_future_motion_reference`:

```bash
make launch-g1-dummy-sim-locomotion-pub-mpc-future-motion-ref
```

or with the hand-pose task:

```bash
make launch-g1-dummy-sim-hand-pose-pub-mpc-future-motion-ref
```

The future message adds:

- `steps`: YAHMP-Future step offsets, currently `[0, 4, 8, ..., 48]`
- `dt`: the YAHMP control step, currently `0.02 s`
- `motion_cmd`: one selected-layout block per future step, concatenated in step-major order

## Deployment via HURo

Deployment on the real G1 robot has been tested using [**HURo**](https://github.com/hucebot/huro).

To communicate with the HURo Docker container in simulation, use:

```bash
source setup_uri.sh lo
```

To communicate with the HURo Docker container and the real robot, use:

```bash
source setup_uri.sh <eth-interface>
```

where `<eth-interface>` is the name of the ethernet interface used to connect to the robot.

## Random MPC Motion Reference Generation

### Joint-Reference Random Generator

To generate random motion references and store them directly to NPZ:

```bash
make generate-g1-random-mpc-npz
```

This writes:

```bash
/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_random_mpc_reference.npz
```

The generator samples smooth random trajectories for:

- base velocity
- pelvis height
- yaw rate
- waist joints
- arm joints

This random-generation task uses the full 29-DOF G1 MPC model.

To generate a named locomotion primitive instead of fully random commands, pass `--basic-primitive`:

```bash
make generate-g1-random-mpc-npz GENERATOR_ARGS="--basic-primitive walk_forward"
```

Available primitives are `stand`, `walk_forward`, `walk_backward`, `walk_left`, `walk_right`, `turn_left`, `turn_right`, `walk_forward_turn_left`, and `walk_forward_turn_right`. Primitive commands are smoothly resampled only along the selected axis or axes, between a nonzero minimum and the configured maximum velocity. The default minimum is 60% of the axis maximum; tune it with `--basic-primitive-min-speed-ratio`.

For tuning the random generator, start from:

- random-generation MPC task: [task_random_reference.info](robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_random_reference.info)
- random-generation reference and command settings: [reference_random_reference.info](robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference_random_reference.info)

### Hand-Pose Random Generator

To generate random hand-pose motions with the hand-pose task:

```bash
make generate-g1-random-hand-pose-mpc-npz
```

This writes:

```bash
/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_random_hand_pose_mpc_reference.npz
```

The hand-pose generator keeps the pelvis height fixed, samples smooth Cartesian hand-pose targets in the torso frame, and switches between two random modes:

- `manipulation`: zero base velocity, stance behavior, full hand workspace
- `walking`: random locomotion command, reduced hand workspace

New hand-pose targets are sampled as bounded local steps from the previous target, then clamped to the configured workspace. This avoids occasional large hand jumps while still exploring the workspace over time. Manipulation resamples hand targets faster than walking by default; tune this with `segment_min`, `segment_max`, and `manipulation_probability` in `reference_random_hand_pose.info`.

The hand-pose generator also supports the same `--basic-primitive` presets. In that mode, the hands stay at the default task references, the pelvis height stays fixed, and only the selected base primitive is commanded:

```bash
make generate-g1-random-hand-pose-mpc-npz GENERATOR_ARGS="--basic-primitive walk_forward"
```

By default, the hand-pose NPZ stores only the generated motion. To also store the sampled command targets, run:

```bash
make generate-g1-random-hand-pose-mpc-npz GENERATOR_ARGS="--save-additional-info"
```

This adds:

- `base_command.npy`
- `left_hand_target_pos.npy`
- `right_hand_target_pos.npy`
- `left_hand_target_quat.npy`
- `right_hand_target_quat.npy`
- `random_mode.npy`

For tuning the hand-pose generator, start from:

- hand-pose MPC task: [task_hand_pose.info](robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_hand_pose.info)
- hand-pose random reference and sampling ranges: [reference_random_hand_pose.info](robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference_random_hand_pose.info)

#### Batch Generation

To generate multiple motions:

```bash
make generate-g1-random-mpc-npz GENERATOR_ARGS="--num-motions 10"
```

For the hand-pose generator, use:

```bash
make generate-g1-random-hand-pose-mpc-npz GENERATOR_ARGS="--num-motions 10"
```

Each output gets a numbered suffix such as:

- `g1_random_mpc_reference_0000.npz`
- `g1_random_mpc_reference_0001.npz`
- `g1_random_hand_pose_mpc_reference_0000.npz`
- `g1_random_hand_pose_mpc_reference_0001.npz`

Use `GENERATOR_ARGS` to pass generator options such as `--duration`, `--fps`, `--seed`, `--num-motions`, or `--save-additional-info`.

## Acknowledgements

Special thanks to [Manuel Yves Galliker](https://github.com/manumerous), author of the original [`wb_humanoid_mpc`](https://github.com/manumerous/wb_humanoid_mpc), for the foundational implementation and for open-sourcing this line of work.

This project also builds on the work of many open-source contributors, in particular:

- [ocs2](https://github.com/leggedrobotics/ocs2)
- [pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [hpipm](https://github.com/giaf/hpipm)
