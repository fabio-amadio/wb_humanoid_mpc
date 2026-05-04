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
- GUI-based base, height, and waist command input
- random-motion NPZ motion reference generation

## Docker Workflow

### Build the Docker Image

From the repo root:

```bash
cd docker
./image_build.bash
```

This builds the `wb-humanoid-mpc:dev` image defined by [docker/Dockerfile](/home/famadio/Workspace/wb_humanoid_mpc/docker/Dockerfile).

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

- locomotion MPC task: [task_locomotion.info](/home/famadio/Workspace/wb_humanoid_mpc/robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_locomotion.info)
- hand-pose MPC task: [task_hand_pose.info](/home/famadio/Workspace/wb_humanoid_mpc/robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_hand_pose.info)
- runtime reference and command settings: [reference.info](/home/famadio/Workspace/wb_humanoid_mpc/robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference.info)

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

## Base/Height/Waist Control

The GUI publishes:

- base linear velocity
- base yaw rate
- root height
- waist yaw, roll, and pitch

It is launched automatically by the standard G1 launch files above.

## Hand Pose Control

The hand-pose launch exposes torso-frame hand references for both hands.

RViz includes interactive markers under `Hand Pose Markers`. Moving them publishes:

- `/g1/left_hand_pose_reference`
- `/g1/right_hand_pose_reference`

Both are expected in the `torso_link` frame.

## MPC Motion Reference Export

The centroidal MPC can be used to generate motion references for RL-based tracking policies such as [**YAHMP**](https://github.com/hucebot/yahmp).

To publish the motion reference computed by the MPC for downstream consumers:

```bash
make launch-g1-dummy-sim-locomotion-pub-mpc-motion-ref
```

or, with the hand-pose task:

```bash
make launch-g1-dummy-sim-hand-pose-pub-mpc-motion-ref
```

This publishes `/g1/mpc_motion_reference`, which contains:

- named joint position and velocity references
- root pose and twist in world
- a flattened motion-command layout

The flattened command layout matches the one adopted by [**YAHMP**](https://github.com/hucebot/yahmp):

```text
[joint_pos, joint_vel, base_vx_body, base_vy_body, base_yaw_rate_body, base_height, base_roll, base_pitch]
```

To publish compact motions references with the current step plus future steps:

```bash
make launch-g1-dummy-sim-locomotion-pub-mpc-future-motion-ref
```

or, with the hand-pose task:

```bash
make launch-g1-dummy-sim-hand-pose-pub-mpc-future-motion-ref
```

This publishes `/g1/mpc_future_motion_reference`, which contains:

- `header`
- `steps`: YAHMP-Future step offsets, currently `[0, 4, 8, ..., 48]`
- `dt`: the YAHMP control step, currently `0.02 s`
- `motion_cmd`: a step-major concatenation of the compact layout above, one block per listed step

### Deployment via HURo

Deployment on the real G1 robot has been tested using [**HURo**](https://github.com/hucebot/huro).

To communicate with the HURo docker in simulation, use:

```bash
source setup_uri.sh lo
```

Instead, to communicate with the HURo docker and the real robot, use:

```bash
source setup_uri.sh <eth-interface>
```

where `<eth-interface>` is the name of the ethernet interface used to connect to the robot.

### Random MPC Motion Reference Generation

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

For tuning the random generator, start from:

- random-generation MPC task: [task_random_reference.info](/home/famadio/Workspace/wb_humanoid_mpc/robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_random_reference.info)
- random-generation reference and command settings: [reference_random_reference.info](/home/famadio/Workspace/wb_humanoid_mpc/robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference_random_reference.info)

#### Batch Generation

To generate multiple motions:

```bash
make generate-g1-random-mpc-npz GENERATOR_ARGS="--num-motions 10"
```

Each output gets a numbered suffix such as:

- `g1_random_mpc_reference_0000.npz`
- `g1_random_mpc_reference_0001.npz`

#### Direct Generator Usage

You can also run the generator directly:

```bash
ros2 run humanoid_centroidal_mpc_ros2 humanoid_centroidal_mpc_random_reference_generator \
  --task-file /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task_random_reference.info \
  --reference-file /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/robot_models/unitree_g1/g1_centroidal_mpc/config/command/reference_random_reference.info \
  --output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_random_mpc_reference.npz \
  --duration 20.0 \
  --fps 30.0 \
  --seed 1 \
  --num-motions 10 \
  --max-attempts-per-motion 20 \
  --stance-probability 0.20 \
  --heading-prob 0.80 \
  --heading-min -3.14159 \
  --heading-max 3.14159 \
  --heading-control-stiffness 1.0 \
  --base-command-deadband 0.1 \
  --upper-body-fixed-probability 0.20
```

## Acknowledgements

Special thanks to [Manuel Yves Galliker](https://github.com/manumerous), author of the original [`wb_humanoid_mpc`](https://github.com/manumerous/wb_humanoid_mpc), for the foundational implementation and for open-sourcing this line of work.

This project also builds on the work of many open-source contributors, in particular:

- [ocs2](https://github.com/leggedrobotics/ocs2)
- [pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [hpipm](https://github.com/giaf/hpipm)
