# Whole-Body Humanoid MPC

This repository contains a Whole-Body Nonlinear Model Predictive Controller (NMPC) for humanoid loco-manipulation control. This approach enables to directly optimize through the **full-order torque-level dynamics in realtime** to generate a wide range of humanoid behaviors building up on an [extended & updated version of ocs2](https://github.com/manumerous/ocs2_ros2)

**Interactive Velocity and Base Height Control via Joystick:**

![vokoscreenNG-2025-12-21_20-35-31-ezgif com-optimize](https://github.com/user-attachments/assets/daf374ba-fe82-469d-9270-63d18a51bb53)


It contains the following hardware platform agnostic MPC fromulations:

### Centroidal Dynamics MPC
The centroidal MPC optimizes over the **whole-body kinematics** and the center off mass dynamics, with a choice to either use a single rigid 
body model or the full centroidal dynamics. This specific approach builds up on the centroidal model in ocs2 by generalizing costs and constraints to a 6 DoF contact among others. I am still working on documenting this. Until then a conscise explanation of the ocs2 centroidal model can be found here [Sleiman et. al., A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation](https://arxiv.org/abs/2103.00946)

### Whole-Body Dynamics MPC
The **whole-body dynamics** MPC optimized over the contact forces and joint accelerations with the option to compute the joint torques for 
each step planned accross the horizon. I am still working on documenting and publishing the approach. The most relevant information on the choosen approach can currently be found in [Galliker et al., Bipedal Locomotion with Nonlinear Model Predictive Control:
Online Gait Generation using Whole-Body Dynamics](http://ames.caltech.edu/galliker2022bipedal.pdf)
### Robot Examples

The project supports the following robot examples:

- Unitree G1
- 1X Neo (Comming soon)

![Screencast2024-12-16180254-ezgif com-optimize(3)](https://github.com/user-attachments/assets/d4b1f0da-39ca-4ce1-b53c-e1d040abe1be)

## Get Started

### Setup Colcon Workspace

Create a colcon workspace and clone the repository into the src folder:

```bash
mkdir -p humanoid_mpc_ws/src && cd humanoid_mpc_ws/src
git clone https://github.com/1x-technologies/wb-humanoid-mpc.git
```

Then initialize all submodules using:

```bash
cd wb-humanoid-mpc
git submodule update --init --recursive
```
### Install Dependencies
The project supports both Dockerized workspaces (recommended) or a local installation for developing and running the humanoid MPC. 

<details>
<summary>Build & run Dockerized workspace in VS Code</summary>

We provide a [Dockerfile](https://github.com/manumerous/wb_humanoid_mpc/blob/main/docker/Dockerfile) to enable running and devloping the project from a containerized environment. Check out the [devcontainer.json](https://github.com/manumerous/wb_humanoid_mpc/blob/main/.devcontainer/devcontainer.json) for the arguments that must be supplied to the `docker build` and `docker run` commands.

For working in **Visual Studio Code**, we recommend to install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension. Then, with the root of this repository as the root of your VS Code workspace, enter `Ctrl + Shift + P` and select `Dev Containers: Rebuild and Reopen in Container` at the top of the screen. VS Code will then automatically handle calling the `docker build` and `docker run` commands for you and will reopen the window at the root of the containerized workspace. Once this step is completed, you are ready to [build and run the code](https://github.com/manumerous/wb_humanoid_mpc/tree/main?tab=readme-ov-file#building-the-mpc).

</details>

<details>
<summary> Build & run Dockerized workspace with bash scripts</summary>

This repository includes two helper scripts: `image_build.bash` builds the `wb-humanoid-mpc:dev` Docker image using the arguments defined in `devcontainer.json`. `launch_wb_mpc.bash` starts the Docker container, mounts your workspace, and drops you into a bash shell ready to build and run the WB Humanoid MPC code. Example of building docker image:
```
cd /path/to/humanoid_mpc_ws/src/wb_humanoid_mpc/docker
./image_build.bash
```
and launching the docker container:
```
cd /path/to/humanoid_mpc_ws/src/wb_humanoid_mpc/docker
./launch_wb_mpc.bash
```

</details>

<details>
<summary>Install Dependencies Locally</summary>

Make sure you have **ros2** installed on your system as e.g specified for jazzy in
the [installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Then install all dependencies using:

```bash
envsubst < dependencies.txt | xargs sudo apt install -y
```
</details>


### Building the MPC 

Building the WB MPC consumes a significant amount of RAM. We recommend saving all open work before starting the first build. The RAM usage can be adjusted by setting the PARALLEL_JOBS environment variable. Our recommendation is:

| PARALLEL_JOBS | Required System RAM |
|--------------:|--------------------:|
| 2 (default)   |  16 GiB             | 
| 4             |  32 GiB              |
| 6             |  64 GiB              | 


```bash
make build-all
```

## Running the examples
Once you run the NMPC a window with Rviz will appear for visualization. The first time you start the MPC for a certain robot model the auto differentiation code will be generated which might take up to 5-15 min depending on your system. Once done the robot appears and you can control it via an xbox gamepad or the controls in the terminal. 

On the top level folder run:

For the **Centroidal Dynamics MPC**

```
make launch-g1-dummy-sim
```

For the **Whole-Body Dynamics MPC**

```
make launch-wb-g1-dummy-sim
```

For the **Centroidal Dynamics MPC with Cartesian hand pose references**

```
make launch-g1-dummy-sim-hands-cartesian
```

The hand references are expressed in the `pelvis` frame and can be sent through ROS 2 topics. To send commands while the simulation is running, open another terminal in the same Docker container. If you are using VS Code Dev Containers, open a new integrated terminal.

```bash
 docker exec -it wb-mpc-dev bash
```

Then send a right hand pose reference:

```bash
ros2 topic pub --once /g1/right_hand_pose_reference geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'pelvis'},
  pose: {
    position: {x: 0.250, y: -0.150, z: 0.10},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

Another example target is:

```bash
ros2 topic pub --once /g1/right_hand_pose_reference geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'pelvis'},
  pose: {
    position: {x: 0.35, y: -0.200, z: 0.20},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

You can check the tracked hand pose with:

```bash
ros2 run tf2_ros tf2_echo pelvis right_rubber_hand
```

To publish MPC references for a downstream CLAMP/RL WBC, launch the centroidal dummy sim with:

```bash
make launch-g1-dummy-sim-mpc-motion-reference
```

or, for the Cartesian hands variant:

```bash
make launch-g1-dummy-sim-hands-cartesian-mpc-motion-reference
```

Both dummy-sim task files enable the same soft foot-separation cost used by the random-reference generator. Tune `foot_separation_cost.weight` or `foot_separation_cost.minLateralSeparation` in `robot_models/unitree_g1/g1_centroidal_mpc/config/mpc/task.info` or `task_hands_cartesian.info`, then rebuild/relaunch.

This publishes `/g1/mpc_motion_reference`, containing the named joint position/velocity reference, root pose/twist in world, and the flattened CLAMP command layout:

```text
[joint_pos, joint_vel, base_vx_body, base_vy_body, base_yaw_rate_body, base_height, base_roll, base_pitch]
```

To record this stream as a CLAMP-compatible NPZ for offline policy playback, open a second terminal in the same Docker container and run:

```bash
ros2 run humanoid_common_mpc_pyutils mpc_motion_reference_recorder \
  --topic /g1/mpc_motion_reference \
  --output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/g1_mpc_motion_reference.npz \
  --duration 10.0
```

The resulting NPZ contains `joint_pos`, `joint_vel`, `body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w`, `body_names`, and `fps`. It will be available on the host at `/home/famadio/Workspace/wb_humanoid_mpc/g1_mpc_motion_reference.npz`.

To generate a CLAMP-style NPZ directly from the centroidal SQP without ROS 2 topics, dummy sim, or visualization, run:

```bash
make generate-g1-random-mpc-npz
```

This writes `/wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_random_mpc_reference.npz`, visible on the host at `/home/famadio/Workspace/wb_humanoid_mpc/generated_motions/g1_random_mpc_reference.npz`. The generator saves a compressed NPZ and samples smooth random base velocity, pelvis height, yaw-rate, waist joints, and arm joints. For this generator, the wrist joints are kept in the MPC model, so the exported motion includes the full 29-DOF G1 joint set. The default random command ranges are `vx [-0.5, 1.0] m/s`, `vy [-0.5, 0.5] m/s`, `yaw_rate [-1.0, 1.0] rad/s`, and `height [0.4, 0.8] m`. The base command velocity, height, stance, and heading segments are resampled every `3.0` to `9.0` seconds by default. With default probability `0.20`, a command velocity segment is forced to stance with zero linear and angular base velocity. With default probability `0.80`, a command velocity segment uses heading control: a target heading is sampled in `[-pi, pi]`, configurable with `--heading-min` and `--heading-max`, then the yaw-rate command is computed from the wrapped heading error and clamped to the yaw-rate range. The waist targets are clamped to the `[-90, 90] deg` range and resampled every `1.5` to `4.5` seconds; the shoulder and elbow targets are resampled every `2.0` to `6.0` seconds; the wrist targets are resampled every `1.5` to `4.5` seconds. With default probability `0.20`, each waist/arm/wrist segment keeps the previous joint reference instead of drawing a new target; if this happens at `t=0`, it keeps the nominal `defaultJointState`. The random-reference task also enables a soft foot-separation cost that penalizes the left foot getting too close to or crossing over the right foot in the pelvis yaw frame; tune `foot_separation_cost.weight` and `foot_separation_cost.minLateralSeparation` in `task_random_reference.info` if it is too weak or too restrictive. The gait is otherwise selected from the same procedural ladder used by the dummy example: `stance`, `slow_walk`, `walk`, `slower_trot`, `slow_trot`, `trot`, and `run`, using the same velocity thresholds. The gait timings are read from the existing `humanoid_common_mpc/config/command/gait.info`.

To generate several independent motions in one run, pass `--num-motions` through the make target:

```bash
make generate-g1-random-mpc-npz GENERATOR_ARGS="--num-motions 10"
```

When `--num-motions` is larger than `1`, the output path is expanded with a numbered suffix, for example `g1_random_mpc_reference_0000.npz`, `g1_random_mpc_reference_0001.npz`, and so on. Each attempt runs in a fresh child process, so a failed rollout or a native crash such as a segmentation fault only discards that attempt instead of killing the full batch. By default each output motion gets up to `20` attempts.

For basic locomotion datasets with the upper body fixed, set `--upper-body-fixed-probability 1.0`, disable heading control with `--heading-prob 0.0`, and zero the velocity axes that should not be active. Except for the stance-height example, these commands also pin the base-height reference to the nominal `defaultBaseHeight` with `--height-min 0.7925 --height-max 0.7925`. The following examples keep the command ranges inside the generator defaults:

```bash
make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_forward.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 0.0 --vx-min 0.2 --vx-max 1.0 --vy-min 0.0 --vy-max 0.0 --yaw-rate-min 0.0 --yaw-rate-max 0.0 --height-min 0.7925 --height-max 0.7925"

make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_backward.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 0.0 --vx-min -0.5 --vx-max -0.1 --vy-min 0.0 --vy-max 0.0 --yaw-rate-min 0.0 --yaw-rate-max 0.0 --height-min 0.7925 --height-max 0.7925"

make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_left.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 0.0 --vx-min 0.0 --vx-max 0.0 --vy-min 0.1 --vy-max 0.5 --yaw-rate-min 0.0 --yaw-rate-max 0.0 --height-min 0.7925 --height-max 0.7925"

make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_right.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 0.0 --vx-min 0.0 --vx-max 0.0 --vy-min -0.5 --vy-max -0.1 --yaw-rate-min 0.0 --yaw-rate-max 0.0 --height-min 0.7925 --height-max 0.7925"

make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_turn_left.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 0.0 --vx-min 0.0 --vx-max 0.0 --vy-min 0.0 --vy-max 0.0 --yaw-rate-min 0.2 --yaw-rate-max 1.0 --height-min 0.7925 --height-max 0.7925"

make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_turn_right.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 0.0 --vx-min 0.0 --vx-max 0.0 --vy-min 0.0 --vy-max 0.0 --yaw-rate-min -1.0 --yaw-rate-max -0.2 --height-min 0.7925 --height-max 0.7925"

make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_stance_height.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 1.0 --vx-min 0.0 --vx-max 0.0 --vy-min 0.0 --vy-max 0.0 --yaw-rate-min 0.0 --yaw-rate-max 0.0 --height-min 0.4 --height-max 0.8"

make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_forward_turn_left.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 0.0 --vx-min 0.2 --vx-max 1.0 --vy-min 0.0 --vy-max 0.0 --yaw-rate-min 0.2 --yaw-rate-max 1.0 --height-min 0.7925 --height-max 0.7925"

make generate-g1-random-mpc-npz GENERATOR_ARGS="--output /wb_humanoid_mpc_ws/src/wb_humanoid_mpc/generated_motions/g1_basic_forward_turn_right.npz --num-motions 10 --upper-body-fixed-probability 1.0 --heading-prob 0.0 --stance-probability 0.0 --vx-min 0.2 --vx-max 1.0 --vy-min 0.0 --vy-max 0.0 --yaw-rate-min -1.0 --yaw-rate-max -0.2 --height-min 0.7925 --height-max 0.7925"
```

You can also call the generator directly after sourcing the workspace:

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
  --upper-body-fixed-probability 0.20
```

#### Interactive Robot Control
Command a desired base velocity and root link height via **Robot Base Controller GUI** and **XBox Controller Joystick**. For the joystick it is easiest to directly connect via USB. Otherwise you need to install the required bluetooth Xbox controller drivers on your linux system. The GUI application automatically scanns for Joysticks and indicates whether one is connected. 

![robot_remote_control](https://github.com/user-attachments/assets/779be1da-97a1-4d0c-8f9b-b9d2df88384f)


## Citing Whole-Body Humanoid MPC
To cite the Whole-Body Humanoid MPC in your academic research, please consider citing the following web BibTeX entry:

```
@misc{wholebodyhumanoidmpcweb,
   author = {Manuel Yves Galliker},
   title = {Whole-body Humanoid MPC: Realtime Physics-Based Procedural Loco-Manipulation Planning and Control},
   howpublished = {https://github.com/1x-technologies/wb_humanoid_mpc},
   year = {2024}
}
```

## Acknowledgements
Created and actively maintained by [Manuel Yves Galliker](https://github.com/manumerous).

Special thanks go to [Nicholas Palermo](https://github.com/nicholaspalomo) for implementing the dockerization among other great inputs and contributions. 

This project is founded on the great work of many open-source contributors. I would especially like to acknowledge:
- [ocs2](https://github.com/leggedrobotics/ocs2)
- [pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [hpipm](https://github.com/giaf/hpipm)
  
Part of this work was developed during my time at [1X Technologies](https://www.1x.tech/). I would like to kindly thank Eric Jang and Bernt Børnich for supporting the open sourcing of this project. 

Further I would like to thank Michael Purcell, Jesper Smith, Simon Zimmermann, Joel Filho, Paal Arthur Schjelderup Thorseth, Varit (Ohm) Vichathorn, Sjur Grønnevik Wroldsen, Armin Nurkanovic, Charles Khazoom and Farbod Farshidian for the many fruitful discussions, insights, contributions and support. 
