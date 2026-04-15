import os

from humanoid_common_mpc_ros2.mpc_launch_config import MPCLaunchConfig
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    cfg = MPCLaunchConfig(
        mpc_lib_pkg="humanoid_centroidal_mpc",
        mpc_config_pkg="g1_centroidal_mpc",
        mpc_model_pkg="g1_description",
        urdf_rel_path="/urdf/g1_29dof.urdf",
        xml_rel_path="/urdf/g1_29dof.xml",
        robot_name="g1",
        solver="sqp",
        enable_debug=False,
    )

    cfg.declare_config_file = DeclareLaunchArgument(
        "config_name",
        default_value=os.path.join(cfg.mpc_config_pkg_dir, "config/mpc/task_hands_cartesian.info"),
        description="Path to MPC config file",
    )

    cfg.ld.add_action(cfg.declare_robot_name)
    cfg.ld.add_action(cfg.declare_config_file)
    cfg.ld.add_action(cfg.declare_target_command_file)
    cfg.ld.add_action(cfg.declare_gait_command_file)
    cfg.ld.add_action(cfg.declare_urdf_path)
    cfg.ld.add_action(cfg.declare_rviz_config_path)

    cfg.ld.add_action(cfg.mpc_node)
    cfg.ld.add_action(cfg.dummy_sim_node)
    cfg.ld.add_action(cfg.robot_state_publisher_node)
    cfg.ld.add_action(cfg.terminal_robot_state_publisher_node)
    cfg.ld.add_action(cfg.target_robot_state_publisher_node)
    cfg.ld.add_action(cfg.rviz_node)
    cfg.ld.add_action(cfg.base_velocity_controller_gui_node)

    return cfg.ld
