from ament_index_python.packages import get_package_share_directory

import launch
from humanoid_common_mpc_ros2.mpc_launch_config import MPCLaunchConfig
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # Add parameters
    cfg.ld.add_action(cfg.declare_robot_name)
    cfg.ld.add_action(cfg.declare_config_file)
    cfg.ld.add_action(cfg.declare_target_command_file)
    cfg.ld.add_action(cfg.declare_gait_command_file)
    cfg.ld.add_action(cfg.declare_urdf_path)
    cfg.ld.add_action(cfg.declare_rviz_config_path)
    cfg.ld.add_action(
        DeclareLaunchArgument(
            "publish_reference_joint_states",
            default_value="false",
            description="Publish live MPC joint position/velocity references as sensor_msgs/JointState.",
        )
    )

    reference_joint_state_publisher_node = Node(
        package="humanoid_centroidal_mpc_ros2",
        executable="humanoid_centroidal_mpc_reference_joint_state_node",
        name="reference_joint_state_publisher",
        output="screen",
        arguments=[
            LaunchConfiguration("robot_name"),
            LaunchConfiguration("config_name"),
            LaunchConfiguration("target_command_file"),
            LaunchConfiguration("description_name"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_reference_joint_states")),
    )

    cfg.ld.add_action(
        DeclareLaunchArgument(
            "publish_mpc_motion_reference",
            default_value="false",
            description="Publish MPC references in the CLAMP motion command layout.",
        )
    )

    mpc_motion_reference_publisher_node = Node(
        package="humanoid_centroidal_mpc_ros2",
        executable="humanoid_centroidal_mpc_motion_reference_node",
        name="mpc_motion_reference_publisher",
        output="screen",
        arguments=[
            LaunchConfiguration("robot_name"),
            LaunchConfiguration("config_name"),
            LaunchConfiguration("target_command_file"),
            LaunchConfiguration("description_name"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_mpc_motion_reference")),
    )

    cfg.ld.add_action(
        DeclareLaunchArgument(
            "publish_mpc_future_motion_reference",
            default_value="false",
            description="Publish MPC current+future references in the YAHMP-Future compact motion command layout.",
        )
    )

    mpc_future_motion_reference_publisher_node = Node(
        package="humanoid_centroidal_mpc_ros2",
        executable="humanoid_centroidal_mpc_motion_reference_node",
        name="mpc_future_motion_reference_publisher",
        output="screen",
        arguments=[
            LaunchConfiguration("robot_name"),
            LaunchConfiguration("config_name"),
            LaunchConfiguration("target_command_file"),
            LaunchConfiguration("description_name"),
            "--publish-future-motion-ref",
        ],
        condition=IfCondition(LaunchConfiguration("publish_mpc_future_motion_reference")),
    )

    # Add nodes
    cfg.ld.add_action(cfg.mpc_node)
    cfg.ld.add_action(cfg.dummy_sim_node)
    cfg.ld.add_action(reference_joint_state_publisher_node)
    cfg.ld.add_action(mpc_motion_reference_publisher_node)
    cfg.ld.add_action(mpc_future_motion_reference_publisher_node)
    cfg.ld.add_action(cfg.robot_state_publisher_node)
    cfg.ld.add_action(cfg.terminal_robot_state_publisher_node)
    cfg.ld.add_action(cfg.target_robot_state_publisher_node)
    cfg.ld.add_action(cfg.rviz_node)
    cfg.ld.add_action(cfg.base_velocity_controller_gui_node)

    return cfg.ld
