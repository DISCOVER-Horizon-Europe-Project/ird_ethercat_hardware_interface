# Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
# Universitat Politecnica de Catalunya)
# SPDX-License-Identifier: GPL-3.0-only

"""First test of one IRD drive: ros2_control with the IRD EtherCAT interface.

Starts the controller manager with urdf/example_robot.urdf.xacro (one drive
turning `axis1_joint`), a joint_state_broadcaster and a velocity controller.

    ros2 launch ird_ethercat_hardware_interface example.launch.py ethercat_interface:=eno1
    ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"
    ros2 topic echo /joint_states

See docs/getting_started.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ird_ethercat_hardware_interface")

    arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([package_share, "config", "minimal_example.yaml"]),
            description="Configuration file of the drives (see config/ird_csv_example.yaml).",
        ),
        DeclareLaunchArgument(
            "ethercat_interface",
            default_value="",
            description="Network interface of the EtherCAT bus; empty = bus.interface of the configuration file.",
        ),
    ]

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([package_share, "urdf", "example_robot.urdf.xacro"]),
                " config_file:=", LaunchConfiguration("config_file"),
                " ethercat_interface:=", LaunchConfiguration("ethercat_interface"),
            ]
        ),
        value_type=str,
    )
    controllers = PathJoinSubstitution([package_share, "config", "example_controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "velocity_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        arguments
        + [
            control_node,
            robot_state_publisher,
            RegisterEventHandler(OnProcessStart(target_action=control_node, on_start=[spawner])),
        ]
    )
