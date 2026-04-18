"""
A launch file for running motion planning for the iris_with_arm
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    so_arm_100_moveit_config_path = get_package_share_directory(
        "so_arm_100_moveit_config"
    )

    ardupilot_gz_motion_planning_path = get_package_share_directory(
        "ardupilot_gz_motion_planning"
    )

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether to use simulation time",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="so_arm_100",
            package_name="so_arm_100_moveit_config",
        )
        .robot_description_semantic(
            str(Path(so_arm_100_moveit_config_path) / "config" / "so_arm_100.srdf")
        )
        .joint_limits(
            str(Path(so_arm_100_moveit_config_path) / "config" / "joint_limits.yaml")
        )
        .trajectory_execution(
            str(
                Path(so_arm_100_moveit_config_path)
                / "config"
                / "moveit_controllers.yaml"
            )
        )
        .robot_description_kinematics(
            str(Path(so_arm_100_moveit_config_path) / "config" / "kinematics.yaml")
        )
        .moveit_cpp(
            file_path=str(
                Path(ardupilot_gz_motion_planning_path)
                / "config"
                / "motion_planning.yaml"
            )
        )
        .to_moveit_configs()
    )

    motion_planning_node = Node(
        package="ardupilot_gz_motion_planning",
        executable="motion_planning",
        name="moveit_py",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
            },
        ],
        # arguments=["--ros-args", "--log-level", "DEBUG"],
    )

    return LaunchDescription(
        arguments
        + [
            motion_planning_node,
        ]
    )
