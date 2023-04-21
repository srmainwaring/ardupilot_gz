# Copyright 2023 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
A ROS interface for spawning models into Gazebo.

A launch file for the `ros_gz_sim create` node. 

See: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/src/create.cpp
"""
from typing import List

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_create_model(
    context: LaunchContext, *args, **kwargs
) -> List[LaunchDescriptionEntity]:
    """Return a launch description to create a model in Gazebo."""
    arg_names = [
        "world",
        "file",
        "param",
        "string",
        "topic",
        "name",
        "allow_renaming",
        "x",
        "y",
        "z",
        "R",
        "P",
        "Y",
    ]

    # Populate non-empty args.
    args = []
    for arg_name in arg_names:
        arg = LaunchConfiguration(arg_name).perform(context)
        if arg:
            args += [f"-{arg_name}", arg]

    # Create action.
    create_model = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=args,
        output="screen",
    )
    return [create_model]


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description to create a model in Gazebo."""
    launch_arguments = generate_launch_arguments()

    return LaunchDescription(
        launch_arguments + [OpaqueFunction(function=launch_create_model)]
    )


def generate_launch_arguments() -> List[DeclareLaunchArgument]:
    """Generate a list of launch arguments."""
    return [
        DeclareLaunchArgument(
            "world",
            default_value="",
            description="World name.",
        ),
        DeclareLaunchArgument(
            "file",
            default_value="",
            description="Load XML from a file.",
        ),
        DeclareLaunchArgument(
            "param",
            default_value="",
            description="Load XML from a ROS param.",
        ),
        DeclareLaunchArgument(
            "string",
            default_value="",
            description="Load XML from a string.",
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="",
            description="Load XML from a ROS string publisher.",
        ),
        DeclareLaunchArgument(
            "name",
            default_value="",
            description="Name for spawned entity.",
        ),
        DeclareLaunchArgument(
            "allow_renaming",
            default_value="",
            description="Rename entity if name already used.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="",
            description="X component of initial position (meters).",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="",
            description="Y component of initial position (meters).",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="",
            description="Z component of initial position (meters).",
        ),
        DeclareLaunchArgument(
            "R",
            default_value="",
            description="Roll component of initial orientation (radians).",
        ),
        DeclareLaunchArgument(
            "P",
            default_value="",
            description="Pitch component of initial orientation (radians).",
        ),
        DeclareLaunchArgument(
            "Y",
            default_value="",
            description="Yaw component of initial orientation (radians).",
        ),
    ]
