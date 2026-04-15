from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build config from package `so_arm_100_moveit_config`
    moveit_config = MoveItConfigsBuilder("so_arm_100").to_moveit_configs()

    tutorial_node = Node(
        package="so_arm_100_moveit",
        executable="so_arm_100_moveit",
        namespace="",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        tutorial_node
    ])
