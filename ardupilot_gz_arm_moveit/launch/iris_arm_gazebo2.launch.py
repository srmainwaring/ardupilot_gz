import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

# from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable

# from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.substitutions import FindPackageShare


from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch

# from moveit_configs_utils.launches import generate_setup_assistant_launch
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from moveit_configs_utils.launches import generate_spawn_controllers_launch
from moveit_configs_utils.launches import generate_move_group_launch

# from moveit_configs_utils.launch_utils import add_debuggable_node
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

PKG_CONFIG = "ardupilot_gz_arm_moveit"


def _load_iris_sdf():
    pkg_gz = get_package_share_directory("ardupilot_gz_description")
    with open(os.path.join(pkg_gz, "models", "iris_with_arm", "model.sdf"), "r") as f:
        sdf = f.read()
    sdf = sdf.replace(
        "model://iris_with_standoffs",
        "package://ardupilot_gazebo/models/iris_with_standoffs",
    )
    sdf = sdf.replace(
        "model://learm",
        "package://ardupilot_gz_description/models/learm",
    )
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        os.environ["SDF_PATH"] = os.environ.get("SDF_PATH", "") + ":" + gz_path
    return sdf


def generate_launch_arguments():
    return [
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        ),
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        ),
        DeclareBooleanLaunchArg(
            "use_rviz", default_value=True, description="Open RViz"
        ),
        DeclareBooleanLaunchArg(
            "use_static_virtual_joint_tfs",
            default_value=True,
            description="If there are virtual joints, broadcast static tf",
        ),
    ]


def generate_launch_description():
    pkg_moveit_config = get_package_share_directory(PKG_CONFIG)

    # Manually load the SDF to allow substitutions
    robot_description = {"robot_description": _load_iris_sdf()}

    # learm.urdf.xacro includes <ros2_control> elements
    # arm_urdf_path = Path(os.path.join(pkg_moveit_config, "urdf", "learm.urdf.xacro"))
    arm_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(PKG_CONFIG), "urdf", "learm.urdf.xacro"]
            ),
        ]
    )
    arm_robot_description = {
        "robot_description": ParameterValue(arm_urdf, value_type=str)
    }

    srdf_path = Path(os.path.join(pkg_moveit_config, "srdf", "iris_with_arm.srdf"))

    kinematics_path = Path(
        os.path.join(
            pkg_moveit_config,
            "config",
            "kinematics.yaml",
        )
    )

    moveit_controllers_path = Path(
        os.path.join(
            pkg_moveit_config,
            "config",
            "moveit_controllers.yaml",
        )
    )

    ros2_controllers_path = Path(
        os.path.join(
            pkg_moveit_config,
            "config",
            "ros2_controllers.yaml",
        )
    )

    joint_limits_path = Path(
        os.path.join(
            pkg_moveit_config,
            "config",
            "joint_limits.yaml",
        )
    )

    pilz_cartesian_limits_path = Path(
        os.path.join(
            pkg_moveit_config,
            "config",
            "pilz_cartesian_limits.yaml",
        )
    )

    # Build config. All overrides must be file paths.
    moveit_config = (
        MoveItConfigsBuilder("iris_arm", package_name="ardupilot_gz_arm_moveit")
        # .robot_description()
        .robot_description_semantic(srdf_path)
        .robot_description_kinematics(kinematics_path)
        # .planning_pipelines()
        .trajectory_execution(moveit_controllers_path)
        # .planning_scene_monitor()
        # .sensors_3d()
        # .move_group_capabilities()
        .joint_limits(joint_limits_path)
        # .moveit_cpp()
        .pilz_cartesian_limits(pilz_cartesian_limits_path)
        .to_moveit_configs()
    )

    # Modify config. All override must be dictionaries.
    moveit_config.robot_description = robot_description

    # Following moveit_configs_utils.launches.generate_demo_launch but calling
    # nested generators directly rather than via an IncludeLaunchDescription so
    # the moveit_config can be passed to each generator.
    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    ld.add_action(
        DeclareBooleanLaunchArg("use_static_virtual_joint_tfs", default_value=True)
    )

    # TODO: add robot state publisher for robot_description, and leave the
    # argument to moveit empty (so it uses the published topic)
    #
    # Robot state publisher for the robot SDF
    # ld.add_action(
    #     Node(
    #         package="robot_state_publisher",
    #         executable="robot_state_publisher",
    #         name="robot_state_publisher",
    #         output="both",
    #         parameters=[
    #             robot_description,
    #             {"frame_prefix": ""},
    #         ],
    #         remappings=[
    #             ("/robot_description", "/robot_description"),
    #         ],
    #     )
    # )

    # If there are virtual joints, broadcast static tf.
    # Inspects SRDF for virtual joints
    ld.add_action(
        generate_static_virtual_joint_tfs_launch(moveit_config),
        # condition=IfCondition(LaunchConfiguration("use_static_virtual_joint_tfs")),
    )

    # Given the published joint states, publish tf for the robot links
    # LaunchArguments
    # - publish_frequency: default: 15.0
    ld.add_action(generate_rsp_launch(moveit_config))

    ld.add_action(generate_move_group_launch(moveit_config))

    # Run Rviz and load the default config to see the state of the move_group node
    # LaunchArguments
    # - rviz_config: default: config/moveit.rviz
    ld.add_action(
        generate_moveit_rviz_launch(moveit_config),
        # condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # TODO: use_sim_time should be LaunchConfiguration("use_sim_time")
    # TODO:
    #
    # Passing the robot description parameter directly to the control_manager
    # node is deprecated. Use ~/robot_description topic from
    # robot_state_publisher instead.
    #
    # Parameters:
    #   use_sim_time
    #   lock_memory
    #   overruns.manage
    #   thread_priority
    #   cpu_affinity
    #
    #
    # See: https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html

    # Robot state publisher for the arm URDF
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="arm_state_publisher",
            output="both",
            parameters=[
                arm_robot_description,
                {"frame_prefix": ""},
            ],
            remappings=[
                ("/robot_description", "/arm/robot_description"),
            ],
        )
    )

    # Controllers
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                ros2_controllers_path,
                {"use_sim_time": True},
            ],
            remappings=[
                ("/robot_description", "/arm/robot_description"),
            ],
        )
    )

    # TODO: check which controllers are actually spawned, and which
    #       robot_description is used (does it include the xacro?)

    # Spawn a joint state broadcaster and all controllers listed in the
    # moveit_simple_controller_manager section of the yaml file in
    # moveit_config.trajectory_execution
    ld.add_action(generate_spawn_controllers_launch(moveit_config))

    return ld
