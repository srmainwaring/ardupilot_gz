"""
Motion planning for the iris_with_arm

Usage

Start robot node

Start moveit nodes

"""

import rclpy
import time

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.planning import MoveItPy
from moveit.planning import MultiPipelinePlanRequestParameters
from rclpy.logging import get_logger


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main(args=None):
    ###########################################################################
    # MoveItPy Setup
    ###########################################################################

    rclpy.init(args=args)
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    robot = MoveItPy(node_name="moveit_py")
    logger.info("MoveItPy instance created")

    robot_arm = robot.get_planning_component("arm")
    logger.info("MoveItPy get 'arm' planning component")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    robot_arm.set_start_state(configuration_name="ready")

    # set pose goal using predefined state
    robot_arm.set_goal_state(configuration_name="extended")

    # plan to goal
    plan_and_execute(robot, robot_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    # instantiate a RobotState instance using the current robot model
    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    robot_arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    robot_arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(robot, robot_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    robot_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    robot_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="Fixed_Gripper")

    # plan to goal
    plan_and_execute(robot, robot_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    # set plan start state to current state
    robot_arm.set_start_state_to_current_state()

    # set constraints message

    joint_values = {
        "Shoulder_Rotation": 0.0,
        "Shoulder_Pitch": 0.0,
        "Elbow": 0.0,
        "Wrist_Pitch": 0.0,
        "Wrist_Roll": 0.0,
    }
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=robot.get_robot_model().get_joint_model_group("arm"),
    )
    robot_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(robot, robot_arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 5 - Planning with Multiple Pipelines simultaneously
    ###########################################################################

    # set plan start state to current state
    robot_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    robot_arm.set_goal_state(configuration_name="ready")

    # initialise multi-pipeline plan request parameters
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        robot, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    )

    # plan to goal
    plan_and_execute(
        robot,
        robot_arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
    )


if __name__ == "__main__":
    main()
