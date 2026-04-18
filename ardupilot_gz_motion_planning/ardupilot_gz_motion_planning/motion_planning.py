"""
Motion planning for the iris_with_arm
"""

import rclpy
import time

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.planning import MoveItPy
from moveit.planning import MultiPipelinePlanRequestParameters
from rclpy.logging import get_logger


def plan_and_execute(
    node,
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
        node.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main(args=None):
    ###########################################################################
    # MoveItPy Setup
    ###########################################################################

    rclpy.init(args=args)

    logger = get_logger("motion_planning")
    rclpy.logging.set_logger_level(
        "motion_planning", rclpy.logging.LoggingSeverity.DEBUG
    )

    # instantiate MoveItPy instance and get planning component
    node = MoveItPy(node_name="moveit_py")
    logger.info("MoveItPy instance created")

    arm = node.get_planning_component("arm")
    logger.info("MoveItPy get 'arm' planning component\n")

    ###########################################################################
    # RobotModel
    ###########################################################################

    robot_model = node.get_robot_model()

    logger.info(f"robot name: {robot_model.name}")
    logger.info(f"model frame: {robot_model.model_frame}")
    logger.info(f"root joint name: {robot_model.root_joint_name}")
    logger.info(f"model info: {robot_model.get_model_info()}")

    for eef in robot_model.end_effectors:
        # NOTE: end effector returns a joint model group?
        logger.info(f"end effector: {eef}")

    for name in robot_model.joint_model_group_names:
        jmg = robot_model.get_joint_model_group(name)

        logger.info(f"joint model group: {jmg.name}")

        try:
            # NOTE: JointModelGroup.eef_name throws
            # TODO: why getting 'No end effector tips found in joint model group'?
            logger.info(f"eef name: {jmg.eef_name}")
        except:
            logger.info(f"joint model group [{jmg.name}] has no end effector")

        for joint_name in jmg.joint_model_names:
            logger.info(f"joint name: {joint_name}")

        for link_name in jmg.link_model_names:
            logger.info(f"link name: {link_name}")

        # NOTE: unsure why each joint model may have more than one bound?
        #       perhaps multiple dof per joint?
        logger.info(f"active joints and bounds")
        for joint_name, joint_bounds in zip(
            jmg.active_joint_model_names, jmg.active_joint_model_bounds
        ):
            logger.info(f"joint name: {joint_name}")
            for joint_bound in joint_bounds:
                logger.info(f"min position: {joint_bound.min_position}")
                logger.info(f"max position: {joint_bound.max_position}")
                logger.info(f"position bdd: {joint_bound.position_bounded}")
                logger.info(f"min velocity: {joint_bound.min_velocity}")
                logger.info(f"max velocity: {joint_bound.max_velocity}")
                logger.info(f"velocity bdd: {joint_bound.velocity_bounded}")

    ###########################################################################
    # RobotState
    ###########################################################################

    robot_state = RobotState(robot_model)

    # RobotState IK solver...
    logger.info(f"Set state from IK")
    pose_goal = Pose()
    pose_goal.position.x = 0.2
    pose_goal.position.y = -0.2
    pose_goal.position.z = 0.3
    robot_state.set_from_ik(
        joint_model_group_name="arm",
        geometry_pose=pose_goal,
        tip_name="End_Effector",
        # timeout=1.0,
    )

    # Call update to ensure all transforms are updated
    robot_state.update()

    logger.info(f"Joint positions")
    for name, value in robot_state.joint_positions.items():
        logger.info(f"{name}: {value}")

    # logger.info(f"Joint velocities")
    # for name, value in robot_state.joint_velocities.items():
    #     logger.info(f"{name}: {value}")

    # Display state tree and info
    # logger.info(f"Robot state tree")
    # logger.info(f"{robot_state.state_tree}")

    # TODO: not working as expected?
    # logger.info(f"Robot state info")
    # logger.info(f"{robot_state.state_info}")

    logger.info(f"Link poses")
    for name in robot_model.joint_model_group_names:
        jmg = robot_model.get_joint_model_group(name)
        logger.info(f"joint model group: {jmg.name}")
        for link_name in jmg.link_model_names:
            logger.info(f"link name: {link_name}")
            pose = robot_state.get_pose(link_name)
            logger.info(f"pose: {pose}")

    ###########################################################################
    # Planning
    ###########################################################################

    run_plan_1 = False
    run_plan_2 = False
    run_plan_3 = True
    run_plan_4 = False
    run_plan_5 = False

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    if run_plan_1:
        logger.info("MoveItPy start Plan 1")

        # NOTE: no need to set start state, and if it is set it must be close
        # set plan start state using predefined state
        # arm.set_start_state(configuration_name="init")

        # NOTE: set to current state instead
        # set plan start state to current state
        arm.set_start_state_to_current_state()

        # set pose goal using predefined state
        arm.set_goal_state(configuration_name="init")

        # plan to goal
        plan_and_execute(node, arm, logger, sleep_time=3.0)

        logger.info("MoveItPy Plan 1 done\n\n")
        time.sleep(1.0)

    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    if run_plan_2:
        logger.info("MoveItPy start Plan 2")

        # instantiate a RobotState instance using the current robot model
        robot_model = node.get_robot_model()
        robot_state = RobotState(robot_model)

        # randomize the robot state
        robot_state.set_to_random_positions()

        # set plan start state to current state
        arm.set_start_state_to_current_state()

        # set goal state to the initialized robot state
        logger.info("Set goal state to the initialized robot state")
        arm.set_goal_state(robot_state=robot_state)

        # plan to goal
        plan_and_execute(node, arm, logger, sleep_time=3.0)

        logger.info("MoveItPy Plan 2 done\n\n")
        time.sleep(1.0)

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    if run_plan_3:
        # NOTE: this one may fail

        logger.info("MoveItPy start Plan 3")

        # set plan start state to current state
        arm.set_start_state_to_current_state()

        # set workspace
        arm.set_workspace(
            min_x=-2.0, min_y=-2.0, min_z=0.0, max_x=2.0, max_y=2.0, max_z=2.0
        )

        # set pose goal with PoseStamped message
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.2
        pose_goal.pose.position.y = -0.2
        pose_goal.pose.position.z = 0.3
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="End_Effector")

        # plan to goal
        plan_and_execute(node, arm, logger, sleep_time=3.0)

        logger.info("MoveItPy Plan 3 done\n\n")
        time.sleep(1.0)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    if run_plan_4:
        logger.info("MoveItPy start Plan 4")

        # set plan start state to current state
        arm.set_start_state_to_current_state()

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
            joint_model_group=node.get_robot_model().get_joint_model_group("arm"),
        )
        arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        plan_and_execute(node, arm, logger, sleep_time=3.0)

        logger.info("MoveItPy Plan 4 done\n\n")
        time.sleep(1.0)

    ###########################################################################
    # Plan 5 - Planning with Multiple Pipelines simultaneously
    ###########################################################################

    if run_plan_5:
        logger.info("MoveItPy start Plan 5")

        # set plan start state to current state
        arm.set_start_state_to_current_state()

        # set pose goal with PoseStamped message
        arm.set_goal_state(configuration_name="init")

        # TODO: only have one planner configured
        # initialise multi-pipeline plan request parameters
        # multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        #     node, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
        # )
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
            node, ["ompl_rrtc"]
        )

        # plan to goal
        plan_and_execute(
            node,
            arm,
            logger,
            multi_plan_parameters=multi_pipeline_plan_request_params,
            sleep_time=3.0,
        )

        logger.info("MoveItPy Plan 5 done, shutting down...\n\n")
        time.sleep(1.0)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
