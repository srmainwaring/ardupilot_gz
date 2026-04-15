"""
Python version of https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html
"""

import rclpy

from copy import deepcopy
from threading import Lock

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import PositionConstraint
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK

from sensor_msgs.msg import JointState

from shape_msgs.msg import SolidPrimitive

# no support for ros2
# import moveit_commander

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def pose_stamped_to_position_ik_request(
    pose_stamped,
    eef_link,
    group_name,
):
    # TODO: obtain joint state using query or SRDF
    robot_state = RobotState()
    robot_state.joint_state = JointState()
    robot_state.joint_state.name = [
        "Shoulder_Rotation",
        "Shoulder_Pitch",
        "Elbow",
        "Wrist_Pitch",
        "Wrist_Roll",
    ]
    robot_state.joint_state.position = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    ik_request = PositionIKRequest()
    ik_request.group_name = group_name
    ik_request.robot_state = robot_state
    # no constraints
    # ik_request.constraints
    ik_request.avoid_collisions = True
    ik_request.ik_link_name = eef_link
    ik_request.pose_stamped = pose_stamped
    # default planning time
    # ik_request.timeout = self.planning_time

    req = GetPositionIK.Request()
    req.ik_request = ik_request

    return req


def pose_stamped_to_move_group_goal(
    pose,
    eef_link,
    group_name,
    position_tolerance_m=0.02,
    orient_tolerance_rad=0.4,
    plan_only=False,
):
    req = MotionPlanRequest()
    # WorkspaceParameters
    req.workspace_parameters.min_corner = Vector3()
    req.workspace_parameters.min_corner.x = -1.0
    req.workspace_parameters.min_corner.y = -1.0
    req.workspace_parameters.min_corner.z = 0.0
    req.workspace_parameters.max_corner = Vector3()
    req.workspace_parameters.max_corner.x = 1.0
    req.workspace_parameters.max_corner.y = 1.0
    req.workspace_parameters.max_corner.z = 1.0
    # RobotState start_state

    req.pipeline_id = "ompl"
    req.planner_id = "RRTConnect"
    req.group_name = group_name
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.5
    req.max_acceleration_scaling_factor = 0.5

    # Constraints
    pc = PositionConstraint()
    pc.header = pose.header
    pc.link_name = eef_link
    pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
    pc.weight = 1.0

    sp = SolidPrimitive()
    sp.type = SolidPrimitive.SPHERE
    sp.dimensions = [float(position_tolerance_m)]

    sp_pose = Pose()
    sp_pose.position = pose.pose.position
    sp_pose.orientation.w = 1.0

    pc.constraint_region.primitives.append(sp)
    pc.constraint_region.primitive_poses.append(sp_pose)

    oc = OrientationConstraint()
    oc.header = pose.header
    oc.link_name = eef_link
    oc.orientation = pose.pose.orientation
    oc.absolute_x_axis_tolerance = float(orient_tolerance_rad)
    oc.absolute_y_axis_tolerance = float(orient_tolerance_rad)
    oc.absolute_z_axis_tolerance = float(orient_tolerance_rad)
    oc.weight = 1.0
    oc.parameterization = OrientationConstraint.XYZ_EULER_ANGLES

    c = Constraints()
    c.name = "goal"
    c.position_constraints.append(pc)
    c.orientation_constraints.append(oc)
    req.goal_constraints.append(c)

    # PlanningOptions
    opts = PlanningOptions()
    opts.plan_only = plan_only
    opts.look_around = False
    opts.replan = False

    g = MoveGroup.Goal()
    g.request = req
    g.planning_options = opts
    return g


def moveit_error_code_to_string(val):
    # overall behavior
    if val == MoveItErrorCodes.SUCCESS:
        return "SUCCESS"
    elif val == MoveItErrorCodes.FAILURE:
        return "FAILURE"
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return "PLANNING_FAILED"
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN"
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return "CONTROL_FAILED"
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "UNABLE_TO_AQUIRE_SENSOR_DATA"
    elif val == MoveItErrorCodes.TIMED_OUT:
        return "TIMED_OUT"
    elif val == MoveItErrorCodes.PREEMPTED:
        return "PREEMPTED"

    # planning & kinematics request errors
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION"
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_PATH_CONSTRAINTS"
    elif val == MoveItErrorCodes.START_STATE_INVALID:
        return "START_STATE_INVALID"
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION"
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_PATH_CONSTRAINTS"
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED"
    elif val == MoveItErrorCodes.GOAL_STATE_INVALID:
        return "GOAL_STATE_INVALID"
    elif val == MoveItErrorCodes.UNRECOGNIZED_GOAL_TYPE:
        return "UNRECOGNIZED_GOAL_TYPE"
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME"
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS"
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE"
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return "INVALID_LINK_NAME"
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return "INVALID_OBJECT_NAME"

    # system errors
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return "FRAME_TRANSFORM_FAILURE"
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return "COLLISION_CHECKING_UNAVAILABLE"
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return "ROBOT_STATE_STALE"
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return "SENSOR_INFO_STALE"
    elif val == MoveItErrorCodes.COMMUNICATION_FAILURE:
        return "COMMUNICATION_FAILURE"
    elif val == MoveItErrorCodes.CRASH:
        return "CRASH"
    elif val == MoveItErrorCodes.ABORT:
        return "ABORT"

    # kinematic errors
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return "NO_IK_SOLUTION"
    else:
        return "UNKNOWN"


class MoveToGoal(Node):
    def __init__(self):
        super().__init__("move_to_goal")

        # context
        self.planning_library = "ompl"

        # planning
        # self.declare_parameter("planning_group", "arm")
        # self.declare_parameter("eef_link", "End_Effector")

        self.planning_group = "arm"
        self.base_frame = "base_link"
        self.eef_link = "End_Effector"

        self.planning_time = 5.0
        self.velocity_scaling = 0.1
        self.acceleration_scaling = 0.1
        self.planning_attempts = 10

        self.collision_aware_ik = True
        self.approx_ik_solutions = True
        self.external_comm = True

        # subscribers
        self.pose_target_mutex = Lock()
        self.pose_target = Pose()
        self.pose_target_sub = self.create_subscription(
            Pose, "/pose_target", self.pose_target_callback, 10
        )

        # move_group service servers
        # moveit_msgs/srv/ApplyPlanningScene
        self.apply_planning_scene_name = "/apply_planning_scene"
        # moveit_msgs/srv/GetStateValidity
        self.check_state_validity_name = "/check_state_validity"
        # moveit_msgs/srv/GetPositionFK
        self.compute_fk_name = "/compute_fk"
        # moveit_msgs/srv/GetPositionIK
        self.compute_ik_name = "/compute_ik"
        # moveit_msgs/srv/GetMotionPlan
        self.plan_kinematic_path = "/plan_kinematic_path"
        # moveit_msgs/srv/GetMotionSequence
        self.plan_sequence_path = "/plan_sequence_path"

        # service clients
        self.compute_ik_client = self.create_client(GetPositionIK, self.compute_ik_name)

        # move_group action servers
        # moveit_msgs/action/MoveGroup
        self.action_name = "/move_action"

        # action clients
        self.action_client = ActionClient(self, MoveGroup, self.action_name)

        pose = Pose()
        pose.position.x = 0.169
        pose.position.y = -0.1957
        pose.position.z = 0.0873
        # pose.orientation.w = 0.0201
        # pose.orientation.x = 0.1072
        # pose.orientation.y = 0.7648
        # pose.orientation.z = 0.6349
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        self.pose_target_callback(pose)

    def pose_target_callback(self, msg: Pose):
        with self.pose_target_mutex:
            self.pose_target = deepcopy(msg)

        self.send_compute_ik_request(self.pose_target)
        # self.send_goal(self.pose_target)

    def send_compute_ik_request(self, pose_target):
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose_target

        req = pose_stamped_to_position_ik_request(
            ps,
            eef_link=self.eef_link,
            group_name=self.planning_group,
        )

        # TODO: give feedback?
        # while not self.compute_ik_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(f"Waiting for service {self.compute_ik_name}...")
        self.compute_ik_client.wait_for_service()
        self.compute_ik_future = self.compute_ik_client.call_async(req)
        self.compute_ik_future.add_done_callback(self.compute_ik_result_callback)

    def compute_ik_result_callback(self, future):
        self.get_logger().info("Compute IK result")
        result = future.result()
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            error_str = moveit_error_code_to_string(result.error_code.val)
            self.get_logger().info(f"Failed to compute IK: {error_str}")
        else:
            # result.solution is a RobotState
            robot_state = result.solution
            joint_state = robot_state.joint_state
            self.get_logger().info(f"frame_id: {joint_state.header.frame_id}")
            for name, position in zip(joint_state.name, joint_state.position):
                self.get_logger().info(f"{name}: {position}")

    def send_goal(self, pose_target):
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose_target

        # populate moveit_msgs/MoveGroup.action
        goal_msg = pose_stamped_to_move_group_goal(
            ps,
            eef_link=self.eef_link,
            group_name=self.planning_group,
            position_tolerance_m=0.1,
            orient_tolerance_rad=0.1,
            plan_only=False,
        )

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.get_logger().info("Goal response")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            error_str = moveit_error_code_to_string(result.error_code.val)
            self.get_logger().info(f"Failed to move arm: {error_str}")
        else:
            self.get_logger().info(f"Arm moved to pose target")

        self.get_logger().info(f"Result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)

    try:
        ex.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected")
    finally:
        node.get_logger().info("Shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
