"""
Python version of https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html
"""

import rclpy
import math
import numpy as np

from copy import deepcopy
from threading import Lock

from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import PositionConstraint
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import VisibilityConstraint
from moveit_msgs.srv import GetPositionIK

from sensor_msgs.msg import JointState

from shape_msgs.msg import SolidPrimitive

from tf2_ros import Buffer, TransformListener

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from transforms3d import affines
from transforms3d import euler
from transforms3d import quaternions

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


def follow_joint_traj_error_code_to_string(val):
    """Convert a FollowJointTrajectory error code to a string"""
    result = FollowJointTrajectory.Result()
    if val == result.SUCCESSFUL:
        return "SUCCESSFUL"
    elif val == result.INVALID_GOAL:
        return "INVALID_GOAL"
    elif val == result.INVALID_JOINTS:
        return "INVALID_JOINTS"
    elif val == result.OLD_HEADER_TIMESTAMP:
        return "OLD_HEADER_TIMESTAMP"
    elif val == result.PATH_TOLERANCE_VIOLATED:
        return "PATH_TOLERANCE_VIOLATED"
    elif val == result.GOAL_TOLERANCE_VIOLATED:
        return "GOAL_TOLERANCE_VIOLATED"
    else:
        return "UNKNOWN"


def joint_state_to_follow_joint_traj_goal(
    joint_state,
):
    """Create a FollowJointTrajectory.Goal from a JointState"""
    jtp = JointTrajectoryPoint()
    jtp.positions = joint_state.position
    jtp.velocities = [0.0 for x in joint_state.position]
    jtp.time_from_start.sec = 2
    jtp.time_from_start.nanosec = 0

    jt = JointTrajectory()
    # jt.header = joint_state.header
    jt.joint_names = joint_state.name
    jt.points.append(jtp)

    g = FollowJointTrajectory.Goal()
    g.trajectory = jt

    print(f"{g.trajectory}")

    return g


def pose_stamped_to_position_ik_request(
    pose_stamped,
    eef_link,
    group_name,
    position_tolerance_m=0.02,
    orient_tolerance_rad=0.4,
    collision_aware_ik=False,
    approx_ik_solutions=False,
):
    """Create a GetPositionIK.Request from a PoseStamped"""
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
    ik_request.avoid_collisions = collision_aware_ik
    ik_request.ik_link_name = eef_link
    ik_request.pose_stamped = pose_stamped
    # default planning time
    # ik_request.timeout = self.planning_time

    # Constraints
    if approx_ik_solutions:
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = eef_link
        pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        pc.weight = 1.0

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [float(position_tolerance_m)]

        sp_pose = Pose()
        sp_pose.position = pose_stamped.pose.position
        sp_pose.orientation.w = 1.0

        pc.constraint_region.primitives.append(sp)
        pc.constraint_region.primitive_poses.append(sp_pose)

        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = eef_link
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = float(orient_tolerance_rad)
        oc.absolute_y_axis_tolerance = float(orient_tolerance_rad)
        oc.absolute_z_axis_tolerance = float(orient_tolerance_rad)
        oc.weight = 1.0
        oc.parameterization = OrientationConstraint.XYZ_EULER_ANGLES

        c = Constraints()
        c.name = "goal"
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        ik_request.constraints = c

    # Goal
    req = GetPositionIK.Request()
    req.ik_request = ik_request

    return req


def pose_stamped_to_move_group_goal(
    pose,
    eef_link,
    group_name,
    position_tolerance_m=0.02,
    orient_tolerance_rad=0.4,
    velocity_scaling=0.5,
    acceleration_scaling=0.5,
    num_planning_attempts=3,
    plan_only=False,
):
    """Create a MoveGroup.Goal from a PoseStamped"""
    req = MotionPlanRequest()
    # WorkspaceParameters
    req.workspace_parameters.min_corner = Vector3()
    req.workspace_parameters.min_corner.x = -2.0
    req.workspace_parameters.min_corner.y = -2.0
    req.workspace_parameters.min_corner.z = 0.0
    req.workspace_parameters.max_corner = Vector3()
    req.workspace_parameters.max_corner.x = 2.0
    req.workspace_parameters.max_corner.y = 2.0
    req.workspace_parameters.max_corner.z = 2.0

    # RobotState
    # TODO: get current to use as start state
    start_state = RobotState()
    start_state.joint_state = JointState()
    start_state.joint_state.name = [
        "Shoulder_Rotation",
        "Shoulder_Pitch",
        "Elbow",
        "Wrist_Pitch",
        "Wrist_Roll",
    ]
    start_state.joint_state.position = [
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
    req.start_state = start_state

    # TODO: except group_name, options should come from params
    req.pipeline_id = "ompl"
    req.planner_id = "RRTConnect"
    req.group_name = group_name
    req.num_planning_attempts = num_planning_attempts
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = velocity_scaling
    req.max_acceleration_scaling_factor = acceleration_scaling

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
    """Convert a MoveItErrorCodes error code to a string"""
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

        # Parameters
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eef_link", "End_Effector")
        self.declare_parameter("plan_only", False)
        self.declare_parameter("use_move_action", True)
        self.declare_parameter("collision_aware_ik", True)
        self.declare_parameter("approx_ik_solutions", True)
        self.declare_parameter("use_target_pos_for_yaw", True)

        self.planning_group = (
            self.get_parameter("planning_group").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.eef_link = (
            self.get_parameter("eef_link").get_parameter_value().string_value
        )
        self.plan_only = (
            self.get_parameter("plan_only").get_parameter_value().bool_value
        )
        self.use_move_action = (
            self.get_parameter("use_move_action").get_parameter_value().bool_value
        )
        self.collision_aware_ik = (
            self.get_parameter("collision_aware_ik").get_parameter_value().bool_value
        )
        self.approx_ik_solutions = (
            self.get_parameter("approx_ik_solutions").get_parameter_value().bool_value
        )
        self.use_target_pos_for_yaw = (
            self.get_parameter("use_target_pos_for_yaw")
            .get_parameter_value()
            .bool_value
        )

        # context
        self.planning_library = "ompl"

        self.position_tolerance_m = 0.01
        self.orient_tolerance_rad = 0.4
        self.planning_time = 5.0
        self.velocity_scaling = 0.75
        self.acceleration_scaling = 0.75
        self.planning_attempts = 3
        self.external_comm = False

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
        self.move_action_name = "/move_action"
        self.follow_joint_traj_action_name = "/arm_controller/follow_joint_trajectory"

        # action clients
        self.move_action_client = ActionClient(self, MoveGroup, self.move_action_name)
        self.follow_joint_traj_action_client = ActionClient(
            self, FollowJointTrajectory, self.follow_joint_traj_action_name
        )

        # activity state
        self.busy = False
        self.ready = False

        # transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_arm = TransformStamped()
        self.tf_shd = TransformStamped()
        self.tf_mutex = Lock()

        # Check for the transform periodically (50 Hz)
        self.timer = self.create_timer(0.02, self.get_pose)

    def get_pose(self):
        # TODO: make the reference link 'world' configurable
        try:
            with self.tf_mutex:
                self.tf_arm: TransformStamped = self.tf_buffer.lookup_transform(
                    "world", "base_link", rclpy.time.Time()
                )
            self.ready = True
        except:
            self.ready = False
            self.get_logger().info("Waiting for valid transform: world to base_link")

        try:
            with self.tf_mutex:
                self.tf_shd: TransformStamped = self.tf_buffer.lookup_transform(
                    "world", "Shoulder_Rotation_Pitch", rclpy.time.Time()
                )
            self.ready = True
        except:
            self.ready = False
            self.get_logger().info(
                "Waiting for valid transform: world to Shoulder_Rotation_Pitch"
            )

    def pose_target_callback(self, msg: Pose):
        """Called when a Pose is received on /pose_target"""
        # Return early if still wating for TF
        if not self.ready:
            return

        with self.pose_target_mutex:
            self.pose_target = deepcopy(msg)

        # TODO: move linear algebra to a separate function

        # {w} coordinates
        with self.tf_mutex:
            w_p_arm = deepcopy(self.tf_arm.transform.translation)
            w_p_shd = deepcopy(self.tf_shd.transform.translation)
        w_p_tgt = self.pose_target.position
        self.get_logger().info(f"w_p_arm: {w_p_arm}")
        self.get_logger().info(f"w_p_shd: {w_p_shd}")
        self.get_logger().info(f"w_p_tgt: {w_p_tgt}")

        # {w} rotations (quaternion)
        with self.tf_mutex:
            w_q_arm = self.tf_arm.transform.rotation

        # base_link pose in world frame
        w_X_b = np.array(
            affines.compose(
                T=[w_p_arm.x, w_p_arm.y, w_p_arm.z],
                R=quaternions.quat2mat([w_q_arm.w, w_q_arm.x, w_q_arm.y, w_q_arm.z]),
                Z=np.ones(3),
            )
        )

        # and its inverse
        b_X_w = np.linalg.inv(w_X_b)

        # {w} vectors
        w_v_arm_shd = np.array(
            [w_p_shd.x - w_p_arm.x, w_p_shd.y - w_p_arm.y, w_p_shd.z - w_p_arm.z]
        )
        w_v_arm_tgt = np.array(
            [w_p_tgt.x - w_p_arm.x, w_p_tgt.y - w_p_arm.y, w_p_tgt.z - w_p_arm.z]
        )
        w_v_shd_tgt = w_v_arm_tgt - w_v_arm_shd
        self.get_logger().info(f"w_v_arm_shd: {w_v_arm_shd}")
        self.get_logger().info(f"w_v_arm_tgt: {w_v_arm_tgt}")
        self.get_logger().info(f"w_v_shd_tgt: {w_v_shd_tgt}")

        # {w} vectors - homogeneous form obtained by zero-padding
        w_v_arm_shd_h = np.pad(w_v_arm_shd, (0, 1))
        w_v_arm_tgt_h = np.pad(w_v_arm_tgt, (0, 1))
        w_v_shd_tgt_h = np.pad(w_v_shd_tgt, (0, 1))

        # {b} vectors
        b_v_arm_shd = np.linalg.matmul(b_X_w, w_v_arm_shd_h)[0:3]
        b_v_arm_tgt = np.linalg.matmul(b_X_w, w_v_arm_tgt_h)[0:3]
        b_v_shd_tgt = np.linalg.matmul(b_X_w, w_v_shd_tgt_h)[0:3]
        self.get_logger().info(f"b_v_arm_shd: {b_v_arm_shd}")
        self.get_logger().info(f"b_v_arm_tgt: {b_v_arm_tgt}")
        self.get_logger().info(f"b_v_shd_tgt: {b_v_shd_tgt}")

        # yaw angles: unit reference {b} vectors
        b_v_unit_x = np.array([1, 0, 0])
        b_v_unit_y = np.array([0, -1, 0])

        proj_tgt_x = np.dot(b_v_shd_tgt / np.linalg.norm(b_v_shd_tgt), b_v_unit_x)
        proj_tgt_y = np.dot(b_v_shd_tgt / np.linalg.norm(b_v_shd_tgt), b_v_unit_y)
        theta_z = np.atan2(proj_tgt_x, proj_tgt_y)
        self.get_logger().info(f"theta_z: {math.degrees(theta_z):.2f} deg")

        # roll and pitch angles (hardcoded)
        theta_x = np.radians(45)
        theta_y = np.radians(0)

        # {b} target pose
        b_X_tgt = Pose()
        b_X_tgt.position.x = b_v_arm_tgt[0]
        b_X_tgt.position.y = b_v_arm_tgt[1]
        b_X_tgt.position.z = b_v_arm_tgt[2]
        b_X_tgt.orientation.w = 1.0
        b_X_tgt.orientation.x = 0.0
        b_X_tgt.orientation.y = 0.0
        b_X_tgt.orientation.z = 0.0

        # Set the yaw based on the angle between the shoulder rotation link
        # and target position, otherwise use the full pose.
        if self.use_target_pos_for_yaw:
            q = euler.euler2quat(theta_x, theta_y, theta_z)
            b_X_tgt.orientation.w = q[0]
            b_X_tgt.orientation.x = q[1]
            b_X_tgt.orientation.y = q[2]
            b_X_tgt.orientation.z = q[3]
        else:
            q = [
                self.pose_target.orientation.w,
                self.pose_target.orientation.x,
                self.pose_target.orientation.y,
                self.pose_target.orientation.z,
            ]
        # ensure pose target orientation is normalised
        if not quaternions.qisunit(q):
            q_norm = quaternions.qnorm(q)
            self.get_logger().warn(f"Quaternion not normalised: {q_norm}")
            if q_norm > 0.0:
                b_X_tgt.orientation.x /= q_norm
                b_X_tgt.orientation.y /= q_norm
                b_X_tgt.orientation.z /= q_norm
                b_X_tgt.orientation.w /= q_norm

        if not self.busy:
            self.get_logger().info(f"Processing pose target: {b_X_tgt}")
            self.busy = True

            if self.use_move_action:
                self.move_action_send_goal(b_X_tgt)
            else:
                self.compute_ik_send_request(b_X_tgt)
        else:
            self.get_logger().warn("Busy, pose target will be ignored")

    def compute_ik_send_request(self, pose_target):
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose_target

        req = pose_stamped_to_position_ik_request(
            ps,
            eef_link=self.eef_link,
            group_name=self.planning_group,
            position_tolerance_m=self.position_tolerance_m,
            orient_tolerance_rad=self.orient_tolerance_rad,
            collision_aware_ik=self.collision_aware_ik,
            approx_ik_solutions=self.approx_ik_solutions,
        )

        # TODO: give feedback while waiting?
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
            self.busy = False
        else:
            # result.solution is a RobotState
            robot_state = result.solution
            joint_state = robot_state.joint_state
            self.get_logger().info(f"frame_id: {joint_state.header.frame_id}")
            for name, position in zip(joint_state.name, joint_state.position):
                self.get_logger().info(f"{name}: {position}")

            # move arm if successful using /arm_controller/follow_joint_trajectory
            # TODO: need better filter of valid joints for follow joint traj
            js = JointState()
            js.header.frame_id = joint_state.header.frame_id
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = joint_state.name[0:-1]
            js.position = joint_state.position[0:-1]

            # move to pose if plane and execute
            if not self.plan_only:
                self.follow_joint_traj_send_goal(js)

        self.get_logger().info("Compute IK done")

    def follow_joint_traj_send_goal(self, joint_state):
        self.get_logger().info("FollowJointTrajectory send goal")
        goal_msg = joint_state_to_follow_joint_traj_goal(joint_state)

        self.follow_joint_traj_action_client.wait_for_server()
        self.follow_joint_traj_send_goal_future = (
            self.follow_joint_traj_action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.follow_joint_traj_feedback_callback,
            )
        )
        self.follow_joint_traj_send_goal_future.add_done_callback(
            self.follow_joint_traj_action_goal_response_callback
        )

    def follow_joint_traj_action_goal_response_callback(self, future):
        self.get_logger().info("FollowJointTrajectory goal response")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("FollowJointTrajectory goal rejected")
            self.busy = False
            return

        self.get_logger().info("FollowJointTrajectory goal accepted")
        self.follow_joint_traj_result_future = goal_handle.get_result_async()
        self.follow_joint_traj_result_future.add_done_callback(
            self.follow_joint_traj_result_callback
        )

    def follow_joint_traj_result_callback(self, future):
        self.get_logger().info("FollowJointTrajectory goal result")
        result = future.result().result
        if result.error_code != result.SUCCESSFUL:
            error_str = follow_joint_traj_error_code_to_string(result.error_code)
            self.get_logger().info(
                f"Failed to move arm: {error_str}: {result.error_string}"
            )
        else:
            self.get_logger().info(f"Arm moved to pose target")
        # self.get_logger().info(f"Result: {result}")
        self.busy = False

    def follow_joint_traj_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"FollowJointTrajectory feedback: {feedback}")

    def move_action_send_goal(self, pose_target):
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose_target

        # populate moveit_msgs/MoveGroup.action
        goal_msg = pose_stamped_to_move_group_goal(
            ps,
            eef_link=self.eef_link,
            group_name=self.planning_group,
            position_tolerance_m=self.position_tolerance_m,
            orient_tolerance_rad=self.orient_tolerance_rad,
            num_planning_attempts=self.planning_attempts,
            plan_only=self.plan_only,
        )

        self.move_action_client.wait_for_server()
        self.move_action_send_goal_future = self.move_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.move_action_feedback_callback,
        )
        self.move_action_send_goal_future.add_done_callback(
            self.move_action_goal_response_callback
        )

    def move_action_goal_response_callback(self, future):
        self.get_logger().info("MoveGroup goal response")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("MoveGroup goal rejected")
            self.busy = False
            return

        self.get_logger().info("MoveGroup goal accepted")
        self.move_action_get_result_future = goal_handle.get_result_async()
        self.move_action_get_result_future.add_done_callback(
            self.move_action_result_callback
        )

    def move_action_result_callback(self, future):
        result = future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            error_str = moveit_error_code_to_string(result.error_code.val)
            self.get_logger().info(f"Failed to move arm: {error_str}")
        else:
            self.get_logger().info(f"Arm moved to pose target")

        self.get_logger().info(f"Result: {result}")
        self.busy = False

    def move_action_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"MoveGroup feedback: {feedback}")


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
