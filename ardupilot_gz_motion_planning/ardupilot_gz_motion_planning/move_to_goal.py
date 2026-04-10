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

from shape_msgs.msg import SolidPrimitive

# no support for ros2
# import moveit_commander

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


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
        self.comput_fk_name = "/compute_fk"
        # moveit_msgs/srv/GetPositionIK
        self.comput_ik_name = "/compute_ik"
        # moveit_msgs/srv/GetMotionPlan
        self.plan_kinematic_path = "/plan_kinematic_path"
        # moveit_msgs/srv/GetMotionSequence
        self.plan_sequence_path = "/plan_sequence_path"

        # move_group action servers
        # moveit_msgs/action/MoveGroup
        self.action_name = "/move_action"

        # action clients
        self.action_client = ActionClient(self, MoveGroup, self.action_name)

        pose = Pose()
        pose.position.x = 0.169
        pose.position.y = -0.1957
        pose.position.z = 0.0873
        pose.orientation.w = 0.0201
        pose.orientation.x = 0.1072
        pose.orientation.y = 0.7648
        pose.orientation.z = 0.6349

        self.pose_target_callback(pose)


    def pose_target_callback(self, msg: Pose):
        with self.pose_target_mutex:
            self.pose_target = deepcopy(msg)

        self.send_goal(self.pose_target)

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
            position_tolerance_m=0.02,
            orient_tolerance_rad=0.04,
            plan_only=False)

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
            self.get_logger().info(f"Failed to move arm")
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
