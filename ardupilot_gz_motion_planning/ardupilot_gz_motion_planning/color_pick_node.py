"""
Python version of https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html
"""

import time
import numpy as np

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Vector3,
)
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
# import tf_transformations

from tf2_ros import TransformException
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive

from transforms3d import quaternions
from transforms3d import euler

# from moveit.planning_interface import MoveGroupInterface


def pose_stamped_to_move_group_goal(
    pose,
    eef_link,
    group_name,
    position_tolerance_m=0.02,
    orient_tolerance_rad=0.4,
    plan_only=False,
):
    req = MotionPlanRequest()
    req.group_name = group_name
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.5
    req.max_acceleration_scaling_factor = 0.5

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

    opts = PlanningOptions()
    opts.plan_only = plan_only
    opts.look_around = False
    opts.replan = False

    g = MoveGroup.Goal()
    g.request = req
    g.planning_options = opts
    return g


def joint_value_to_move_group_goal(
    joint_name, joint_value, group_name, plan_only=False
):
    req = MotionPlanRequest()
    req.group_name = group_name
    req.num_planning_attempts = 3
    req.allowed_planning_time = 3.0
    req.max_velocity_scaling_factor = 0.5
    req.max_acceleration_scaling_factor = 0.5

    jc = JointConstraint()
    jc.joint_name = joint_name
    jc.position = float(joint_value)
    jc.tolerance_above = 0.01
    jc.tolerance_below = 0.01
    jc.weight = 1.0

    c = Constraints()
    c.name = "joint_goal"
    c.joint_constraints.append(jc)
    req.goal_constraints.append(c)

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

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("optical_frame", "camera_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eef_link", "End_Effector")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("gripper_group", "gripper")
        self.declare_parameter("gripper_joint", "Gripper")
        self.declare_parameter("gripper_close_value", 0.0)
        # self.declare_parameter("enable_gripper_close", True)
        self.declare_parameter("enable_gripper_close", False)
        self.declare_parameter("table_z_in_base", 0.0)
        self.declare_parameter("approach_z_offset", 0.05)
        self.declare_parameter("hsv_lower", [40, 40, 40])
        self.declare_parameter("hsv_upper", [90, 255, 255])
        self.declare_parameter("min_area_px", 400)
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("single_shot", True)

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.optical_frame = (
            self.get_parameter("optical_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.eef_link = (
            self.get_parameter("eef_link").get_parameter_value().string_value
        )
        self.planning_group = (
            self.get_parameter("planning_group").get_parameter_value().string_value
        )
        self.gripper_group = (
            self.get_parameter("gripper_group").get_parameter_value().string_value
        )
        self.gripper_joint = (
            self.get_parameter("gripper_joint").get_parameter_value().string_value
        )
        self.gripper_close_value = (
            self.get_parameter("gripper_close_value").get_parameter_value().double_value
        )
        self.enable_gripper_close = (
            self.get_parameter("enable_gripper_close").get_parameter_value().bool_value
        )
        self.table_z = (
            self.get_parameter("table_z_in_base").get_parameter_value().double_value
        )
        self.approach_z = (
            self.get_parameter("approach_z_offset").get_parameter_value().double_value
        )
        lo = list(
            self.get_parameter("hsv_lower").get_parameter_value().integer_array_value
        )
        hi = list(
            self.get_parameter("hsv_upper").get_parameter_value().integer_array_value
        )
        self.hsv_lo = np.array(lo, dtype=np.uint8)
        self.hsv_hi = np.array(hi, dtype=np.uint8)
        self.min_area = (
            self.get_parameter("min_area_px").get_parameter_value().integer_value
        )
        action_name = (
            self.get_parameter("move_group_action").get_parameter_value().string_value
        )
        self.single_shot = (
            self.get_parameter("single_shot").get_parameter_value().bool_value
        )

        self._done = False
        self._busy = False
        self._bridge = CvBridge()
        self._K = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_dbg = self.create_publisher(PoseStamped, "debug/grasp_pose", 10)

        self._action_client = ActionClient(self, MoveGroup, action_name)

        # self.create_subscription(CameraInfo, self.camera_info_topic, self._on_ci, 1)
        # self.create_subscription(Image, self.image_topic, self._on_image, 1)

        # robot = MoveItPy(node_name="moveit_py")
        # robot_arm = robot.get_planning_component("arm")
        # planning_scene_monitor = robot.get_planning_scene_monitor()

        # target_pose = Pose()
        # target_pose.orientation.w = 1.0
        # target_pose.position.x = 0.3
        # target_pose.position.y = 0.3
        # target_pose.position.z = 0.2

        # move_group_interface = MoveGroupInterface(self, "arm")
        # move_group_interface.set_pose(target_pose)

        # // Create a plan to that target pose
        # [success, plan] = move_group_interface()
        # def make_plan():
        #     msg = MoveGroupInterface.Plan()
        #     ok = move_group_interface.plan(msg)
        #     return ok, msg
        # [success, plan] = make_plan()

        # // Execute the plan
        # if success:
        #     move_group_interface.execute(plan)
        # else:
        #   self.get_logger().info("Planning failed!")

        self.set_eef_pose()


    def set_eef_pose(self):
        x = 0.0
        y = -0.3888
        z = 0.2368

        qw = 0.70709
        qx = 0.0
        qy = 1.5707
        qz = 0.70709

        # q = euler.euler2quat(r, p, y)

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        # ps.pose.position.z = z + self.approach_z
        ps.pose.orientation.w = qw
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz

        self.pub_dbg.publish(ps)
        self.get_logger().info(f"Set pose: {ps}")

        goal = pose_stamped_to_move_group_goal(
            ps, self.eef_link, self.planning_group, plan_only=False
        )
        # self.get_logger().info(f"Set goal: {goal}")

        self._busy = True
        self._action_client.send_goal_async(goal).add_done_callback(
            self._on_arm_goal_sent
        )

    def _on_arm_goal_sent(self, future):
        self.get_logger().info("Goal sent")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._busy = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future):
        self.get_logger().info("Received arm result")
        res = future.result().result
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Failed to move arm")
            self._busy = False
            return
        if self.enable_gripper_close:
            self.get_logger().info("Closing gripper")
            g = joint_value_to_move_group_goal(
                self.gripper_joint,
                self.gripper_close_value,
                self.gripper_group,
                plan_only=False,
            )
            self._action_client.send_goal_async(g).add_done_callback(
                self._on_gripper_goal_sent
            )
        else:
            self.get_logger().info("Move arm done")
            self._done = True
            self._busy = False

    def _on_gripper_goal_sent(self, future):
        self.get_logger().info("Gripper goal sent")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Gripper goal not accepted")
            self._busy = False
            return
        goal_handle.get_result_async().add_done_callback(self._on_gripper_result)

    def _on_gripper_result(self, future):
        self.get_logger().info("Received gripper result")
        self._busy = False
        res = future.result().result
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            self._done = True
        else:
            self.get_logger().info("Failed to close gripper")


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
