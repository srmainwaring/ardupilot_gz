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

"""Combine pose and twist subscriptions and publish odometry."""

import rclpy
import rclpy.node
import threading

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


class OdometryPublisher(rclpy.node.Node):
    """Combine pose and twist subscriptions and publish odometry."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("odometry_publisher")
        self.msg_event_object = threading.Event()

        # Declare and acquire `pose_topic` parameter.
        self.declare_parameter("pose_topic", "pose")
        self.pose_topic = (
            self.get_parameter("pose_topic").get_parameter_value().string_value
        )

        # Declare and acquire `twist_topic` parameter.
        self.declare_parameter("twist_topic", "twist")
        self.twist_topic = (
            self.get_parameter("twist_topic").get_parameter_value().string_value
        )

    def start_subscribers(self):
        """Start the subscribers."""
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.on_pose, 1
        )

        self.twist_sub = self.create_subscription(
            TwistStamped, self.twist_topic, self.on_twist, 1
        )

        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def on_pose(self, msg):
        """Process a PoseStamped message."""
        self.msg_event_object.set()
        self.get_logger().info("{}".format(msg))

    def on_twist(self, msg):
        """Process a TwistStamped message."""
        self.msg_event_object.set()
        self.get_logger().info("{}".format(msg))
