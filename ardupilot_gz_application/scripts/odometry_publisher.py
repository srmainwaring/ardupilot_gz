#!/usr/bin/env python3

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

from ardupilot_gz_application.odometry_publisher import OdometryPublisher


def main(args=None):
    """Entrypoint for the odometry_publisher node."""
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
