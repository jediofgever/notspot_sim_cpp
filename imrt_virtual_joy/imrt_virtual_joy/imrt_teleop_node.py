#!/usr/bin/env python

# Copyright 2022 NMBU Robotics

# Use of this source code is governed by an MIT
# license that can be found at
# https://opensource.org/licenses/MIT.
#
# Author: Lars Grimstad (lars.grimstad@nmbu.no)
# Author: Fetullah Atas (fetullah.atas@nmbu.no)


import rclpy
import rclpy.qos
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class IMRTTeleop(Node):

    def __init__(self):
        super().__init__('imrt_teleop_node')
        self.cmd_publisher = self.create_publisher(
            Twist, 'imrt_virtual_joy/cmd_vel', rclpy.qos.qos_profile_sensor_data)

        self.create_subscription(
            Joy, 'joy',  self.joy_callback, rclpy.qos.qos_profile_sensor_data)

        self._vx_gain = 0.7
        self._wz_gain = 0.9

    def joy_callback(self, joy_msg):
        vx = joy_msg.axes[0] * self._vx_gain
        wz = -joy_msg.axes[3] * self._wz_gain

        twist_msg = Twist()
        twist_msg.linear.x = vx
        twist_msg.angular.z = wz
        self.cmd_publisher.publish(twist_msg)


def main():
    rclpy.init(args=None)
    node = IMRTTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
