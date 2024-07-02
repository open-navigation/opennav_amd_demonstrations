#! /usr/bin/env python3
# Copyright 2024 Open Naviation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import time


"""
A watchdog to check for the soft E-Stop from the joystick
"""
class JoyEStop(Node):
    def __init__(self):
        super().__init__('joystick_estop')
        self.declare_parameter('poweroff_joy_button', 1)  # Red circle on PS4 controller
        self.declare_parameter('joy_topic', '/joy_teleop/joy')
        self.button_num = self.get_parameter('poweroff_joy_button').value
        joy_topic = self.get_parameter('joy_topic').value

        joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        soft_estop_pub = self.create_publisher(Bool, 'joy_teleop/emergency_stop', 10)

        self.estop_hit = False
        print(f'Soft joystick e-stop node started. Checking for e-stop events on {joy_topic} topic on button {self.button_num}')

    def joy_callback(self, msg):
        # Infinitely block if the button is pressed
        if msg.buttons[self.button_num] == 1:
            print('E-Stop request detected, stopping robot!')
            bool_msg = Bool()
            bool_msg.data = True
            soft_estop_pub.publish(bool_msg)
            self.estop_hit = True

         if self.estop_hit:
            bool_msg = Bool()
            bool_msg.data = True
            soft_estop_pub.publish(bool_msg)


def main():
    rclpy.init()
    node = JoyEStop()
    rclpy.spin(node)
    exit(0)


if __name__ == '__main__':
    main()
