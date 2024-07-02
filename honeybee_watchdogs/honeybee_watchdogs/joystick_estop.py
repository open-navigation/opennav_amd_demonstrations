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
        self.declare_parameter('estop_joy_button', 1)  # Red circle on PS4 controller
        self.declare_parameter('enable_joy_button', 3)  # Red circle on PS4 controller
        self.declare_parameter('joy_topic', '/joy_teleop/joy')
        self.estop_button_num = self.get_parameter('estop_joy_button').value
        self.enable_button_num = self.get_parameter('enable_joy_button').value
        joy_topic = self.get_parameter('joy_topic').value

        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.soft_estop_pub = self.create_publisher(Bool, 'joy_teleop/emergency_stop', 10)

        self.estop_hit = False
        self.reactivate_count = 0
        bool_msg = Bool()
        bool_msg.data = False
        self.soft_estop_pub.publish(bool_msg)
        print(f'Soft joystick e-stop node started. Checking for e-stop events on {joy_topic} topic on button {self.estop_button_num}')

    def joy_callback(self, msg):
        # Infinitely block if the button is pressed
        if msg.buttons[self.estop_button_num] == 1:
            print('E-Stop request detected, stopping robot until reset!')
            bool_msg = Bool()
            bool_msg.data = True
            self.soft_estop_pub.publish(bool_msg)
            self.estop_hit = True

        if self.estop_hit:
            bool_msg = Bool()
            bool_msg.data = True
            self.soft_estop_pub.publish(bool_msg)
        
        if msg.buttons[self.enable_button_num] == 1:
            self.reactivate_count += 1
            if self.reactivate_count >= 100:  # hold down for 5s
                print('E-Stop reset detected, allowing robot to move again!')
                self.estop_hit = False
                bool_msg = Bool()
                bool_msg.data = False
                self.soft_estop_pub.publish(bool_msg)
                self.reactivate_count = 0

def main():
    rclpy.init()
    node = JoyEStop()
    rclpy.spin(node)
    exit(0)


if __name__ == '__main__':
    main()
