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

import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

"""
A watchdog to check if PS controller indicates a power off event is coming.
"""


class PowerOffBackPack(Node):

    def __init__(self):
        super().__init__('power_off_backpack')
        self.poweroff_count = 0
        self.declare_parameter('poweroff_joy_button', 8)  # PS button on PS4 controller
        self.declare_parameter('joy_topic', '/joy_teleop/joy')
        self.button_num = self.get_parameter('poweroff_joy_button').value
        joy_topic = self.get_parameter('joy_topic').value

        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        print(f'Power off backpack node started. '
              f'Checking for power off event on {joy_topic} topic on button {self.button_num}')

    def joy_callback(self, msg):
        if msg.buttons[self.button_num] == 1:
            self.poweroff_count += 1
            if self.poweroff_count >= 20:  # Hold down for 2-3s
                print('Power off event detected. Shutting down backpack.')
                os.system('poweroff')
        else:
            self.poweroff_count = 0


def main():
    rclpy.init()
    node = PowerOffBackPack()
    rclpy.spin(node)
    exit(0)


if __name__ == '__main__':
    main()
