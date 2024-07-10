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


"""
A high-speed GPS waypoint navigation task 
"""
class GPSWaypointDemo(Node):
    def __init__(self):
        super().__init__('gps_waypoint_demo')
        self.poweroff_count = 0
        self.declare_parameter('poweroff_joy_button', 8)  # PS button on PS4 controller
        self.declare_parameter('joy_topic', '/joy_teleop/joy')
        self.button_num = self.get_parameter('poweroff_joy_button').value
        joy_topic = self.get_parameter('joy_topic').value

        joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        print(f'Power off backpack node started. Checking for power off event on {joy_topic} topic on button {self.button_num}')


# Initialize system checks
# Set datum datum: [37.80046733333333, -122.45829418, 0.0]
# Navigate to replanned corners
# preempt / set tolerance to high (3-5m) for GPS
# loop N times
# interrupt / return to base option / battery low
# wait for user to say "start"

# Purpose to express for it: show GPS waypoint navigation, (all modes?), high speed navigation, metrics associated
# ... use GPS WPF or cartesian? 



def main():
    rclpy.init()
    node = GPSWaypointDemo()
    rclpy.spin(node)
    exit(0)


if __name__ == '__main__':
    main()
