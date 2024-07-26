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
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty


"""
A script to use rosbag to record data for data set creation and analysis of experiments
"""


class RecordRosbagData(Node):

    def __init__(self):
        super().__init__('record_rosbag_data')
        self.declare_parameter('filepath', '~/experiment_files')
        self.filepath = os.path.expanduser(self.get_parameter('filepath').value)
        if not os.path.exists(self.filepath):
            os.makedirs(self.filepath)
        self.filename = self.filepath + f'/rosbag_{int(time.time())}'
        self.process = None

        self.exec = [
            'ros2', 'bag', 'record',
            '/tf', '/tf_static', '/robot_description', '/platform/odom', '/platform/odom/filtered',
            '/platform/joint_states', '/platform/cmd_vel_unstamped', '/platform/bms/state',
            '/joy_teleop/joy', '/cmd_vel', '/joy_teleop/cmd_vel', '/joy_teleop/emergency_stop',
            '/sensors/lidar_0/scan', '/sensors/lidar_0/points', '/sensors/imu_1/data',
            '/sensors/gps_0/fix', '/sensors/camera_0/points', '/sensors/camera_0/color/image',
            '/platform/dynamic_joint_states',
            '--max-bag-size', '3000000000',
            '-s', 'mcap',
            '-o', self.filename]

        # Get demo buttons (square=3, X=0 on PS4)
        self.declare_parameter('start_button', 2)
        self.declare_parameter('exit_button', 0)
        self.start_button = self.get_parameter('start_button').value
        self.exit_button = self.get_parameter('exit_button').value

        self.start_srv = self.create_service(Empty, '~/start_recording', self.startService)
        self.stop_srv = self.create_service(Empty, '~/stop_recording', self.stopService)
        self.joy_sub = self.create_subscription(
            Joy, 'joy_teleop/joy', self.joyCallback, 10)
        print('record_rosbag_data is up and ready to record data.')

        self.declare_parameter('start_on_bringup', False)
        self.start_on_bringup = self.get_parameter('start_on_bringup').value
        if self.start_on_bringup:
            self.startService(None, None)

    def startService(self, request, response):
        if self.process is not None:
            print('Data recording is already running.')
            return response
        print(f'Starting to record rosbag data with command: {self.exec}...')
        self.process = subprocess.Popen(self.exec)
        return response

    def stopService(self, request, response):
        if self.process is None:
            print('No data recording to stop.')
            return response

        self.process.terminate()
        try:
            self.process.wait(timeout=60)
        except subprocess.TimeoutExpired:
            print('kill required')
            self.process.kill()
        self.process = None
        print('Stopped recording rosbag data.')
        return response

    def joyCallback(self, msg):
        # Use same start/stop as with demo for recording passively
        if msg.buttons[self.exit_button] == 1:
            self.stopService(None, None)
        if msg.buttons[self.start_button] == 1:
            self.startService(None, None)


def main():
    rclpy.init()
    node = RecordRosbagData()
    rclpy.spin(node)
    exit(0)


if __name__ == '__main__':
    main()
