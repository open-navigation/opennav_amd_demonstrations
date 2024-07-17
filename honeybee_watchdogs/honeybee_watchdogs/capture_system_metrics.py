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
import psutil
import time
import subprocess
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


"""
A method to easily parse system metrics from a file for analytics
"""
def parseSystemMetrics(file):
    data_dicts = []
    with open(file, 'r') as file:
        for line in file:
            data = line.split(', ')
            dict = {}
            for pair in data:
                key, value = pair.split(': ')
                try:
                    value = float(value)
                    if value.is_integer():
                        value = int(value)
                except ValueError:
                    pass
                dict[key] = value
            data_dicts.append(dict)
    return data_dicts


"""
A script to capture system metrics for later analysis
"""
class CaptureSystemMetrics(Node):
    def __init__(self):
        super().__init__('capture_system_metrics')
        self.declare_parameter('filepath', '/home/administrator/experiment_files')
        self.filepath = self.get_parameter('filepath').value
        if not os.path.exists(self.filepath):
            os.makedirs(self.filepath)
        self.filename = self.filepath + f'/system_metrics_{int(time.time())}.txt'

        self.speed = 0
        self.distance = 0
        self.last_pose = None
        self.battery_level = -1

        self.timer = self.create_timer(1.0, self.callback)
        self.odom_sub = self.create_subscription(
            Odometry, '/platform/odom/filtered', self.odom_callback, 10)
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.batt_sub = self.create_subscription(
            BatteryState, '/platform/bms/state', self.battery_callback, qos_profile)

    def odom_callback(self, msg):
        self.speed = msg.twist.twist.linear.x

        if self.last_pose is None:
            self.last_pose = msg.pose.pose.position
            return
        else:
            pose_msg = msg.pose.pose.position
            self.distance += math.hypot(pose_msg.x - self.last_pose.x, pose_msg.y - self.last_pose.y)
            self.last_pose = msg.pose.pose.position
    
    def battery_callback(self, msg):
        self.battery_level = msg.percentage * 100

    def get_wifi_signal_strength(self):
        try:
            # Run the iwconfig command to get WiFi information
            result = subprocess.run(['iwconfig'], capture_output=True, text=True, check=True)
            output = result.stdout

            # Parse the output to find the signal strength
            for line in output.split('\n'):
                if 'Signal level' in line:
                    # Extract the signal strength value
                    signal_level = line.split('Signal level=')[1].split(' ')[0]
                    return signal_level

            return None
        except subprocess.CalledProcessError as e:
            return None

    def callback(self):
        # Get System Info
        curr_time = int(time.time())
        cpu_percent = psutil.cpu_percent(interval=0.1)
        memory_percent = psutil.virtual_memory().percent
        swap_memory_percent = psutil.swap_memory().percent
        disk_percent = psutil.disk_usage('/').percent
        network_issues = psutil.net_io_counters().errin + psutil.net_io_counters().errout
        signal = (int(self.get_wifi_signal_strength()) + 110) * 10 / 7

        # Get ROS Info
        topics_and_types = self.get_topic_names_and_types()
        topics = ''
        for topic in topics_and_types:
            topics += topic[0] + ' '
        topics = topics[:-1]

        # Get Robot Info: sensor rates
        data = f'Time: {curr_time}, CPU: {cpu_percent}, Memory: {memory_percent}, Swap: {swap_memory_percent}, Disk: {disk_percent}, Signal: {signal}, NetError: {network_issues}, Speed: {self.speed}, Distance: {self.distance}, Battery: {self.battery_level}, Topics: {topics} \n'
        # print(data)
        print(f'Logging system metrics to file: {self.filename}')
        with open(self.filename, 'a+') as f:
            f.write(data)


def main():
    rclpy.init()
    node = CaptureSystemMetrics()
    rclpy.spin(node)
    exit(0)


if __name__ == '__main__':
    main()
