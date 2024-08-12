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


import math
import os
import subprocess
import time

from nav_msgs.msg import Odometry
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState

"""
A method to easily parse system metrics & analze files for analytics
"""


def parseSystemMetrics(file):
    data_dicts = []
    with open(file, 'r') as file:
        for line in file:
            data = line.split(', ')
            adict = {}
            for pair in data:
                key, value = pair.split(': ')
                try:
                    value = float(value)
                    if value.is_integer():
                        value = int(value)
                except ValueError:
                    pass
                adict[key] = value
            data_dicts.append(adict)
    return data_dicts


def analyze_system_metrics():
    # Obtain all the raw data from files
    files_in_directory = os.listdir('.')  # os.path.expanduser('~/experiment_files')
    metrics_files = [file for file in files_in_directory if file.startswith('system_metrics')]
    raw_data = []
    for file in metrics_files:
        data_dicts = parseSystemMetrics(file)
        raw_data.append(data_dicts)

    # Extract data for each field to analyze. For this script, resource utilization.
    cpu_use = []
    memory_use = []
    disk_use = []
    for data in raw_data:
        for entry in data:
            cpu_use.append(entry['CPU'])
            memory_use.append(entry['Memory'])
            disk_use.append(entry['Disk'])

    # Calculate the average of each field
    print(f'Dataset duration: {(len(cpu_use) / 60.0):.2f} min')  # 1 hz
    print(f'Ave CPU usage: {sum(cpu_use) / len(cpu_use):.2f}%')
    print(f'Ave Disk usage: {sum(disk_use) / len(disk_use):.2f}%')
    print(f'Ave Memory usage: {sum(memory_use) / len(memory_use):.2f}%')

    # Find and print the highest value of each field
    print(f'Highest CPU usage: {max(cpu_use):.2f}%')
    print(f'Highest Disk usage: {max(disk_use):.2f}%')
    print(f'Highest Memory usage: {max(memory_use):.2f}%')


"""
A script to capture system metrics for later analysis
"""


class CaptureSystemMetrics(Node):

    def __init__(self):
        super().__init__('capture_system_metrics')
        self.declare_parameter('filepath', '~/experiment_files')
        self.filepath = os.path.expanduser(self.get_parameter('filepath').value)
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
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.batt_sub = self.create_subscription(
            BatteryState, '/platform/bms/state', self.battery_callback, qos_profile)

    def odom_callback(self, msg):
        self.speed = msg.twist.twist.linear.x

        if self.last_pose is None:
            self.last_pose = msg.pose.pose.position
            return
        else:
            pose_msg = msg.pose.pose.position
            self.distance += math.hypot(pose_msg.x - self.last_pose.x,
                                        pose_msg.y - self.last_pose.y)
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
        except subprocess.CalledProcessError:
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
        data = f'Time: {curr_time}, CPU: {cpu_percent}, Memory: {memory_percent}, ' \
               f'Swap: {swap_memory_percent}, Disk: {disk_percent}, Signal: {signal}, ' \
               f'NetError: {network_issues}, Speed: {self.speed}, Distance: {self.distance}, ' \
               f'Battery: {self.battery_level}, Topics: {topics} \n'
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
