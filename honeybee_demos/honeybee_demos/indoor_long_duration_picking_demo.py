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

from threading import Lock, Thread
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
from opennav_docking_msgs.action import DockRobot, UndockRobot  # nav_msgs in Iron and newer
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Joy

from .pick_dispatcher import Dispatcher

###################################################################################################
# For this demo, we will hardcode in this script the picking locations and placing bin locations
# to simulate a pick and place task in a warehouse environment. In a real application, a cloud
# dispatcher would have this information and send it to the robot to execute as part of a job
# description. Our robot will take jobs from a dispatcher and navigate to the picking locations,
# wait for a human to pick an item (simulated), and then navigate to the place bin & simulate
# placing the item in a bin or picked by an arm. The robot will dock to charge at the end of the
# mission and wait for the next. Annotated inline in the script are ways to have it 1 automatically
# continue executing missions until the mission queue is empty or the battery is low and 2 trigger
# tasks based on sufficient charge state and 3 cloud dispatch mission triggering.

# This can be reproduced by annotating a map with the appropriate pick and place locations!

# 1. Go to the location and joystick robot, or navigate autonomously to generate the 2D map
# 2. Update the key points of interest: Picking locations, place bins, charging dock, etc.
# 3. Update keepout or speed restricted zones, if necessary
# 4. Run, starting at dock
###################################################################################################

# Define the positions of picking locations in map and place bins to drop off goods at
shelf_a = {'slot_a': [-3.46, 15.92, -1.922], 'slot_b': [-2.36, 15.42, -1.922],
           'slot_c': [-1.26, 14.96, -1.922], 'slot_d': [-0.16, 14.78, -1.922],
           'slot_e': [2.00, 13.77, -1.922]}  # x, y, yaw in map
shelf_b = {'slot_a': [-4.63, 12.73, -1.922], 'slot_b': [-3.47, 11.88, -1.922],
           'slot_c': [-2.25, 11.36, -1.922], 'slot_d': [-1.01, 10.88, -1.922],
           'slot_e': [0.94, 10.46, -1.922]}
shelf_c = {'slot_a': [-12.30, 2.47, -1.922], 'slot_b': [-10.77, 1.95, -1.922],
           'slot_c': [-9.17, 1.34, -1.922]}
shelf_d = {'slot_a': [-8.52, 5.77, -1.922], 'slot_b': [-9.11, 4.23, -1.922]}
picking_locations = {'shelf_a': shelf_a,
                     'shelf_b': shelf_b,
                     'shelf_c': shelf_c,
                     'shelf_d': shelf_d}

place_bin_a = [5.79, 3.83, -0.367]  # x, y, yaw in map
place_bin_b = [5.92, 1.09, -0.367]
place_bin_c = [5.04, 7.51, -0.367]
goods_bins = {'bin_a': place_bin_a,
              'bin_b': place_bin_b,
              'bin_c': place_bin_c}

dock_pose = [-10.24, 0.87, -1.922]

"""
A Long-Duration, Indoor Picking demo navigating to pick and place bins & charging between missions
"""


class IndoorPickingDemo(Node):

    def __init__(self):
        super().__init__('indoor_picking_demo')
        self.demo_thread = None
        self.lock = Lock()
        self.stop = False

        self.navigator = BasicNavigator()
        self.dispatcher = Dispatcher(picking_locations, goods_bins)

        self.waitUntilActive()
        self.getParameters()

        self.docking_client = ActionClient(self.navigator, DockRobot, 'dock_robot')
        self.undocking_client = ActionClient(self.navigator, UndockRobot, 'undock_robot')

        self.batt_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.joy_sub = self.create_subscription(
            Joy, 'joy_teleop/joy', self.joyCallback, 10)
        self.batt_sub = self.create_subscription(
            BatteryState, 'platform/bms/state', self.batteryCallback, self.batt_qos)

        # Could be used to run mission every N minutes, for example once an hour
        # self.timer = self.create_timer(3600.0, self.runDemo)
        print('Long-duration picking demo node started.')

    def waitUntilActive(self):
        """Block until the components of the navigation system are up and running."""
        self.navigator._waitForNodeToActivate('bt_navigator')
        print('Nav2 is ready for use!')

    def getParameters(self):
        # Get demo buttons (square=3, X=0 on PS4)
        self.declare_parameter('start_button', 2)
        self.declare_parameter('exit_button', 0)
        self.start_button = self.get_parameter('start_button').value
        self.exit_button = self.get_parameter('exit_button').value

        # Get minimum battery to exit demo and return to base
        self.declare_parameter('min_battery_lvl', 0.20)
        self.min_battery_lvl = self.get_parameter('min_battery_lvl').value

        # Get battery level to restart taking missions once recharged automatically
        self.declare_parameter('battery_mission_restart_minimum', 0.80)
        self.battery_mission_restart_minimum = \
            self.get_parameter('battery_mission_restart_minimum').value

    def dockRobot(self, dock_id, nav_to_dock=True):
        while not self.docking_client.wait_for_server(timeout_sec=1.0):
            print('"DockRobot" action server not available, waiting...')
        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = dock_id
        goal_msg.navigate_to_staging_pose = nav_to_dock
        print('Docking at dock ID: ' + str(dock_id) + '...')
        send_goal_future = self.docking_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.navigator, send_goal_future)
        goal_handle = send_goal_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.navigator, result_future)
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            print(f'Docking with failed with status code: {status}')
        else:
            print('Docking successful!')

    def undockRobot(self, dock_type=''):
        while not self.undocking_client.wait_for_server(timeout_sec=1.0):
            print('"UndockRobot" action server not available, waiting...')
        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = dock_type
        print('Undocking from dock of type: ' + str(dock_type) + '...')
        send_goal_future = self.undocking_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.navigator, send_goal_future)
        goal_handle = send_goal_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.navigator, result_future)
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            print(f'Undocking with failed with status code: {status}')
        else:
            print('Undocking successful!')

    def batteryCallback(self, msg):
        if msg.percentage < self.min_battery_lvl:
            with self.lock:
                self.stop = True

        # Could be used to continue mission once battery levels are sufficiently high
        # if msg.percentage > self.battery_mission_restart_minimum:
        #     self.demo_thread = Thread(target=self.runDemo)
        #     self.demo_thread.daemon = True
        #     self.demo_thread.start()

    def joyCallback(self, msg):
        if msg.buttons[self.exit_button] == 1:
            print('Stop request detected, stopping pick and place at the end of this mission!')
            with self.lock:
                self.stop = True

        if msg.buttons[self.start_button] == 1 and self.demo_thread is None:
            print('Start request detected, starting a pick and place mission!')
            # Could be triggered by external cloud system to dispatch a new mission
            # A joystick for our demonstration purposes
            self.demo_thread = Thread(target=self.runDemo)
            self.demo_thread.daemon = True
            self.demo_thread.start()

    def wpToPose(self, wp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        quaternion = self._quaternion_from_euler(0.0, 0.0, wp[2])
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def getQuad(self, yaw):
        q_list = self._quaternion_from_euler(0.0, 0.0, yaw)
        q = Quaternion()
        q.z = q_list[2]
        q.w = q_list[3]
        return q

    def _quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
            np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
            np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def runDemo(self):
        stop = False
        nav_start = self.navigator.get_clock().now()
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = nav_start.to_msg()
        init_pose.pose.position.x = dock_pose[0]
        init_pose.pose.position.y = dock_pose[1]
        init_pose.pose.orientation = self.getQuad(dock_pose[2])
        self.navigator.setInitialPose(init_pose)
        time.sleep(2)
        self.navigator.clearAllCostmaps()
        self.undockRobot(dock_type='charging_dock')

        # Get next mission from dispatcher: N picks and places
        pick_poses = self.dispatcher.get_next_picks(num_picks=3)
        place_poses = self.dispatcher.get_next_drops(num_drops=3)

        while rclpy.ok() and len(pick_poses) > 0 and len(place_poses) > 0 and stop is False:
            pick_pose = pick_poses.pop(0)
            place_pose = place_poses.pop(0)
            print(f'Picking item from {pick_pose} and dropping at {place_pose}...')

            # Go to picking location
            self.navigator.goToPose(self.wpToPose(pick_pose))
            while not self.navigator.isTaskComplete() or not rclpy.ok():
                if self.navigator.get_clock().now() - nav_start > Duration(seconds=600.0):
                    self.navigator.cancelTask()

            # Wait at picking location for user to place item (simulated with 3s stop)
            time.sleep(3)

            # Go to place bin
            self.navigator.goToPose(self.wpToPose(place_pose))
            while not self.navigator.isTaskComplete() or not rclpy.ok():
                if self.navigator.get_clock().now() - nav_start > Duration(seconds=600.0):
                    self.navigator.cancelTask()

            # Wait at place bin arm to take item off robot (simulated with 3s stop)
            time.sleep(3)

            with self.lock:
                stop = self.stop

        # Completed task, dock robot to charge, but:
        # An application could check here if there are new tass to do rather than exiting
        # as long as battery healthy was sufficient to continue
        # if self.battery_level < self.battery_mission_minimum:
        #     self.dockRobot(dock_id='charging_dock', nav_to_dock=True)
        # else:
        #     pick_poses = self.dispatcher.get_next_picks(num_picks=3)
        #     place_poses = self.dispatcher.get_next_drops(num_drops=3)
        #     continue
        self.dockRobot(dock_id='home_dock', nav_to_dock=True)
        print('Pick and place mission completed and robot is docked to charge.'
              ' Waiting for next mission...')
        self.stop = False
        self.demo_thread = None


def main():
    rclpy.init()
    node = IndoorPickingDemo()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    exit(0)


if __name__ == '__main__':
    main()
