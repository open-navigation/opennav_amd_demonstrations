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

import random
from threading import Lock, Thread
import time

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Joy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .simple_route_planner import dist, RouteNode, RoutePlanner

###################################################################################################
# For this demo, we will hardcode in this script the navigation route graph defined by our
# 3D map of the urban environment. The area of the experiment is bounded by the streets of: Midway,
# Lexington, Saratoga, and Tower on Alameda near City Hall, Humble Sea Brewing, and a Fire Station:
# https://www.google.com/maps/@37.7845673,-122.3033875,485m/data=!3m1!1e3?entry=ttu

# This can also be reproduced by another robot on Alameda without modification!

# 1. Go to the location and joystick robot, or navigate autonomously to generate the 3D map
# 2. Find the key points of interest: the street intersections and locations for navigation targets
# 3. Update the script's route graph below to match the 3D map and key points of interest
# Pro Tip: Try to translate/rotate the map's origin to align with already defined waypoints below!
#          We set the origin to be Tower & Lexington
###################################################################################################

# Define the positions of each of the interactions and points of interest for planning
midway_and_saratoga = [295.7, -128.1, 0.0]
midway_and_lexington = [295.7, 0.0, 0.0]
ranger_and_saratoga = [181.4, -128.1, 0.0]
ranger_and_lexington = [181.4, 0.0, 0.0]
tower_and_saratoga = [0.0, -128.1, 0.0]
tower_and_lexington = [0.0, 0.0, 0.0]
alameda_fire_station = [181.4, -94.6, 0.0]
humble_sea_brewing = [163.9, -128.1, 0.0]
alameda_city_hall = [295.7, -62.5, 0.0]


# Define the route graph
def populateGraph():
    nodes = []
    midway_and_lexington_node = RouteNode('midway_and_lexington', midway_and_lexington)
    midway_and_saratoga_node = RouteNode('midway_and_saratoga', midway_and_saratoga)
    ranger_and_lexington_node = RouteNode('ranger_and_lexington', ranger_and_lexington)
    ranger_and_saratoga_node = RouteNode('ranger_and_saratoga', ranger_and_saratoga)
    tower_and_lexington_node = RouteNode('tower_and_lexington', tower_and_lexington)
    tower_and_saratoga_node = RouteNode('tower_and_saratoga', tower_and_saratoga)
    alameda_city_hall_node = RouteNode('alameda_city_hall', alameda_city_hall)
    humble_sea_brewing_node = RouteNode('humble_sea_brewing', humble_sea_brewing)
    alameda_fire_station_node = RouteNode('alameda_fire_station', alameda_fire_station)

    midway_and_lexington_node.addNeighbor(ranger_and_lexington_node)
    midway_and_lexington_node.addNeighbor(midway_and_saratoga_node)
    midway_and_lexington_node.addNeighbor(alameda_city_hall_node)
    nodes.append(midway_and_lexington_node)

    midway_and_saratoga_node.addNeighbor(midway_and_lexington_node)
    midway_and_saratoga_node.addNeighbor(ranger_and_saratoga_node)
    midway_and_saratoga_node.addNeighbor(alameda_city_hall_node)
    nodes.append(midway_and_saratoga_node)

    ranger_and_lexington_node.addNeighbor(midway_and_lexington_node)
    ranger_and_lexington_node.addNeighbor(ranger_and_saratoga_node)
    ranger_and_lexington_node.addNeighbor(tower_and_lexington_node)
    ranger_and_lexington_node.addNeighbor(alameda_fire_station_node)
    nodes.append(ranger_and_lexington_node)

    ranger_and_saratoga_node.addNeighbor(midway_and_saratoga_node)
    ranger_and_saratoga_node.addNeighbor(ranger_and_lexington_node)
    ranger_and_saratoga_node.addNeighbor(tower_and_saratoga_node)
    ranger_and_saratoga_node.addNeighbor(alameda_fire_station_node)
    ranger_and_saratoga_node.addNeighbor(humble_sea_brewing_node)
    nodes.append(ranger_and_saratoga_node)

    tower_and_lexington_node.addNeighbor(ranger_and_lexington_node)
    tower_and_lexington_node.addNeighbor(tower_and_saratoga_node)
    nodes.append(tower_and_lexington_node)

    tower_and_saratoga_node.addNeighbor(ranger_and_saratoga_node)
    tower_and_saratoga_node.addNeighbor(tower_and_lexington_node)
    tower_and_saratoga_node.addNeighbor(humble_sea_brewing_node)
    nodes.append(tower_and_saratoga_node)

    alameda_city_hall_node.addNeighbor(midway_and_lexington_node)
    alameda_city_hall_node.addNeighbor(midway_and_saratoga_node)
    nodes.append(alameda_city_hall_node)

    humble_sea_brewing_node.addNeighbor(ranger_and_saratoga_node)
    humble_sea_brewing_node.addNeighbor(tower_and_saratoga_node)
    nodes.append(humble_sea_brewing_node)

    alameda_fire_station_node.addNeighbor(ranger_and_saratoga_node)
    alameda_fire_station_node.addNeighbor(ranger_and_lexington_node)
    nodes.append(alameda_fire_station_node)
    return nodes


"""
An Urban Navigation demo navigating between buildings on a city block
"""


class UrbanNavigationDemo(Node):

    def __init__(self):
        super().__init__('urban_navigation_demo')
        self.demo_thread = None
        self.lock = Lock()
        self.stop = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.route_planner = RoutePlanner(populateGraph())

        self.waitUntilActive()
        self.getParameters()

        self.batt_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.joy_sub = self.create_subscription(
            Joy, 'joy_teleop/joy', self.joyCallback, 10)
        self.batt_sub = self.create_subscription(
            BatteryState, 'platform/bms/state', self.batteryCallback, self.batt_qos)

        # Publish artifacts for visualization
        self.path_pub = self.create_publisher(Path, 'plan', 10)
        self.graph_pub = self.create_publisher(PoseArray, 'graph', 10)
        viz_graph = PoseArray()
        viz_graph.header.frame_id = 'map'
        viz_graph.header.stamp = self.get_clock().now().to_msg()
        for node in self.route_planner.graph:
            pose = Pose()
            pose.position.x = node.position[0]
            pose.position.y = node.position[1]
            viz_graph.poses.append(pose)
        self.graph_pub.publish(viz_graph)
        print('Urban Navigation Demo node started.')

    def waitUntilActive(self):
        """Block until the components of the navigation system are up and running."""
        self.navigator._waitForNodeToActivate('controller_server')
        self.navigator._waitForNodeToActivate('smoother_server')
        self.navigator._waitForNodeToActivate('velocity_smoother')
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

    def batteryCallback(self, msg):
        if msg.percentage < self.min_battery_lvl:
            with self.lock:
                self.stop = True

    def joyCallback(self, msg):
        if msg.buttons[self.exit_button] == 1:
            print('Stop request detected, stopping urban navigation demo at end of this loop!')
            with self.lock:
                self.stop = True

        if msg.buttons[self.start_button] == 1 and self.demo_thread is None:
            print('Start request detected, starting urban navigation demo!')
            self.demo_thread = Thread(target=self.runDemo)
            self.demo_thread.daemon = True
            self.demo_thread.start()

    def runDemo(self):
        start_node_id = None
        while rclpy.ok():
            # Pause before starting and pause at each location
            time.sleep(2)

            # Get starting pose and node for search
            nav_start = self.navigator.get_clock().now()
            start_pose = PoseStamped()
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                start_pose.header = t.header
                start_pose.pose.position.x = t.transform.translation.x
                start_pose.pose.position.y = t.transform.translation.y
                start_pose.pose.position.z = t.transform.translation.z
                start_pose.pose.orientation = t.transform.rotation
            except TransformException as ex:
                print(f'Could not transform map to base_link: {ex}')
                return

            if not start_node_id:
                start_node_id = self.route_planner.getClosestNode(
                    [start_pose.pose.position.x, start_pose.pose.position.y]).getName()

            # Select a random, non-current goal to navigate to in the graph
            start_pose_list = [start_pose.pose.position.x, start_pose.pose.position.y]
            goal = random.choice(self.route_planner.graph)
            while dist(goal.position, start_pose_list) < 10.0:
                goal = random.choice(self.route_planner.graph)

            print(f'Navigating from {start_node_id} to {goal.getName()}...')

            # Compute a route to the goal on the graph, then find its dense path
            route = self.route_planner.getRoute(goal.getName(), start_id=start_node_id)
            route_plan = self.route_planner.routeToPlan(route, nav_start.to_msg())

            # If distance from current pose is too far from start node, plan there first
            if dist(route[0].position, start_pose_list) > 3.0:
                init_plan = self.route_planner.planToRouteStart(
                    start_pose, route[0], nav_start.to_msg())
                route_plan.poses = init_plan.poses + route_plan.poses

            # Finally, smooth corners, publish visualization, and follow it with Nav2
            # Controller to handle deviations and obstacles required to complete the task
            for i in range(10):
                route_plan = self.navigator.smoothPath(route_plan)
            self.path_pub.publish(route_plan)
            self.navigator.followPath(route_plan)

            # Track ongoing progress
            while not self.navigator.isTaskComplete() or not rclpy.ok():
                if self.navigator.get_clock().now() - nav_start > Duration(seconds=1200.0):
                    self.navigator.cancelTask()

            # Log a result of the loop
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'Urban navigation demo succeeded going to {goal.getName()}!')
            elif result == TaskResult.CANCELED:
                print(f'Urban navigation demo was canceled going to {goal.getName()}!')
                return
            elif result == TaskResult.FAILED:
                print(f'Urban navigation demo failed going to {goal.getName()}!')
                return
            else:
                print('Urban navigation demo has an invalid return status!')
                return

            # Check if a stop is requested or no looping is necessary
            with self.lock:
                if self.stop:
                    print('Exiting Urban navigation demo. Stop was requested.')
                    return

            # Else, Start from the goal node for the next task
            start_node_id = goal.getName()


def main():
    rclpy.init()
    node = UrbanNavigationDemo()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    exit(0)


if __name__ == '__main__':
    main()
