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
import copy
from threading import Thread, Lock
import numpy as np
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy, BatteryState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy

###################################################################################################
# For a deployed production application, use the Nav2 Route Server to generate the route from A->B
# on a navigation graph. A simplified version is used here for illustration purposes to be more
# clear and create a demonstration which uses Nav2's python API to create and manage the autonomy
# system in this script rather than using the behavior tree. Note: using the BT is a better choice.

# Thus, for this demo, we will hardcode in this script the navigation route graph defined by our
# 3D map of the urban environment. The area of the experiment is bounded by the streets of: Midway,
# Lexington, Saratoga, and Tower on Alameda near City Hall, Humble Sea Brewing, and a Fire Station:
# https://www.google.com/maps/@37.7845673,-122.3033875,485m/data=!3m1!1e3?entry=ttu

# This can also be reproduced by another robot on Alameda without modification!

# 1. Go to the location and joystick robot, or navigate autonomously to generate the 3D map
# 2. Find the key points of interest: the street intersections and locations for navigation targets
# 3. Update the script's route graph below to match the 3D map and key points of interest
# Pro Tip: Try to translate/rotate the map's origin to align with already defined waypoints below!
###################################################################################################

# Define the positions of each of the interactions and points of interest for planning
midway_and_lexington = [0.0, 0.0, 0.0]
midway_and_saratoga = [100.0, 0.0, 0.0]
ranger_and_lexington = [200.0, 0.0, 0.0]
ranger_and_saratoga = [300.0, 0.0, 0.0]
tower_and_lexington = [400.0, 0.0, 0.0]
tower_and_saratoga = [500.0, 0.0, 0.0]
alameda_city_hall = [600.0, 0.0, 0.0]
humble_sea_brewing = [700.0, 0.0, 0.0]
alameda_fire_station = [800.0, 0.0, 0.0]

# Convert RPY into Quaternion
def rpyToQuad(self, roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

# Convert the annotated waypoints to PoseStamped
def wpsToPoses(self, wps):
    poses = []
    for wp in wps:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = wp[0]
        pose.pose.position.y = wp[1]
        quaternion = rpyToQuad(0.0, 0.0, wp[2])
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        poses.append(pose)
    return poses


# Route graph node for the urban navigation demo
class RouteNode():
    def __init__(self, name, position):
        self.name = name
        self.position = position
        self.neighbors = []
    
    def addNeighbor(self, neighbor):
        self.neighbors.append(neighbor)
    
    def getNeighbors(self):
        return self.neighbors
    
    def getPosition(self):
        return self.position
    
    def getName(self):
        return self.name


# Route planner for the urban navigation demo
class RoutePlanner():
    def __init__():
        nodes = self.populateGraph()
    
    def populateGraph(self):
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
  
    def getRoute(self, end, start = None):
        queue = [] # TODO data structures
        goal_node = end

        # Get the starting node
        if start is None:
            closest_node = self.getClosestNode(self.navigator.getRobotPose())
            queue.push(closest_node)
        else:
            for node in self.nodes:
                if node.getName() == start:
                    queue.push(node)

        # TODO Perform a breadth-first search to find the route
        # while len(queue) > 0:
        #     current_node = queue.pop(0)
        #     if current_node.getName() == goal_node:
        #         break
        #     for neighbor in current_node.getNeighbors():
        #         queue.append(neighbor)


    def getClosestNode(self, position):
        closest_distance = 1e20
        closest_node = None
        for node in self.nodes:
            distance = math.sqrt((node.position[0] - position[0]) ** 2 + (node.position[1] - position[1]) ** 2)
            if distance < closest_distance:
                closest_distance = distance
                closest_node = node
        return closest_node

    def interpolate_points(p1, p2, resolution = 0.05):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        distance = math.sqrt(dx**2 + dy**2)
        steps = int(math.ceil(distance / resolution))
        step_x = dx / steps
        step_y = dy / steps        
        return [(p1[0] + step_x * i, p1[1] + step_y * i) for i in range(steps + 1)]

    def planToRouteStart(self, start_pose, start_node):
        # Plan a path from the current pose to the start node of the route
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = self.navigator.getNode().now().to_msg()
        start_pose_list = [start_pose.pose.position.x, start_pose.pose.position.y]
        interpolated_pts = self.interpolate_points(start_pose_list, start_node.position)
        for pt in interpolated_pts:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            plan.poses.append(pose)

    def routeToPlan(self, route):
        # Upsample the sparse route into a path for the controller to loosely follow
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = self.navigator.getNode().now().to_msg()
        for i, node in enumerate(route[:-1]):
            next_node = route[i + 1]
            interpolated_pts = self.interpolate_points(node.position, next_node.position)
            for pt in interpolated_pts:
                pose = PoseStamped()
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                plan.poses.append(pose)

        # Add the final point, for good measure
        pose = PoseStamped()
        pose.pose.position.x = route[-1].position[0]
        pose.pose.position.y = route[-1].position[1]
        plan.poses.append()
        return plan


"""
An Urban Navigation demo navigating between buildings on a city block
"""
class UrbanNavigationDemo(Node):
    def __init__(self):
        super().__init__('urban_navigation_demo')
        self.demo_thread = None
        self.lock = Lock()
        self.stop = False

        self.navigator = BasicNavigator()
        self.route_planner = RoutePlanner()

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

        print('Urban Navigation Demo node started.')

    def waitUntilActive(self):
        """Block until the full navigation system is up and running."""
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
        while rclpy.ok():
            nav_start = self.navigator.get_clock().now()

            # Select a random, non-correct goal to navigate to in the graph
            curr_pose = self.navigator.getRobotPose()
            goal = random.choice(self.route_planner.nodes)
            while math.sqrt((goal.position[0] - curr_pose.pose.position.x[0]) ** 2 + (goal.position[1] - curr_pose.pose.position.x[1]) ** 2) < 10.0:
                goal = random.choice(self.route_planner.nodes)

            # Compute a route to the goal on the graph, then find its dense path
            route = self.route_planner.getRoute(goal.getName())
            route_plan = self.route_planner.routeToPlan(route)
            if math.sqrt((route[0].position[0] - curr_pose.pose.position.x[0]) ** 2 + (route[0].position[1] - curr_pose.pose.position.x[1]) ** 2) > 1.0:
                init_plan = self.route_planner.planToRouteStart(curr_pose, route[0])
                route_plan.poses = init_plan.poses + route_plan.poses

            # Finally, follow it using Nav2's controller
            self.navigator.followPath(route_plan)

            # Track ongoing progress
            i = 0
            while not self.navigator.isTaskComplete() or not rclpy.ok():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 10 == 0:
                    print('Executing current waypoint: ' + str(feedback.current_waypoint))

                    # Some navigation timeout to demo cancellation
                    if self.navigator.get_clock().now() - nav_start > Duration(seconds=1200.0):
                        self.navigator.cancelTask()

            # Log a result of the loop
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'Urban navigation demo succeeded going to {goal.getName()}!')
            elif result == TaskResult.CANCELED:
                print(f'Urban navigation demo was canceled going to {goal.getName()}!')
            elif result == TaskResult.FAILED:
                print(f'Urban navigation demo failed going to {goal.getName()}!')
            else:
                print('Urban navigation demo has an invalid return status!')

            # Check if a stop is requested or no looping is necessary
            with self.lock:
                if self.stop
                    print('Exiting Urban navigation demo. Stop was requested.')
                    return


def main():
    rclpy.init()
    node = UrbanNavigationDemo()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    exit(0)


if __name__ == '__main__':
    main()
