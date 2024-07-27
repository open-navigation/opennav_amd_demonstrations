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

import copy
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

###################################################################################################
# For a deployed production application, use the Nav2 Route Server to generate the route from A->B
# on a navigation graph. A simplified version is used here for illustration purposes to be more
# clear and create a demonstration which uses Nav2's python API to create and manage the autonomy
# system in this script rather than using the behavior tree. Note: using the BT is a better choice.
###################################################################################################


# Helper function to calculate the distance between two points
def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# Route navigation graph node
class RouteNode():

    def __init__(self, name, position):
        self.name = name
        self.position = position
        self.neighbors = []
        self.previous_node = None
        self.shortest_path_from_start = 1.0e99
        self.is_visited = False

    def addNeighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def getNeighbors(self):
        return self.neighbors

    def getPosition(self):
        return self.position

    def getName(self):
        return self.name

    def getCost(self, neighbor):
        return dist(neighbor.position, self.position)

    def isVisited(self):
        return self.is_visited


# A *simple* route navigation planner for the urban navigation demo
class RoutePlanner():

    def __init__(self, nodes):
        self.graph = nodes

    def getRoute(self, end_id, start_id):
        # Find the start node in the graph
        source = None
        for node in self.graph:
            if node.getName() == start_id:
                source = node
                source.shortest_path_from_start = 0

        # Make an unvisited list
        current_node = source
        unvisited_nodes = copy.deepcopy(self.graph)

        # Perform Dikstra's algorithm to find the shortest path
        while len(unvisited_nodes) != 0:
            # Find next shortest path node in a reusable, OOP way (inefficient for large graphs)
            unvisited_nodes.sort(key=lambda x: x.shortest_path_from_start)
            candidate_node = None
            for node in unvisited_nodes:
                if not node.isVisited():
                    candidate_node = node
                    break

            # All nodes visited, find path to the goal
            if candidate_node is None:
                for node in unvisited_nodes:
                    if node.getName() == end_id:
                        if node.previous_node is None:
                            print('No path found to the goal node!')
                            return None
                        else:
                            shortest_path = []
                            current_node = node
                            while current_node is not None:
                                shortest_path.append(current_node)
                                current_node = current_node.previous_node
                            shortest_path.reverse()
                            return shortest_path

            # Search all neighbors
            for neighbor in candidate_node.getNeighbors():
                candidate_dist = \
                  candidate_node.shortest_path_from_start + candidate_node.getCost(neighbor)
                if candidate_dist < neighbor.shortest_path_from_start:
                    neighbor.shortest_path_from_start = candidate_dist
                    neighbor.previous_node = candidate_node
            candidate_node.is_visited = True
        return None

    def getClosestNode(self, position):
        closest_distance = 1.0e99
        closest_node = None
        for node in self.graph:
            distance = dist(node.position, position)
            if distance < closest_distance:
                closest_distance = distance
                closest_node = node
        return closest_node

    def interpolatePoints(self, p1, p2, resolution=0.05):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        distance = math.sqrt(dx**2 + dy**2)
        steps = int(math.ceil(distance / resolution))
        step_x = dx / steps
        step_y = dy / steps
        return [(p1[0] + step_x * i, p1[1] + step_y * i) for i in range(steps + 1)]

    def planToRouteStart(self, start_pose, start_node, stamp):
        # Plan a path from the current pose to the start node of the route
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = stamp
        interpolated_pts = self.interpolatePoints(
            [start_pose.pose.position.x, start_pose.pose.position.y], start_node.position)
        for pt in interpolated_pts:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            plan.poses.append(pose)
        return plan

    def routeToPlan(self, route, stamp):
        # Upsample the sparse route into a path for the controller to loosely follow
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = stamp
        for i, node in enumerate(route[:-1]):
            next_node = route[i + 1]
            interpolated_pts = self.interpolatePoints(node.position, next_node.position)
            for pt in interpolated_pts:
                pose = PoseStamped()
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                plan.poses.append(pose)

        # Add the final point, for good measure
        pose = PoseStamped()
        pose.pose.position.x = route[-1].position[0]
        pose.pose.position.y = route[-1].position[1]
        plan.poses.append(pose)
        return plan
