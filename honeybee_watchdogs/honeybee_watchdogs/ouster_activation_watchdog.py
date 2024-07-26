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

import time


from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from ros2node.api import get_node_names


"""
A watchdog to check if the ouster is properly activated or not. If not, activate itself.
"""


def main():
    rclpy.init()
    node = rclpy.create_node('ouster_activation_watchdog')
    start_time = time.time()
    node_name = '/sensors/lidar_0/os_sensor'
    srv_name = node_name + '/change_state'
    srv_get_name = node_name + '/get_state'

    # Check if node exists
    node_found = False
    while rclpy.ok() and time.time() - start_time < 30.0:
        available_nodes = get_node_names(node=node, include_hidden_nodes=False)
        for av_node in available_nodes:
            if av_node.full_name == node_name:
                node_found = True
                print('Ouster node ' + node_name + ' was found')
        if node_found:
            break

    # Configure it
    start_time = time.time()
    configured = False
    while rclpy.ok() and time.time() - start_time < 160.0 and node_found:
        mgr_client = node.create_client(ChangeState, srv_name)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            print(f'{srv_name} service not available, waiting...')
        req = ChangeState.Request()
        req.transition.id = req.transition.TRANSITION_CONFIGURE
        future = mgr_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=35.0)
        if not future or not future.result().success:
            print('Failed to configure sensor, trying again...')
        else:
            configured = True
            print('Ouster node ' + node_name + ' was configured')
            break

    # Activate it
    start_time = time.time()
    activated = False
    while rclpy.ok() and time.time() - start_time < 40.0 and configured:
        mgr_client = node.create_client(ChangeState, srv_name)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            print(f'{srv_name} service not available, waiting...')
        req = ChangeState.Request()
        req.transition.id = req.transition.TRANSITION_ACTIVATE
        future = mgr_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future or not future.result().success:
            print('Failed to activate sensor, trying again...')
        else:
            activated = True
            print('Ouster node ' + node_name + ' was activated')
            break

    # Validate that it is up fully
    if configured and activated:
        mgr_client = node.create_client(GetState, srv_get_name)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            print(f'{srv_name} service not available, waiting...')
        req = GetState.Request()
        future = mgr_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if future and future.result().current_state.id == 3:
            print('Ouster node ' + node_name + ' was successfully activated!')
            exit(0)

    print('Ouster node ' + node_name + ' was not successfully activated!')
    exit(-1)


if __name__ == '__main__':
    main()
