# Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab
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

"""A Nav2 planner server with the plugin on a static map, driven through its action."""

import math
import os
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
import pytest
import rclpy
from rclpy.action import ActionClient

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', '..', 'config', 'nav2_params.yaml')


@pytest.mark.launch_test
def generate_test_description():
    """Launch a map server, the shipped planner server configuration and a lifecycle manager."""
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': os.path.join(HERE, 'room.yaml')}], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[CONFIG, os.path.join(HERE, 'planner_server.yaml')], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '1.0', '--y', '1.0', '--frame-id', 'map',
                        '--child-frame-id', 'base_link'], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server']}]),
        launch_testing.actions.ReadyToTest(),
    ])


def pose(x, y, yaw):
    """Return a pose in the map frame."""
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(0.5 * yaw)
    p.pose.orientation.w = math.cos(0.5 * yaw)
    return p


class TestPlannerServer(unittest.TestCase):
    """Plans, a typed failure and a cancellation through ComputePathToPose."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('geodex_nav2_planner_test')
        cls.client = ActionClient(cls.node, ComputePathToPose, 'compute_path_to_pose')
        state = cls.node.create_client(GetState, 'planner_server/get_state')
        deadline = time.monotonic() + 60.0
        active = False
        while time.monotonic() < deadline and not active:
            if state.wait_for_service(timeout_sec=1.0):
                future = state.call_async(GetState.Request())
                rclpy.spin_until_future_complete(cls.node, future, timeout_sec=2.0)
                active = future.result() is not None and future.result().current_state.id == 3
            if not active:
                time.sleep(0.5)
        assert active, 'planner_server did not become active'
        assert cls.client.wait_for_server(timeout_sec=30.0)
        # The costmap needs the map before the first plan.
        time.sleep(2.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def send(self, start, goal, planner_id):
        """Send one request and return its goal handle."""
        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal
        request.use_start = True
        request.planner_id = planner_id
        future = self.client.send_goal_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        handle = future.result()
        self.assertIsNotNone(handle)
        self.assertTrue(handle.accepted)
        return handle

    def result(self, handle, timeout):
        """Wait for the result of a goal."""
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), 'no result in time')
        return future.result()

    def test_plans_through_the_door(self):
        goal = pose(8.0, 4.5, math.pi / 2)
        response = self.result(self.send(pose(2.0, 1.5, 0.0), goal, 'GridBased'), 60.0)
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(response.result.error_code, ComputePathToPose.Result.NONE)
        path = response.result.path
        self.assertEqual(path.header.frame_id, 'map')
        self.assertGreater(len(path.poses), 10)
        last = path.poses[-1].pose.position
        self.assertAlmostEqual(last.x, 8.0, places=6)
        self.assertAlmostEqual(last.y, 4.5, places=6)

    def test_a_goal_in_the_wall_is_reported(self):
        response = self.result(self.send(pose(2.0, 1.5, 0.0), pose(5.0, 5.0, 0.0), 'GridBased'),
                               60.0)
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(response.result.error_code, ComputePathToPose.Result.GOAL_OCCUPIED)

    def test_cancel_stops_a_long_plan(self):
        start = time.monotonic()
        handle = self.send(pose(2.0, 1.5, 0.0), pose(8.0, 4.5, 0.0), 'Slow')
        time.sleep(1.0)
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=10.0)
        response = self.result(handle, 20.0)
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertLess(time.monotonic() - start, 10.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Every process exits cleanly."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
