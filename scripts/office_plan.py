#!/usr/bin/env python3
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
"""
Request a path through the office from the planner server of office.launch.py.

The request goes from (3.885, 1.235) at a heading of -2.81 degrees to (12.425, 3.675) at
-120.44 degrees. --out writes the returned poses as CSV.

    ros2 run geodex_nav2_planner office_plan.py [--out path.csv]
"""

import argparse
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient

START = (3.885, 1.235, math.radians(-2.81))
GOAL = (12.425, 3.675, math.radians(-120.44))


def pose(x, y, yaw):
    """Return a pose in the map frame."""
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def wait_active(node, timeout=60.0):
    """Wait until the planner server is active and its costmap has the map."""
    state = node.create_client(GetState, 'planner_server/get_state')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if state.wait_for_service(timeout_sec=1.0):
            future = state.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if future.result() is not None and future.result().current_state.id == 3:
                time.sleep(2.0)
                return
        time.sleep(0.5)
    raise RuntimeError('planner_server did not become active')


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--out', type=Path, help='CSV file for the returned poses')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('geodex_nav2_office_client')
    try:
        wait_active(node)
        client = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        if not client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('compute_path_to_pose is not available')
        goal = ComputePathToPose.Goal(start=pose(*START), goal=pose(*GOAL), use_start=True,
                                      planner_id='GridBased')
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=10.0)
        result = sent.result().get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=180.0)
        response = result.result().result
        poses = [(p.pose.position.x, p.pose.position.y,
                  2.0 * math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                 for p in response.path.poses]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:]))
        planning_time = response.planning_time.sec + response.planning_time.nanosec * 1e-9
        print(f'error_code={response.error_code} poses={len(poses)} length={length:.3f} m '
              f'planning_time={1000 * planning_time:.0f} ms')
        if args.out:
            rows = ''.join(f'{x:.6f},{y:.6f},{t:.6f}\n' for x, y, t in poses)
            args.out.write_text('x,y,theta\n' + rows)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
