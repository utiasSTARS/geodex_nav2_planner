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
Start a Nav2 planner server with geodex_nav2_planner on the map of an office.

The robot argument names a file of config/robots, and rviz:=true starts RViz.

    ros2 launch geodex_nav2_planner office.launch.py robot:=jackal rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The start pose of office_plan.py, (x, y, yaw) in meters and radians.
START = ('3.885', '1.235', '-0.049044')


def _nodes(context):
    share = get_package_share_directory('geodex_nav2_planner')
    robot = LaunchConfiguration('robot').perform(context)
    params = [os.path.join(share, 'config', 'nav2_params.yaml'),
              os.path.join(share, 'config', 'robots', f'{robot}.yaml'),
              os.path.join(share, 'config', 'office.yaml')]
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': os.path.join(share, 'maps', 'office.yaml')}],
             output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=params, output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', START[0], '--y', START[1], '--yaw', START[2],
                        '--frame-id', 'map', '--child-frame-id', 'base_link'], output='log'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server']}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'rviz', 'office.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='jackal',
                              description='a file of config/robots'),
        DeclareLaunchArgument('rviz', default_value='false', description='start RViz'),
        OpaqueFunction(function=_nodes),
    ])
