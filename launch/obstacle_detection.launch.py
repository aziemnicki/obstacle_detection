# Copyright 2024 Andrzej_Norbert_Jeremiasz
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('obstacle_detection')
    param_path = PathJoinSubstitution([pkg_prefix, 'config/obstacle_detection.param.yaml'])

    obstacle_detection_node = Node(
        package='obstacle_detection',
        executable='obstacle_detection_node_exe',
        name='obstacle_detection_node',
        parameters=[
            param_path
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
        remappings=[
            ("~/input/laser_scan", LaunchConfiguration('input_scan')),
            ("~/input/occupancy_grid", LaunchConfiguration('input_occupancy_grid')),
            ("~/input/odometry", LaunchConfiguration('input_odometry')),
            ("~/output/occupancy_grid", LaunchConfiguration('output_occupancy_grid')),
        ],
    )

    return [
        obstacle_detection_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('input_scan', '/sensing/lidar/scan')
    add_launch_arg('input_occupancy_grid', '/map')
    add_launch_arg('input_odometry', '/localization/kinematic_state')
    add_launch_arg('output_occupancy_grid', '/modified_map')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])