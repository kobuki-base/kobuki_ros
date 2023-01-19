# Copyright 2022 Intelligent Robotics Lab
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    kobuki_pkg = get_package_share_directory('kobuki_description')

    urdf_file = os.path.join(kobuki_pkg, 'urdf', 'kobuki.urdf')

    with open(urdf_file, 'r') as info:
        robot_desc = info.read()

    # Robot description
    robot_model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf_file]
    )

    # TF Tree
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Spawn
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='entity_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", "kobuki"]
    )

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(robot_model)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity_node)
    return ld