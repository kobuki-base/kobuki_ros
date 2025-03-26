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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    kobuki_pkg = get_package_share_directory('kobuki_description')

    urdf_xacro_file = os.path.join(kobuki_pkg, 'urdf', 'kobuki_hexagons_asus_xtion_pro.urdf.xacro')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to apply to the nodes'
    )

    namespace = LaunchConfiguration('namespace')

    # Robot description
    robot_model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(
                Command(['xacro ', urdf_xacro_file]), value_type=str),}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # TF Tree
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(robot_model)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(namespace_arg)

    return ld