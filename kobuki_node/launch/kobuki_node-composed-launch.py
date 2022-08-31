import os

import ament_index_python.packages

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import yaml


def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    # Passing parameters to a composed node must be done via a dictionary of
    # key -> value pairs.  Here we read in the data from the configuration file
    # and create a dictionary of it that the ComposableNode will accept.
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    container = ComposableNodeContainer(
            name='kobuki_node_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='kobuki_node',
                    plugin='kobuki_node::KobukiRos',
                    name='kobuki_ros_node',
                    parameters=[params]),
            ],
            output='both',
    )

    return LaunchDescription([container])
