import os
import ament_index_python.packages
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # package root
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_auto_docking')

    # kobuki_ros node
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    kobuki_node = ComposableNode(
        package='kobuki_node',
        plugin='kobuki_node::KobukiRos',
        name='kobuki_ros_node',
        parameters=[params]
    )

    # kobuki_auto_docking
    params_file = os.path.join(share_dir, 'config', 'auto_docking.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_auto_docking']['ros__parameters']
    
    kobuki_auto_docking_node = ComposableNode(
        package='kobuki_auto_docking',
        plugin='kobuki_auto_docking::AutoDockingROS',
        name='kobuki_auto_docking',
        #remappings=[
        #    ('velocity', '/commands/velocity'),
        #    ('core', 'sensors/core'),
        #    ('dock_ir', 'sensors/dock_ir')
        #],
        parameters=[params]
    )

    # packs to the container
    mobile_base_container = ComposableNodeContainer(
            name='mobile_base_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                kobuki_node,
                kobuki_auto_docking_node
            ],
            output='both',
    )

    return LaunchDescription([
        mobile_base_container
    ])
