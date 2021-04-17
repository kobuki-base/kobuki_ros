import os

import ament_index_python.packages
import launch
import launch_ros.actions

import yaml

# Example standalone launcher for the bumper/cliff to pointcloud node.
#
# The 'pointcloud_radius' parameter gives the bumper/cliff pointcloud distance to base frame;
# it should be something like the robot radius plus plus costmap resolution plus an extra to
# cope with robot inertia. This parameter is a bit tricky: if it's too low, costmap will
# ignore this pointcloud (the robot footprint runs over the hit obstacle), but if it's too
# big, hit obstacles will be mapped too far from the robot and the navigation around them
# will probably fail.

def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_bumper2pc')
    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    params_file = os.path.join(share_dir, 'config', 'kobuki_bumper2pc_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_bumper2pc']['ros__parameters']
    kobuki_bumper2pc_node = launch_ros.actions.Node(package='kobuki_bumper2pc',
                                              node_executable='kobuki_bumper2pc',
                                              output='both',
                                              parameters=[params])

    return launch.LaunchDescription([kobuki_bumper2pc_node])
