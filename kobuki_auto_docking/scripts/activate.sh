#!/usr/bin/env bash

set -e

ros2 action send_goal --feedback /auto_docking_action kobuki_ros_interfaces/action/AutoDocking {}
