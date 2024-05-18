#!/usr/bin/env python3
#
# Author: Chuizheng Kong
# Date: 05/17/2024

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    nvidia_apriltag_detect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vision_pkg'), 
                         'launch/isaac_ros_apriltag_endoscope.launch.py')
        ) 
    )

    gen3_controller = Node(
        package='gen3_7dof',
        executable='chase_controller_node',
        name='chase_controller_node'
    )

    tag_follower = Node(
        package='gen3_7dof',
        executable='apriltag_follower_node',
        name='apriltag_follower_node'
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(nvidia_apriltag_detect)
    ld.add_action(gen3_controller)
    ld.add_action(tag_follower)
    return ld