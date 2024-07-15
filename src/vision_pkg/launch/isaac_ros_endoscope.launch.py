# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
# Modified by Chuizheng Kong
# On Date: 05/12/2024

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace='',
        parameters=[{
            'output_width': 1600,
            'output_height': 1200,
        }],
        remappings=[('image_raw', 'endoscope/image_raw'),  # specify this as endoscope
                    ('camera_info', 'endoscope/camera_info'),
                    ('image_rect', 'endoscope/image_rect'),
                    ('camera_info_rect', 'endoscope/camera_info_rect')],
    )

    resize_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        name='resize',
        parameters=[{
            'input_width': 1600,
            'input_height': 1200,
            'output_width': 640,
            'output_height': 480,
        }],
        remappings=[('image', 'endoscope/image_rect'),  # connect to the RectifyNode
                    ('camera_info', 'endoscope/camera_info_rect'),
                    ('resize/image', 'endoscope/resize/image'),  # publish to this topic
                    ('resize/camera_info', 'endoscope/resize/camera_info')],
        )

    usb_cam_params_path = os.path.join(
        get_package_share_directory('vision_pkg'),
        'config',
        'params_endoscope.yaml'
    )
    usb_cam_node = ComposableNode(
        package='usb_cam',
        plugin='usb_cam::UsbCamNode',
        name='usb_cam_endoscope',
        namespace='endoscope',
        parameters=[usb_cam_params_path]
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='endoscope_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            usb_cam_node,
            rectify_node,
            resize_node,
        ],
        output='screen'
    )

    return launch.LaunchDescription([apriltag_container])
