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

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # realsense_camera_node = Node(
    #     package='realsense2_camera',
    #     executable='realsense2_camera_node',
    #     name='realsense2_camera',
    #     namespace='rs_link1',
    #     parameters=[{
    #         'enable_infra1': False,
    #         'enable_infra2': False,
    #         'enable_color': True,
    #         'enable_rgbd': True,
    #         'enable_sync': True,
    #         'align_depth.enable': True,
    #         'rgb_camera.profile': '640x360x60',
    #         'depth_module.profile': '1280x720x30',
    #         'enable_depth': True,
    #         'publish_tf': True,
    #         'tf_publish_rate': 1.0,
    #         'pointcloud.enable': True,
    #         'pointcloud.texture': True,
    #     }],
    #     # remappings=[('/color/image_raw', '/image'),
    #     #             ('/color/camera_info', '/camera_info')]
    # )

    # rectify_node = ComposableNode(
    #     package='isaac_ros_image_proc',
    #     plugin='nvidia::isaac_ros::image_proc::RectifyNode',
    #     name='rectify',
    #     namespace='',
    #     parameters=[{
    #         'output_width': 640,
    #         'output_height': 360,
    #     }]
    # )

    # apriltag_node = ComposableNode(
    #     package='isaac_ros_apriltag',
    #     plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
    #     name='apriltag',
    #     namespace=''
    # )

    realsense_camera_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense2_camera',
        namespace='rs_on_link1',
        parameters=[{
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_color': True,
            'enable_rgbd': True,
            'enable_sync': True,
            'align_depth.enable': True,
            # 'rgb_camera.profile': '640x360x60',
            'rgb_camera.profile': '1280x720x30',
            # 'rgb_camera.exposure': 150, # 150 indoor, 10 outdoor
            # 'rgb_camera.gain': 10,
            # 'rgb_camera.auto_exposure_roi.bottom': 570,
            # 'rgb_camera.auto_exposure_roi.left': 900,
            # 'rgb_camera.auto_exposure_roi.right': 1280,
            # 'rgb_camera.auto_exposure_roi.top': 150,
            'depth_module.profile': '1280x720x30',
            'enable_depth': True,
            'publish_tf': False,
            # 'tf_publish_rate': 1.0,
            'pointcloud.enable': False,
            'pointcloud.texture': False,
        }],
        # remappings=[('/color/image_raw', '/image'),
        #             ('/color/camera_info', '/camera_info')]
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            # rectify_node,
            # apriltag_node,
            realsense_camera_node
        ],
        output='screen'
    )
    

    return launch.LaunchDescription([apriltag_container])
