#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2024 ZhangDuo @AUBO Robotics
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
#
# Description:
# This launch file is a part of ROS-LLM project developed to control and interact with the turtlesim robot or your own robot.
# The launch file contains a LaunchDescription object which defines the ROS2 nodes to be executed.
# 
# Node test Method:
# ros2 launch llm_bringup chatgpt_with_arm_robot.launch.py
# ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1

# Author:  ZhangDuo @AUBO Robotics

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
def launch_setup(context, *args, **kwargs):
    robot_launch_file = os.path.join(
        get_package_share_directory('llm_robot'), 'launch', 'llm_robot.launch.py'
    )
    start_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_file),
            launch_arguments={'robot_ip': '127.0.0.1'}.items()
        )
    nodes_to_start = [
        start_robot,
    ]
    return nodes_to_start
def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="audio_input",
                executable="audio_input_local",
                name="audio_input_local",
                output="screen",
            ),
            Node(
                package="llm_model",
                executable="chatllm",
                name="chatllm",
                output="screen",
            ),
            Node(
                package="audio_output",
                executable="audio_output_local",
                name="audio_output_local",
                output="screen",
            ),
        ] 
        # + [OpaqueFunction(function=launch_setup)]
    )
