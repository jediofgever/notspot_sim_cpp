# Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use or not sim time.')

    # Declare node actions
    imrt_virtual_joy = Node(
        package='imrt_virtual_joy',
        executable='imrt_virtual_joy',
        name='imrt_virtual_joy',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(imrt_virtual_joy)

    return ld
