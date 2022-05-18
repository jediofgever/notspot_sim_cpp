# Copyright (c) 2021 Fetullah Atas, Norwegian University of Life Sciences
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable

import os


def generate_launch_description():

    # Get hare directories of thorvald packages
    notspot_bringup_share_dir = get_package_share_directory(
        'notspot_bringup')
    notspot_description_share_dir = get_package_share_directory(
        'notspot_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='whether to use or not sim time.')

    xacro_full_dir = os.path.join(
        notspot_description_share_dir, 'urdf/notspot.urdf.xacro')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': Command(['xacro ', xacro_full_dir])}],
        remappings=remappings)

    spawn_entity_to_gazebo = Node(package='gazebo_ros',
                                  executable='spawn_entity.py',
                                  parameters=[
                                      {"use_sim_time": use_sim_time}],
                                  arguments=['-entity', '',
                                             '-z', '0.5',
                                             '-topic', '/robot_description'],
                                  output='screen')

    # START GAZEBO ONLY IF use_simulator IS SET TO TRUE
    gazebo_world = os.path.join(get_package_share_directory(
        'notspot_gazebo'), 'world/', 'normal.world'),
    declare_start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', gazebo_world,
            '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'
        ],
        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_trajectory_controller'],
        output='screen'
    )

    # Declare node actions
    robot_controller_gazebo = Node(
        package='notspot_controller',
        executable='robot_controller_gazebo',
        name='robot_controller_gazebo',
        output='screen',
        parameters=[])

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        declare_start_gazebo_cmd,
        spawn_entity_to_gazebo,
        load_joint_state_controller,
        load_joint_trajectory_controller,
        robot_controller_gazebo
    ])
