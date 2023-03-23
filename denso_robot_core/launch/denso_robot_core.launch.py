# Copyright (c) 2021 DENSO WAVE INCORPORATED
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
# Author: DENSO WAVE INCORPORATED

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


""" Launch Description generator function. """


def generate_launch_description():

    denso_robot_core_pkg = get_package_share_directory('denso_robot_core')

    declared_arguments = []
# bcap_service connection arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'model', default_value='',
            description='Type/series of used denso robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'config_file', default_value='config.xml',
            description='Path to the bcap config file.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address', default_value='192.168.0.1',
            description='IP address by which the robot can be reached.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'bcap_slave_control_cycle_msec', default_value='8.0',
            description='Control frequency.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_name', default_value='',
            description='Name of the DENSO controller object.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_type', default_value='8',
            description='Type of DENSO controller.'))

# Initialize Arguments
    denso_robot_model = LaunchConfiguration('model')
    config_file = LaunchConfiguration('config_file')
    ip_address = LaunchConfiguration('ip_address')
    bcap_slave_control_cycle_msec = LaunchConfiguration('bcap_slave_control_cycle_msec')
    controller_name = LaunchConfiguration('controller_name')
    controller_type = LaunchConfiguration('controller_type')

    denso_robot_core_parameters = {
        'denso_config_file': PathJoinSubstitution([denso_robot_core_pkg, 'config', config_file]),
        'denso_ip_address': ip_address,
        'denso_bcap_slave_control_cycle_msec': bcap_slave_control_cycle_msec,
        'denso_controller_name': controller_name,
        'denso_controller_type': controller_type,
        'denso_robot_model': denso_robot_model,
        }

    denso_robot_core_node = Node(
        package='denso_robot_core',
        executable='denso_robot_core_exec',
        name='denso_robot_core_node',
        namespace=denso_robot_model,
        parameters=[
            denso_robot_core_parameters,
        ])

    nodes_to_start = [denso_robot_core_node]

    return LaunchDescription(declared_arguments + nodes_to_start)
