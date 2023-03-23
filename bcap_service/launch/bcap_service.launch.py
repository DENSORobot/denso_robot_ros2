# Copyright (c) 2021 Denso Wave Incorporated
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
# Author: Denso Wave Incorporated

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

""" Launch Description generator function. """


def generate_launch_description():
    bcap_service_pkg = get_package_share_directory('bcap_service')

    declared_arguments = []
# bcap_service connection arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'conn_type',
            default_value='tcp',
            description='Connection type used [udp/tcp].'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.0.1',
            description='IP address by which the robot can be reached.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'port_number',
            default_value='5007',
            description='Number of the port for connection to the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'timeout',
            default_value='3000',
            description='Connection timeout parameter.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'retry_count',
            default_value='5',
            description='Retry count parameter.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'wait_time',
            default_value='0',
            description='Wait time parameter.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'watchdog_timer',
            default_value='400',
            description='Watchdog parameter.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'invoke_timeout',
            default_value='180000',
            description='Invoke Timeout parameter.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Type/series of used denso robot.'))

# Initialize Arguments
    denso_robot_model = LaunchConfiguration('model')
    conn_type = LaunchConfiguration('conn_type')
    ip_address = LaunchConfiguration('ip_address')
    port_number = LaunchConfiguration('port_number')
    timeout = LaunchConfiguration('timeout')
    retry_count = LaunchConfiguration('retry_count')
    wait_time = LaunchConfiguration('wait_time')
    watchdog_timer = LaunchConfiguration('watchdog_timer')
    invoke_timeout = LaunchConfiguration('invoke_timeout')

    bcap_service_parameters = {
        'denso_conn_type': conn_type,
        'denso_ip_address': ip_address,
        }

    bcap_service_node = Node(
        package='bcap_service',
        name='bcap_service_node',
        namespace=denso_robot_model,
        executable='bcap_service_exec',
        parameters=[
            os.path.join(bcap_service_pkg, 'config', 'bcap_service.yaml'),
            bcap_service_parameters])

    nodes_to_start = [bcap_service_node]

    return LaunchDescription(declared_arguments + nodes_to_start)
