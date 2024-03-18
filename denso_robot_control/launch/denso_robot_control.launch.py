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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from typing import Text
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from typing import Iterable
from typing import Text
from launch.some_substitutions_type import SomeSubstitutionsType


""" Substitution class for appending LaunchConfig parameters to a string.

Helpful for namespaces and/or MULTI-ROBOT applications.
"""


class TextJoinSubstitution(Substitution):
    """Substitution that join paths, in a platform independent way."""

    def __init__(
            self, substitutions: Iterable[SomeSubstitutionsType], text: Text,
            sequence: Text) -> None:
        super().__init__()
        """Create a TextJoinSubstitution."""
        from launch.utilities import normalize_to_list_of_substitutions
        self.__substitutions = normalize_to_list_of_substitutions(substitutions)
        self.__text = text
        self.__sequence = sequence

    @property
    def substitutions(self) -> Iterable[Substitution]:
        """Getter for variable_name."""
        return self.__substitutions

    def text(self) -> Text:
        """Getter for text."""
        return self.__text

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return "LocalVar('{}')".format(' + '.join([s.describe() for s in self.substitutions]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by retrieving the local variable."""
        performed_substitutions = [sub.perform(context) for sub in self.__substitutions]
        return self.__sequence.join(performed_substitutions) + self.__sequence + self.__text


""" Launch Description generator function. """


def generate_launch_description():

    declared_arguments = []
# Denso specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            description='Type/series of used denso robot.'))
    # TODO: shall we let the user to only select from a list of robots ??
    # choices=['cobotta', 'vs060', 'vs087']))
    declared_arguments.append(
        DeclareLaunchArgument(
            'send_format', default_value='288',
            description='Data format for sending commands to the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'recv_format', default_value='292',
            description='Data format for receiving robot status.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'bcap_slave_control_cycle_msec', default_value='8.0',
            description='Control frequency.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address', default_value='192.168.0.1',
            description='IP address by which the robot can be reached.'))
# Configuration arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package', default_value='denso_robot_descriptions',
            description='Description package with robot URDF/XACRO files. Usually the argument \
                is not set, it enables use of a custom description.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file', default_value='denso_robot.urdf.xacro',
            description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package', default_value='denso_robot_moveit_config',
            description='MoveIt config package with robot SRDF/XACRO files. Usually the argument \
                is not set, it enables use of a custom moveit config.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_file', default_value='denso_robot.srdf.xacro',
            description='MoveIt SRDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', default_value='',  # 'denso',
            description="Prefix of the joint names, useful for \
                multi-robot setup. If changed than also joint names in the controllers' \
                configuration have to be updated."))
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file', default_value='denso_robot_controllers.yaml',
            description='YAML file with the controllers configuration.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller', default_value='denso_joint_trajectory_controller',
            description='Robot controller to start.'))
# Execution arguments (Rviz and Gazebo)
# TODO: shall we give the user the choice not to load the rviz graphical environment ??
#    declared_arguments.append(
#        DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz?')
#    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Start robot with fake hardware mirroring command to its states.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Print out additional debug information.'))

# Initialize Arguments
    denso_robot_model = LaunchConfiguration('model')
    ip_address = LaunchConfiguration('ip_address')
    send_format = LaunchConfiguration('send_format')
    recv_format = LaunchConfiguration('recv_format')
    bcap_slave_control_cycle_msec = LaunchConfiguration('bcap_slave_control_cycle_msec')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    namespace = LaunchConfiguration('namespace')
#    launch_rviz = LaunchConfiguration('launch_rviz')
    sim = LaunchConfiguration('sim')
    verbose = LaunchConfiguration('verbose')
    controllers_file = LaunchConfiguration('controllers_file')
    robot_controller = LaunchConfiguration('robot_controller')

    denso_robot_core_pkg = get_package_share_directory('denso_robot_core')

    denso_robot_control_parameters = {
        'bcap_slave_control_cycle_msec': bcap_slave_control_cycle_msec,
        'config_file': PathJoinSubstitution([denso_robot_core_pkg, 'config', 'config.xml']),
    }

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]),
            ' ',
            'ip_address:=', ip_address, ' ',
            'model:=', denso_robot_model, ' ',
            'send_format:=', send_format, ' ',
            'recv_format:=', recv_format, ' ',
            'namespace:=', namespace, ' ',
            'verbose:=',  verbose, ' ',
            'sim:=', sim, ' '
        ])
    robot_description = {'robot_description': robot_description_content}

# --------- Robot Control Node (only if 'sim:=false') ---------
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package), 'robots',
            denso_robot_model, 'config', controllers_file
        ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        condition=UnlessCondition(sim),
        parameters=[
            robot_description,
            robot_controllers,
            denso_robot_control_parameters
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        })

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['denso_joint_state_broadcaster', '--controller-manager', '/controller_manager'])

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller, '-c', '/controller_manager'])

# TODO: do we need the Warehouse mongodb server ?
# (always / never / only in simulation with Gazebo ...)
    # Warehouse mongodb server

#    mongodb_server_node = Node(
#        package='warehouse_ros_mongo',
#        executable='mongo_wrapper_ros.py',
#        parameters=[
#            {'warehouse_port': 33829},
#            {'warehouse_host': 'localhost'},
#            {'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'}
#        ],
#        output='screen',
#    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', TextJoinSubstitution([namespace], 'base_link', '')
        ])

    nodes_to_start = [
        control_node,
        robot_controller_spawner,
#        mongodb_server_node,
        static_tf,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
