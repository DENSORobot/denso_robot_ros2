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


""" Function for loading a yaml file. """


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


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
            description='Description package with robot URDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom description.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file', default_value='denso_robot.urdf.xacro',
            description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package', default_value='denso_robot_moveit_config',
            description='MoveIt config package with robot SRDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom moveit config.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_file', default_value='denso_robot.srdf.xacro',
            description='MoveIt SRDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', default_value='',
            description="Prefix of the joint names, useful for" \
                + " multi-robot setup. If changed than also joint names in the controllers'" \
                + " configuration have to be updated."))
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
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    namespace = LaunchConfiguration('namespace')
#    launch_rviz = LaunchConfiguration('launch_rviz')
    sim = LaunchConfiguration('sim')
    verbose = LaunchConfiguration('verbose')
    controllers_file = LaunchConfiguration('controllers_file')
    robot_controller = LaunchConfiguration('robot_controller')

    denso_robot_core_pkg = get_package_share_directory('denso_robot_core')

    denso_robot_control_parameters = {
        'denso_bcap_slave_control_cycle_msec': bcap_slave_control_cycle_msec,
        'denso_config_file': PathJoinSubstitution([denso_robot_core_pkg, 'config', 'config.xml'])}

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
            'verbose:=', verbose, ' ',
            'sim:=', sim, ' '
        ])
    robot_description = {'robot_description': robot_description_content}

# --------- MoveIt Configuration ---------

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), 'srdf', moveit_config_file]),
            ' ',
            'model:=', denso_robot_model, ' ',
            'namespace:=', namespace, ' '
        ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    kinematics_yaml = load_yaml('denso_robot_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            # 'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization \
                # default_planner_request_adapters/FixWorkspaceBounds \
                # default_planner_request_adapters/FixStartStateBounds \
                # default_planner_request_adapters/FixStartStateCollision \
                # default_planner_request_adapters/FixStartStatePathConstraints""",
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization' \
                + ' default_planner_request_adapters/FixWorkspaceBounds' \
                + ' default_planner_request_adapters/FixStartStateBounds' \
                + ' default_planner_request_adapters/FixStartStateCollision' \
                + ' default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('denso_robot_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager'\
            + '/MoveItSimpleControllerManager',
    }
    moveit_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package), 'robots',
            denso_robot_model, 'config/moveit_controllers.yaml'
        ])
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
            'attached_collision_object_topic': '/move_group/planning_scene_monitor',
            'publish_planning_scene_topic': '/move_group/publish_planning_scene',
            'monitored_planning_scene_topic': '/move_group/monitored_planning_scene',
            'wait_for_initial_state_timeout': 10.0,
        },
    }

    occupancy_map_monitor_parameters = {
        'sensors': ['3D_sensor'],
        '3D_sensor': {
            'sensor_plugin': '', #'~'
        },
    }
    robot_limits_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package), 'robots',
            denso_robot_model, 'config/joint_limits.yaml'
        ])

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_limits_file,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            moveit_controllers_file,
            occupancy_map_monitor_parameters,
            planning_scene_monitor_parameters,
            {'use_sim_time': sim}
        ])

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
        parameters=[{'use_sim_time': sim}, robot_description]
    )

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

# --------- rviz with moveit configuration ---------
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), 'rviz', 'view_robot.rviz'])

    rviz_node = Node(
        package='rviz2',
#        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics
        ])

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

# --------- Gazebo Nodes (only if 'sim:=true') ---------
    gazebo = ExecuteProcess(
        condition=IfCondition(sim),
        cmd=['gazebo', '--verbose', 'worlds/empty.world', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        condition=IfCondition(sim),
        arguments=['-topic', 'robot_description', '-entity', denso_robot_model],
        output='screen')

    nodes_to_start = [
        control_node,
        robot_controller_spawner,
        move_group_node,
#        mongodb_server_node,
        rviz_node,
        static_tf,
        gazebo,
        spawn_entity,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
