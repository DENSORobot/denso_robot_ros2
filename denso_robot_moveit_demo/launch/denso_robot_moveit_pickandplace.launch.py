import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


""" Function for loading a yaml file. """


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


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
            'sim', default_value='true',
            description='Start robot with fake hardware mirroring command to its states.'))
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
            'scale_factor', default_value='0.1',
            description='Scale factor for motion execution (speed and acc).'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_cycles', default_value='1',
            description='Number of times to repeat the pick-and-place cycle.'))

    denso_robot_model = LaunchConfiguration('model')
    sim = LaunchConfiguration('sim')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    scale_factor = LaunchConfiguration('scale_factor')
    num_cycles = LaunchConfiguration('num_cycles')

    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_cpp_yaml_file_name = (
        get_package_share_directory('denso_robot_moveit_demo') \
        + '/config/denso_robot_moveit_demo.yaml')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]),
            ' ',
            'model:=', denso_robot_model, ' ',
            'sim:=', sim, ' ',
            "namespace:=''"
        ])

    robot_description = {'robot_description': robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), 'srdf', moveit_config_file]),
            ' ',
            'model:=', denso_robot_model, ' ',
            "namespace:=''"
        ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    kinematics_yaml = load_yaml('denso_robot_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
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

    # controllers_yaml = load_yaml('denso_robot_moveit_config', 'config/controllers.yaml')
    moveit_controllers = {
        # 'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'\
            +'/MoveItSimpleControllerManager',
    }
    moveit_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package), 'robots',
            denso_robot_model, 'config/moveit_controllers.yaml'
        ])

    denso_robot_moveit_demo_params = {
        'model': denso_robot_model,
        'scale_factor': scale_factor,
        'num_cycles': num_cycles,
    }

    # MoveItCpp pick-and-place executable
    denso_robot_moveit_pickandplace_node = Node(
        package='denso_robot_moveit_demo',
        executable='denso_robot_moveit_pickandplace',
        output='screen',
        parameters=[
            moveit_cpp_yaml_file_name,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            moveit_controllers,
            moveit_controllers_file,
            denso_robot_moveit_demo_params
        ])

    return LaunchDescription(declared_arguments + [denso_robot_moveit_pickandplace_node,])
