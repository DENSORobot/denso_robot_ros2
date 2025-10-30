from launch import LaunchDescription
from launch_ros.actions import Node

#!/usr/bin/env python3

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='denso_robot_control',
            executable='denso_gripper_control_node',
            name='denso_gripper_control_node',
            output='screen',
        )
    ])