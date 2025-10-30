# Commands

## Overview
This document provides an overview of the commands available for controlling Denso robots using ROS 2. The commands are designed to facilitate communication between the ROS 2 framework and Denso robot controllers, enabling users to perform various operations such as moving the robot, setting parameters, and retrieving status information.

## Run Moveit and Control Gripper

```bash
ros2 launch bcap_service bcap_service.launch.py model:=cobotta ip_address:=172.16.6.103

ros2 launch denso_robot_bringup denso_robot_bringup.launch.py model:=cobotta sim:=false ip_address:=172.16.6.103 send_format:=0 recv_format:=2

ros2 launch denso_robot_bringup denso_gripper_bringup.launch.py
```

## Open and Close gripper

```bash
ros2 service call /gripper_service bcap_service_interfaces/srv/Gripper "value: 30
speed: 10" 
requester: making request: bcap_service_interfaces.srv.Gripper_Request(value=30, speed=10)

response:
bcap_service_interfaces.srv.Gripper_Response(success=True, message='Gripper moved successfully.')
```