
# ORiN2 Command Interface Control

The ORiN2 command interface allows to control DENSO robots by sending _"high-level"_ commands to the robot controller.
Considering robot motion, only the final target point is sent to the DENSO robot controller.
Motion planning and trajectory generation are internally managed by the DENSO controller (the final robot trajectory is not exposed to ROS2 nodes).


Please refer to the ORiN2 Programming Manual for details about the available robot commands and their parameters:

  - [RC8 Provider User's Guide](https://www.fa-manuals.denso-wave.com/en/usermanuals/001511/)
  - [RC9 Provider User's Guide](https://www.fa-manuals.denso-wave.com/en/RC9/010328/)

## Usage

The main launch file that starts the ORiN2 node is in the `denso_robot_core` package.

  - COBOTTA robot:

   ```bash
   ros2 launch denso_robot_core denso_robot_core.launch.py model:=cobotta ip_address:=192.168.0.1
   ```


  - NOT COBOTTA robot (e.g. VS-060 robot):

   ```bash
   ros2 launch denso_robot_core denso_robot_core.launch.py model:=vs060 ip_address:=192.168.0.1
   ```


### Launch File Details

The main launch file that starts the ORiN2 node is in the `denso_robot_core` package:

   ```bash
   ros2 launch denso_robot_core denso_robot_core.launch.py model:=<robot_model> ip_address:=<robot_ip_address>
   ```

The arguments for launch files can be listed using:

   ```bash
   ros2 launch denso_robot_core denso_robot_core.launch.py --show-args
   ```

The most relevant arguments are the following:

  - `ip_address` (**mandatory**) - IP address of the robot
  - `controller_name` (default: _""_) - string that will be assigned to the ORiN2 Robot Controller Object
  - `controller_type` (default: _8_) - type of robot controller (8 --> RC8 , 9 --> RC9)
  - `model` (default: _""_) - the model of the DENSO robot (COBOTTA, VS-060, etc.).
    In the original DENSO ROS2 stack 2 robot models are already available ( _"cobotta"_ , _"vs060"_). 

     **CAUTION: When connecting to a COBOTTA you MUST specify this parameter !!** (_model:=cobotta_)

  - `bcap_slave_control_cycle_msec` (default: _8.0_) - DENSO robot control cycle \[ms\]

**PLEASE NOTE THAT THE** `model` **PARAMETER WILL ALSO SET A NAMESPACE FOR THE denso_robot_core NODE AND ASSOCIATED TOPICS, SERVICES AND ACTIONS !!**


## Commands Examples

**NOTE**: following example commands assume that the _denso\_robot\_core_ node was launched with `model` parameter not set (default value: _""_) !!
When setting e.g.  `model:=cobotta`, the name of the topics, actions and services will change from e.g. `/ChangeMode` to `/cobotta/ChangeMode`

**Run the following commands in a new Terminal**

 - Get the list of available topics:

   ```bash
   ros2 topic list
   ```

   Output will be:

   ```bash
   [...]

   /Arm0/ACCEL_Read
   /Arm0/CURRENT_ANGLE_Read
   /Arm0/CURRENT_POSITION_Read
   /Arm0/DECEL_Read
   /Arm0/EXTSPEED_Read
   /Arm0/EXTSPEED_Write
   /Arm0/SERVO_ON_Read
   /Arm0/SERVO_ON_Write
   /Arm0/SPEED_Read
   /Arm0_ChangeTool
   /Arm0_ChangeWork
   /Arm0_Speed
   /Arm0_armgroup
   /ChangeMode
   /CurMode
   /D_ID
   /D_Read
   /D_Write
   /EMERGENCY_STOP_Read
   /ERROR_CODE_Read
   /ERROR_CODE_Write
   /F_ID
   /F_Read
   /F_Write
   /IO_ID
   /IO_Read
   /IO_Write
   /I_ID
   /I_Read
   /I_Write
   /J_ID
   /J_Read
   /J_Write
   /LOCK_Read
   /LOCK_Write
   /MODE_Read
   /P_ID
   /P_Read
   /P_Write
   /Pro1/START_Write
   /Pro1/STATUS_Read
   /Pro1/STOP_Write
   /Pro2/START_Write
   /Pro2/STATUS_Read
   /Pro2/STOP_Write
   /S_ID
   /S_Read
   /S_Write
   /T_ID
   /T_Read
   /T_Write
   /V_ID
   /V_Read
   /V_Write

   [...]
   ```


 - Read robot speed:

   ```bash
   ros2 topic echo /Arm0/EXTSPEED_Read
   ```


 - Write robot speed:

   ```bash
   ros2 topic pub --once /Arm0/EXTSPEED_Write std_msgs/msg/Float32 "data: 100"
   ```


 - Read robot motor status:

   ```bash
   ros2 topic echo /Arm0/SERVO_ON_Read
   ```


 - Write robot motor ON / OFF:

   ```bash
   ros2 topic pub --once /Arm0/SERVO_ON_Write std_msgs/msg/Bool "data: False"
   ```


 - List of available actions:

   ```bash
   ros2 action list
   ```

   Output will be:

   ```bash
   [...]

   /Arm0_DriveAExString
   /Arm0_DriveAExValue
   /Arm0_DriveExString
   /Arm0_DriveExValue
   /Arm0_MoveString
   /Arm0_MoveValue

   [...]
   ```


 - Move robot to position stored in controller's variable P\[1\]:

   ```bash
   ros2 action send_goal /Arm0_MoveString denso_robot_core_interfaces/action/MoveString '{comp: 1, pose: "P1", option: ""}'
   ```


 - Read robot I/O lines (e.g. IO24):

   ```bash
   ros2 topic pub --once /IO_ID std_msgs/msg/Int32 "data: 24"
   ros2 topic echo /IO_Read
   ```


 - Write robot I/O lines (e.g. IO24):

   ```bash
   ros2 topic pub --once /IO_ID std_msgs/msg/Int32 "data: 24"
   ros2 topic pub --once /IO_Write std_msgs/msg/Bool "data: true"
   ros2 topic pub --once /IO_Write std_msgs/msg/Bool "data: false"
   ```


## COBOTTA

To execute CALSET on COBOTTA without TP, set "252: CALSET on start-up" to "1: DoNot". Then execute AutoCal Command.

cf. [COBOTTA USER MANUAL ID:7299](https://www.fa-manuals.denso-wave.com/en/COBOTTA/007299/)
