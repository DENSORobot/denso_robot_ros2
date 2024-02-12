# b-CAP Command Interface Control

The [b-CAP command interface](https://www.denso-wave.com/en/robot/product/function/b-CAP.html) allows to control DENSO robots by sending _"high-level"_ commands to the robot controller.
Considering robot motion, only the final target point is sent to the DENSO robot controller.
Motion planning and trajectory generation are internally managed by the DENSO controller (the final robot trajectory is not exposed to ROS2 nodes).
Compared to the [ORiN2 command interface](denso_robot_core/README.md), the [b-CAP command interface](https://www.denso-wave.com/en/robot/product/function/b-CAP.html) exposes basic functions and methods to the ROS2 nodes.


Please refer to the ORiN2 Programming Manual for details about the available robot commands and their parameters:

  - [RC8 Provider User's Guide](https://www.fa-manuals.denso-wave.com/en/usermanuals/001511/)
  - [RC9 Provider User's Guide](https://www.fa-manuals.denso-wave.com/en/RC9/010328/)


## Usage

The main launch file that starts the application is in the `bcap_service` package.

  - COBOTTA robot:

   ```bash
   ros2 launch bcap_service bcap_service.launch.py model:=cobotta ip_address:=192.168.0.1
   ```

  - NOT COBOTTA robot (e.g. VS-060 robot):

   ```bash
   ros2 launch bcap_service bcap_service.launch.py model:=vs060 ip_address:=192.168.0.1
   ```


### Launch File Details

The main launch file that starts the application is in the `bcap_service` package:

   ```bash
   ros2 launch bcap_service bcap_service.launch.py model:=<robot_model> ip_address:=<robot_ip_address>
   ```

The arguments for launch files can be listed using:

   ```bash
   ros2 launch bcap_service bcap_service.launch.py --show-args
   ```

The most relevant arguments are the following:

  - `robot_ip` (**mandatory**) - IP address of the robot
  - `conn_type` (default: _tcp_) - type of connection protocol to be used (_tcp_/_udp_)
  - `port_number` (default: _5007_) - number of the port that will be used for connection
  - `wait_time` (default: _0_) - time to wait before connecting to the robot controller \[s\]
  - `model` (default: _""_) - the model of the DENSO robot (COBOTTA, VS-060, etc.).
    In the original DENSO ROS2 stack 2 robot models are already available ( _"cobotta"_ , _"vs060"_)

**PLEASE NOTE THAT THE** `model` **PARAMETER WILL ALSO SET A NAMESPACE FOR THE _bcap\_service_ NODE AND ASSOCIATED TOPICS AND SERVICES !!**

## Commands Examples

**NOTE**: following example commands assume that the _bcap\_service_ node was launched with `model` parameter not set (default value: _""_) !!
When setting e.g.  `model:=cobotta`, the name of the topics and services will change from e.g. `/bcap_service` to `/cobotta/bcap_service`

**Run the following commands in a new Terminal**

 - Establish connection with the controller, and get the Controller Object Handle:

   ```bash
   ros2 service call /bcap_service bcap_service_interfaces/srv/Bcap '{func_id: 3, vnt_args: [{vt: 8, value: "b-CAP"}, {vt: 8, value: "CaoProv.DENSO.VRC"}, {vt: 8, value: "localhost"}, {vt: 8, value: ""}] }'
   ```

   Output will be (**please note the returned controller handle in the response field,** _"value=\`2\`"_):

   ```bash
   requester: making request: bcap_service_interfaces.srv.Bcap_Request(func_id=3, vnt_args=[bcap_service_interfaces.msg.Variant(vt=8, value='b-CAP'), bcap_service_interfaces.msg.Variant(vt=8, value='CaoProv.DENSO.VRC'), bcap_service_interfaces.msg.Variant(vt=8, value='localhost'), bcap_service_interfaces.msg.Variant(vt=8, value='')])

   response:
   bcap_service_interfaces.srv.Bcap_Response(hresult=0, vnt_ret=bcap_service_interfaces.msg.Variant(vt=19, value='2'))
   ```


 - Get the list of Robot Objects from the controller (**please use the value of the controller handle returned by the previous command,** _"{vt: 19, value: \'2\'}"_):

   ```bash
   ros2 service call /bcap_service bcap_service_interfaces/srv/Bcap '{func_id: 13, vnt_args: [{vt: 19, value: '2'}, {vt: 8, value: ''}] }'
   ```

   Output will be (**please note the returned robot identifier in the response field,** _"value=\`Arm0\`"_):

   ```bash
   waiting for service to become available...
   requester: making request: bcap_service_interfaces.srv.Bcap_Request(func_id=13, vnt_args=[bcap_service_interfaces.msg.Variant(vt=19, value='2'), bcap_service_interfaces.msg.Variant(vt=8, value='None')])

   response:
   bcap_service_interfaces.srv.Bcap_Response(hresult=0, vnt_ret=bcap_service_interfaces.msg.Variant(vt=8200, value='Arm0'))
   ```



 - Get the Robot Object Handle (**please use the values of the controller handle,** _"{vt: 19, value: \'2\'}"_ **, and of the robot identifier,** _"{vt: 8, value: \'Arm0\'}"_ **, returned by the previous commands**):

   ```bash
   ros2 service call /bcap_service bcap_service_interfaces/srv/Bcap '{func_id: 7, vnt_args: [{vt: 19, value: '2'}, {vt: 8, value: 'Arm0'}, {vt: 8, value: ''}] }'
   ```

   Output will be (**please note the returned robot handle in the response field,** _"value=\`16\`"_):

   ```bash
   waiting for service to become available...
   requester: making request: bcap_service_interfaces.srv.Bcap_Request(func_id=7, vnt_args=[bcap_service_interfaces.msg.Variant(vt=19, value='15'), bcap_service_interfaces.msg.Variant(vt=8, value='Arm0'), bcap_service_interfaces.msg.Variant(vt=8, value='None')])

   response:
   bcap_service_interfaces.srv.Bcap_Response(hresult=0, vnt_ret=bcap_service_interfaces.msg.Variant(vt=19, value='16'))
   ```


 - Call service request for b-CAP command, e.g. _"TakeArm"_ command (**please use the value of the robot handle returned by the previous command,** _"{vt: 19, value: \'16\'}"_):

   ```bash
   ros2 service call /bcap_service bcap_service_interfaces/srv/Bcap '{func_id: 64, vnt_args: [{vt: 19, value: '16'}, {vt: 8, value: 'TakeArm'}, {vt: 8195, value: "0,0"}] }'
   ```

   Output will be:

   ```bash
   requester: making request: bcap_service_interfaces.srv.Bcap_Request(func_id=64, vnt_args=[bcap_service_interfaces.msg.Variant(vt=19, value='16'), bcap_service_interfaces.msg.Variant(vt=8, value='TakeArm'), bcap_service_interfaces.msg.Variant(vt=8195, value='0,0')])

   response:
   bcap_service_interfaces.srv.Bcap_Response(hresult=0, vnt_ret=bcap_service_interfaces.msg.Variant(vt=0, value=''))
   ```


 - Call service request for b-CAP command, e.g. _"Motor"_ ON command (**please use the value of the robot handle returned by the previous command,** _"{vt: 19, value: \'16\'}"_):

   ```bash
   ros2 service call /bcap_service bcap_service_interfaces/srv/Bcap '{func_id: 64, vnt_args: [{vt: 19, value: '16'}, {vt: 8, value: 'Motor'}, {vt: 8195, value: "1,0"}] }'
   ```

   Output will be:

   ```bash
   waiting for service to become available...
   requester: making request: bcap_service_interfaces.srv.Bcap_Request(func_id=64, vnt_args=[bcap_service_interfaces.msg.Variant(vt=19, value='16'), bcap_service_interfaces.msg.Variant(vt=8, value='Motor'), bcap_service_interfaces.msg.Variant(vt=8195, value='1,0')])

   response:
   bcap_service_interfaces.srv.Bcap_Response(hresult=0, vnt_ret=bcap_service_interfaces.msg.Variant(vt=0, value=''))
   ```


 - Call service request for b-CAP command, e.g. _"Move"_ command to position stored in controller's variable P\[1\] (**please use the value of the robot handle returned by the previous command,** _"{vt: 19, value: \'16\'}"_):

   ```bash
   ros2 service call /bcap_service bcap_service_interfaces/srv/Bcap '{func_id: 72, vnt_args: [{vt: 19, value: '16'}, {vt: 3, value: '1'}, {vt: 8, value: "P1"}, {vt: 8, value: 'SPEED=10'}] }'
   ```

   Output will be (**please note that the robot will move!!**):

   ```bash
   waiting for service to become available...
   requester: making request: bcap_service_interfaces.srv.Bcap_Request(func_id=72, vnt_args=[bcap_service_interfaces.msg.Variant(vt=19, value='16'), bcap_service_interfaces.msg.Variant(vt=3, value='1'), bcap_service_interfaces.msg.Variant(vt=8, value='P1'), bcap_service_interfaces.msg.Variant(vt=8, value='SPEED=10')])

   response:
   bcap_service_interfaces.srv.Bcap_Response(hresult=0, vnt_ret=bcap_service_interfaces.msg.Variant(vt=0, value=''))
   ```


 - Call service request for b-CAP command, e.g. _"HandMoveA"_ command (**please use the value of the controller handle returned by the previous command,** _"{vt: 19, value: \'2\'}"_):

   **NOTE**: this command is only available for COBOTTA robot models !!

   ```bash
   ros2 service call /bcap_service bcap_service_interfaces/srv/Bcap '{func_id: 17, vnt_args: [{vt: 19, value: '2'}, {vt: 8, value: "HandMoveA"}, {vt: 8195, value: "30, 100"}] }'
   ```

   Output will be (**please note that the robot will move!!**):

   ```bash
   waiting for service to become available...
   requester: making request: bcap_service_interfaces.srv.Bcap_Request(func_id=17, vnt_args=[bcap_service_interfaces.msg.Variant(vt=19, value='2'), bcap_service_interfaces.msg.Variant(vt=8, value='HandMoveA'), bcap_service_interfaces.msg.Variant(vt=8195, value='30, 100')])

   response:
   bcap_service_interfaces.srv.Bcap_Response(hresult=0, vnt_ret=bcap_service_interfaces.msg.Variant(vt=0, value=''))
   ```


 - Call service request for b-CAP command, e.g. _"Motor"_ OFF command (**please use the value of the robot handle returned by the previous command,** _"{vt: 19, value: \'16\'}"_):

   ```bash
   ros2 service call /bcap_service bcap_service_interfaces/srv/Bcap '{func_id: 64, vnt_args: [{vt: 19, value: '16'}, {vt: 8, value: 'Motor'}, {vt: 8195, value: "0,0"}] }'
   ```

   Output will be:

   ```bash
   waiting for service to become available...
   requester: making request: bcap_service_interfaces.srv.Bcap_Request(func_id=64, vnt_args=[bcap_service_interfaces.msg.Variant(vt=19, value='16'), bcap_service_interfaces.msg.Variant(vt=8, value='Motor'), bcap_service_interfaces.msg.Variant(vt=8195, value='0,0')])

   response:
   bcap_service_interfaces.srv.Bcap_Response(hresult=0, vnt_ret=bcap_service_interfaces.msg.Variant(vt=0, value=''))
   ```
