/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef DENSO_ROBOT_CONTROL__DENSO_ROBOT_CONTROL_HPP_
#define DENSO_ROBOT_CONTROL__DENSO_ROBOT_CONTROL_HPP_

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include <boost/thread.hpp>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"
// ROS
#include "rclcpp/macros.hpp"
// Message (std_msgs)
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// DENSO libraries
#include "denso_robot_core/denso_robot_core.h"
#include "denso_robot_core/denso_controller.h"
#include "denso_robot_core/denso_robot.h"
#include "denso_robot_core/denso_variable.h"
#include "denso_robot_core_interfaces/msg/user_io.hpp"
#include "denso_robot_core_interfaces/srv/change_mode.hpp"

using namespace denso_robot_core;
using namespace std_msgs;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::status;

#define JOINT_MAX (8)


namespace denso_robot_control {

class DensoRobotControl
{
public:
  DensoRobotControl(
    const std::string& node_name, const std::string& node_namespace, const std::string& robot_name,
    const std::string& robot_ip_address, int ctrl_type, int robot_joints, std::vector<int> joint_type,
    int arm_group, int send_format, int recv_format, bool verbose);
  virtual ~DensoRobotControl();

  HRESULT Initialize(std::vector<double>& pos_interface, std::vector<double>& cmd_interface);

  rclcpp::Time getTime() const
  {
    return rclcpp::Clock().now();
  }

  rclcpp::Duration getPeriod() const
  {
    return ctrl_->get_Duration();
  }

  void setNode(rclcpp::Node::SharedPtr& node)
  {
    node_ = node;
  }

  void read(std::vector<double>& pos_interface);
  void write(std::vector<double>& cmd_interface);
  void Start();
  void Stop();
  void Update();

  bool isSlaveSyncMode() const;


private:

  // Store the commands for the real robot
  double cmd_[JOINT_MAX];
  double pos_[JOINT_MAX];
  double vel_[JOINT_MAX];
  double eff_[JOINT_MAX];
  int type_[JOINT_MAX];
  std::vector<double> joint_;

  std::string node_name_;
  std::string node_namespace_;
  std::string robot_name_;
  std::string robot_ip_address_;
  int robot_joints_;
  int ctrl_type_;
  int arm_group_;
  int send_format_;
  int recv_format_;
  bool verbose_;

  HRESULT ChangeModeWithClearError(int mode);

  HRESULT CheckRobotType();

  void Callback_MiniIO(const std_msgs::msg::UInt32::SharedPtr msg);
  void Callback_HandIO(const std_msgs::msg::UInt32::SharedPtr msg);
  void Callback_SendUserIO(const denso_robot_core_interfaces::msg::UserIO::SharedPtr msg);
  void Callback_RecvUserIO(const denso_robot_core_interfaces::msg::UserIO::SharedPtr msg);

  bool ChangeModeFunction(
    const std::shared_ptr<denso_robot_core_interfaces::srv::ChangeMode::Request> request,
    std::shared_ptr<denso_robot_core_interfaces::srv::ChangeMode::Response> response);

  bool hasError();
  void printErrorDescription(HRESULT error_code, const std::string& error_message);

  DensoRobotCore_Ptr eng_;
  DensoController_Ptr ctrl_;
  DensoRobot_Ptr rob_;
  DensoVariable_Ptr var_err_;

  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_mini_io_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_hand_io_;
  rclcpp::Subscription<denso_robot_core_interfaces::msg::UserIO>::SharedPtr sub_send_user_io_;
  rclcpp::Subscription<denso_robot_core_interfaces::msg::UserIO>::SharedPtr sub_recv_user_io_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cur_mode_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_mini_io_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_hand_io_;
  rclcpp::Publisher<denso_robot_core_interfaces::msg::UserIO>::SharedPtr pub_recv_user_io_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_current_;

  // ChangeMode Service
  rclcpp::Service<denso_robot_core_interfaces::srv::ChangeMode>::SharedPtr change_mode_srv_;

  std::mutex mtx_mode_;

  // ROS2 Node Handle
  rclcpp::Node::SharedPtr node_;

};

typedef std::shared_ptr<DensoRobotControl> DensoRobotControl_Ptr;

}  // namespace denso_robot_control

#endif  // DENSO_ROBOT_CONTROL__DENSO_ROBOT_CONTROL_HPP_
