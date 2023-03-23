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

#include "denso_robot_control/denso_robot_control.hpp"

#include <chrono>
#include <cmath>
// #include <limits>
// #include <memory>
// #include <vector>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define RAD_2_DEG(x) ((x)*180.0 / M_PI)
#define DEG_2_RAD(x) ((x) / 180.0 * M_PI)
#define M_2_MM(x) ((x)*1000.0)
#define MM_2_M(x) ((x) / 1000.0)


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  HRESULT hr;

  auto node = rclcpp::Node::make_shared("denso_robot_control");

  std::string robot_name, ip_address;
  int joints, arm_group, send_format, recv_format, ctrl_type;
  std::vector<int> joints_type;
  std::vector<double> pos_interface, cmd_interface;

  node->declare_parameter("robot_name", "");
  node->declare_parameter("ip_address", "192.168.0.1");
  node->declare_parameter("controller_type", 8);
  node->declare_parameter("joints", 6);
  node->declare_parameter("jointsType");
  node->set_parameters({rclcpp::Parameter("jointsType", std::vector<int>({0}))});
  node->declare_parameter("armGroup", 0);
  node->declare_parameter("sendFormat", 0);
  node->declare_parameter("recvFormat", 2);
  node->declare_parameter("verbose", false);

  if(!node->get_parameter("robot_name", robot_name)) {
    robot_name = "";
  }
  if(!node->get_parameter("ip_address", ip_address)) {
    ip_address = "192.168.0.1";
  }
  if(!node->get_parameter("controller_type", ctrl_type)) {
    ctrl_type = 8;
  }
  if(!node->get_parameter("joints", joints)) {
    joints = 6;
  }
  if(!node->get_parameter("armGroup", arm_group)) {
    arm_group = 0;
  }
  if(!node->get_parameter("sendFormat", send_format)) {
    send_format = 0;
  }
  if(!node->get_parameter("recvFormat", recv_format)) {
    recv_format = 2;
  }

  joints_type.resize(joints);
  for (int i = 0; i < joints; i++) {
    joints_type[i] = 1;
  }

  denso_robot_control::DensoRobotControl drobo(
    "", "", robot_name, ip_address, ctrl_type, joints, joints_type,
    arm_group, send_format, recv_format, true);
  drobo.setNode(node);

  pos_interface.resize(joints, std::numeric_limits<double>::quiet_NaN());
  cmd_interface.resize(joints, std::numeric_limits<double>::quiet_NaN());
  hr = drobo.Initialize(pos_interface, cmd_interface);
  if (FAILED(hr)) {
    RCLCPP_FATAL(rclcpp::get_logger(node->get_name()), "Failed to initialize. (%X)", hr);
    return 1;
  } else {
    std::thread t(std::bind(&denso_robot_control::DensoRobotControl::Start, &drobo));
    drobo.Stop();
    t.join();
    return 0;
  }
}


namespace denso_robot_control
{
  DensoRobotControl::DensoRobotControl(
    const std::string& node_name, const std::string& node_namespace,
    const std::string& robot_name, const std::string& robot_ip_address, int ctrl_type,
    int robot_joints, std::vector<int> joint_type,
    int arm_group, int send_format, int recv_format, bool verbose)
  : node_name_(node_name), node_namespace_(node_namespace), robot_name_(robot_name),
    robot_ip_address_(robot_ip_address),
    robot_joints_(robot_joints), ctrl_type_(ctrl_type), //type_(jointType),
    arm_group_(arm_group), send_format_(send_format), recv_format_(recv_format), verbose_(verbose)
  {
    memset(type_, 0, sizeof(type_));
    for (int i = 0; i < robot_joints; i++) {
      type_[i] = joint_type[i];
    }
  }

  DensoRobotControl::~DensoRobotControl()
  {
  }

  HRESULT DensoRobotControl::Initialize(
    std::vector<double>& pos_interface, std::vector<double>& cmd_interface)
  {
    if (NULL == node_) {
      node_ = rclcpp::Node::make_shared(node_name_, node_namespace_);
    }

    joint_.resize(robot_joints_);
    memset(cmd_, 0, sizeof(cmd_));
    memset(pos_, 0, sizeof(pos_));
    memset(vel_, 0, sizeof(vel_));
    memset(eff_, 0, sizeof(eff_));

    if (verbose_) {
      RCLCPP_INFO(
        rclcpp::get_logger(node_->get_name()),
        "***** DENSO robot control node name: %s", node_name_.c_str());
      RCLCPP_INFO(
        rclcpp::get_logger(node_->get_name()),
        "***** DENSO robot control node namespace: %s", node_namespace_.c_str());
      RCLCPP_INFO(
        rclcpp::get_logger(node_->get_name()),
        "***** DENSO robot name: %s", robot_name_.c_str());
    }
    eng_ = std::make_shared<DensoRobotCore>(node_, robot_ip_address_, robot_name_, ctrl_type_);
    ctrl_.reset();
    rob_.reset();
    var_err_.reset();

    if (verbose_) {
      RCLCPP_INFO(
        rclcpp::get_logger(node_->get_name()), "[DEBUG] Initializing b-cap engine ...");
    }
    HRESULT hr = eng_->Initialize();
    if (FAILED(hr)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(node_->get_name()), "Failed to connect real controller. (%X)", hr);
      return hr;
    }

    if (verbose_) {
      RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "[DEBUG] Adding robot controller ...");
    }
    ctrl_ = eng_->get_Controller();

    if (verbose_) {
      RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "[DEBUG] Adding robot arm ...");
    }
    DensoRobot_Ptr pRob;
    hr = ctrl_->get_Robot(DensoBase::SRV_ACT, &pRob);
    if (FAILED(hr)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(node_->get_name()), "Failed to connect real robot. (%X)", hr);
      return hr;
    }

    rob_ = pRob;
    hr = CheckRobotType();
    if (FAILED(hr)) {
      RCLCPP_FATAL(rclcpp::get_logger(node_->get_name()), "Invalid robot type.");
      return hr;
    }
    rob_->ChangeArmGroup(arm_group_);

    hr = ctrl_->AddVariable("@ERROR_CODE");
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to add @ERROR_CODE object");
      return hr;
    }
    hr = ctrl_->get_Variable("@ERROR_CODE", &var_err_);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to get @ERROR_CODE object");
      return hr;
    }

    hr = ctrl_->ExecClearError();
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to clear error");
      return hr;
    }

    hr = ctrl_->ExecResetStoState();
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to reset STO");
      return hr;
    }

    hr = rob_->ExecCurJnt(joint_);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to get current joint");
      return hr;
    }

    for (int i = 0; i < robot_joints_; i++) {
      switch (type_[i]) {
        case 0:  // prismatic
          pos_[i] = MM_2_M(joint_[i]);
          break;
        case 1:  // revolute
          pos_[i] = DEG_2_RAD(joint_[i]);
          break;
        case -1:  // fixed
        default:
          pos_[i] = 0.0;
          break;
      }
      cmd_[i] = pos_[i];
      pos_interface[i] = pos_[i];
      cmd_interface[i] = pos_[i];
    }

    hr = rob_->AddVariable("@SERVO_ON");
    if (SUCCEEDED(hr)) {
      DensoVariable_Ptr p_var;
      hr = rob_->get_Variable("@SERVO_ON", &p_var);
      if (SUCCEEDED(hr)) {
        VARIANT_Ptr vnt_val(new VARIANT());
        vnt_val->vt = VT_BOOL;
        vnt_val->boolVal = VARIANT_TRUE;
        hr = p_var->ExecPutValue(vnt_val);
      }
    }
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to motor on");
      return hr;
    }

    int recv_format_type = recv_format_;
    switch (recv_format_ & DensoRobot::RECVFMT_POSE) {
      case DensoRobot::RECVFMT_POSE_J:
      case DensoRobot::RECVFMT_POSE_PJ:
      case DensoRobot::RECVFMT_POSE_TJ:
        break;
      case DensoRobot::RECVFMT_POSE_P:
        recv_format_type = DensoRobot::RECVFMT_POSE_PJ;
        break;
      case DensoRobot::RECVFMT_POSE_T:
        recv_format_type = DensoRobot::RECVFMT_POSE_TJ;
        break;
      default:
        recv_format_type = DensoRobot::RECVFMT_POSE_J;
    }
    if (recv_format_type != recv_format_) {
      recv_format_ = ((recv_format_ & ~DensoRobot::RECVFMT_POSE) | recv_format_type);
      RCLCPP_WARN(
        rclcpp::get_logger(node_->get_name()),
        "Changed recv_format to %d to contain joint.", recv_format_);
    }
    rob_->set_SendFormat(send_format_);
    rob_->set_RecvFormat(recv_format_);

    change_mode_srv_ = node_->create_service<denso_robot_core_interfaces::srv::ChangeMode>(
      "ChangeMode",
      std::bind(&DensoRobotControl::ChangeModeFunction, this, std::placeholders::_1, std::placeholders::_2));

    pub_cur_mode_ = node_->create_publisher<std_msgs::msg::Int32>("CurMode", 1);

    if (verbose_) {
      RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "[DEBUG] Changing to slave mode ...");
    }
    hr = ChangeModeWithClearError(DensoRobot::SLVMODE_SYNC_WAIT | DensoRobot::SLVMODE_POSE_J);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to change to slave mode");
      return hr;
    }
    if (verbose_) {
      RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "[DEBUG] Changed to slave mode ...");
    }

    return S_OK;
  }

  HRESULT DensoRobotControl::ChangeModeWithClearError(int mode)
  {
    HRESULT hr = eng_->ChangeMode(mode, mode == DensoRobot::SLVMODE_NONE);
    if (FAILED(hr)) {
      // Clear Error
      HRESULT hres = ctrl_->ExecClearError();
      if (FAILED(hres)) {
        printErrorDescription(hres, "Failed to clear error");
      }
    }

    std_msgs::msg::Int32 msg;
    msg.data = eng_->get_Mode();
    pub_cur_mode_->publish(msg);

    if (msg.data == DensoRobot::SLVMODE_NONE) {
      sub_mini_io_.reset();
      sub_hand_io_.reset();
      sub_send_user_io_.reset();
      sub_recv_user_io_.reset();
      pub_mini_io_.reset();
      pub_hand_io_.reset();
      pub_recv_user_io_.reset();
      pub_current_.reset();
    } else {
      if (send_format_ & DensoRobot::SENDFMT_HANDIO) {
        sub_hand_io_ = node_->create_subscription<std_msgs::msg::UInt32>(
          "Write_HandIO", 1,
          std::bind(&DensoRobotControl::Callback_HandIO, this, std::placeholders::_1));
      }
      if (send_format_ & DensoRobot::SENDFMT_MINIIO) {
        sub_mini_io_ = node_->create_subscription<std_msgs::msg::UInt32>(
          "Write_MiniIO", 1,
          std::bind(&DensoRobotControl::Callback_MiniIO, this, std::placeholders::_1));
      }
      if (send_format_ & DensoRobot::SENDFMT_USERIO) {
        sub_send_user_io_ = node_->create_subscription<denso_robot_core_interfaces::msg::UserIO>(
          "Write_SendUserIO", 1,
          std::bind(&DensoRobotControl::Callback_SendUserIO, this, std::placeholders::_1));
      }
      if (recv_format_ & DensoRobot::RECVFMT_HANDIO) {
        pub_hand_io_ = node_->create_publisher<std_msgs::msg::UInt32>("Read_HandIO", 1);
      }
      if (recv_format_ & DensoRobot::RECVFMT_CURRENT) {
        pub_current_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
          "Read_Current", 1);
      }
      if (recv_format_ & DensoRobot::RECVFMT_MINIIO) {
        pub_mini_io_ = node_->create_publisher<std_msgs::msg::UInt32>("Read_MiniIO", 1);
      }
      if (recv_format_ & DensoRobot::RECVFMT_USERIO) {
        sub_recv_user_io_ = node_->create_subscription<denso_robot_core_interfaces::msg::UserIO>(
          "Write_RecvUserIO", 1,
          std::bind(&DensoRobotControl::Callback_RecvUserIO, this, std::placeholders::_1));
        pub_recv_user_io_ = node_->create_publisher<denso_robot_core_interfaces::msg::UserIO>(
          "Read_RecvUserIO", 1);
      }
    }

    return hr;
  }

  bool DensoRobotControl::ChangeModeFunction(
    const std::shared_ptr<denso_robot_core_interfaces::srv::ChangeMode::Request> request,
    std::shared_ptr<denso_robot_core_interfaces::srv::ChangeMode::Response> response)
  {
    std::unique_lock<std::mutex> lock_mode(mtx_mode_);

    RCLCPP_INFO(rclcpp::get_logger(node_->get_name()), "Change to mode %d.", request->mode);
    HRESULT hr = ChangeModeWithClearError(request->mode);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to change mode");
    }
    response->hresult = hr;
    return true;
  }

  HRESULT DensoRobotControl::CheckRobotType()
  {
    DensoVariable_Ptr p_var;
    VARIANT_Ptr vnt_val(new VARIANT());
    std::string type_name = "@TYPE_NAME";

    HRESULT hr = rob_->AddVariable(type_name);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to add @TYPE_NAME");
      return hr;
    }
    rob_->get_Variable(type_name, &p_var);
    hr = p_var->ExecGetValue(vnt_val);
    if (FAILED(hr)) {
      printErrorDescription(hr, "Failed to get @TYPE_NAME");
      return hr;
    }
    type_name = DensoBase::ConvertBSTRToString(vnt_val->bstrVal);
    RCLCPP_INFO(
      rclcpp::get_logger(node_->get_name()),
      "***** Full robot name: %s", type_name.c_str());
    if (
    strncmp(
      robot_name_.c_str(), type_name.c_str(),
      (robot_name_.length() < type_name.length()) ? robot_name_.length() : type_name.length()))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(node_->get_name()), "Expected robot type is %s , real robot type is %s",
        robot_name_.c_str(), type_name.c_str());
      return E_FAIL;
    }

    return 0;
  }

  void DensoRobotControl::Callback_MiniIO(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    rob_->put_MiniIO(msg->data);
  }

  void DensoRobotControl::Callback_HandIO(const std_msgs::msg::UInt32::SharedPtr msg)
  {
    rob_->put_HandIO(msg->data);
  }

  void DensoRobotControl::Callback_SendUserIO(const denso_robot_core_interfaces::msg::UserIO::SharedPtr msg)
  {
    rob_->put_SendUserIO(msg);
  }

  void DensoRobotControl::Callback_RecvUserIO(const denso_robot_core_interfaces::msg::UserIO::SharedPtr msg)
  {
    rob_->put_RecvUserIO(msg);
  }

  bool DensoRobotControl::isSlaveSyncMode() const
  {
    if (eng_->get_Mode() & DensoRobot::SLVMODE_SYNC_WAIT) {
      return true;
    }
    return false;
  }

  bool DensoRobotControl::hasError()
  {
    HRESULT hr;
    VARIANT_Ptr vnt_val(new VARIANT());
    hr = var_err_->ExecGetValue(vnt_val);
    if (SUCCEEDED(hr) && (vnt_val->lVal == 0)) {
      return false;
    }
    return true;
  }

  void DensoRobotControl::printErrorDescription(
    HRESULT error_code, const std::string& error_message)
  {
    HRESULT hr;
    std::string error_description;
    if (eng_->get_Mode() == DensoRobot::SLVMODE_NONE) {
      hr = ctrl_->ExecGetErrorDescription(error_code, error_description);
      if (SUCCEEDED(hr)) {
        RCLCPP_FATAL(
          rclcpp::get_logger(node_->get_name()), "%s: %s (%X)",
          error_message.c_str(), error_description.c_str(), error_code);
        return;
      }
    }
    RCLCPP_FATAL(
      rclcpp::get_logger(node_->get_name()), "%s (%X)", error_message.c_str(), error_code);
  }

  void DensoRobotControl::read(std::vector<double>& pos_interface)
  {
    std::unique_lock<std::mutex> lock_mode(mtx_mode_);

    // read robot current position
    if (eng_->get_Mode() == DensoRobot::SLVMODE_NONE) {
      HRESULT hr = rob_->ExecCurJnt(joint_);
      if (FAILED(hr)) {
        RCLCPP_FATAL(rclcpp::get_logger("DensoRobotHW"), "Failed to get current joint. (%X)", hr);
      }
    }

    // get robot joints value (conversion based on joint type, e.g. prismatic or revolute)
    for (int i = 0; i < robot_joints_; i++) {
      switch (type_[i]) {
        case 0:  // prismatic
          pos_[i] = MM_2_M(joint_[i]);
          break;
        case 1:  // revolute
          pos_[i] = DEG_2_RAD(joint_[i]);
          break;
        case -1:  // fixed
        default:
          pos_[i] = 0.0;
          break;
      }
      pos_interface[i] = pos_[i];
    }

    return;
  }

  void DensoRobotControl::write(std::vector<double>& cmd_interface)
  {
    std::unique_lock<std::mutex> lock_mode(mtx_mode_);
    if (eng_->get_Mode() != DensoRobot::SLVMODE_NONE) {
      std::vector<double> pose;
      pose.resize(JOINT_MAX);
      int bits = 0x0000;
      for (int i = 0; i < robot_joints_; i++) {
        cmd_[i] = cmd_interface[i];
        switch (type_[i]) {
          case 0:  // prismatic
            pose[i] = M_2_MM(cmd_[i]);
            break;
          case 1:  // revolute
            pose[i] = RAD_2_DEG(cmd_[i]);
            break;
          case -1:  // fixed
          default:
          pose[i] = 0.0;
          break;
        }
        bits |= (1 << i);
      }
      // TODO: what is the purpose of this "push_back" function call ?
      // why "0x400000 | bits" ?
      pose.push_back(0x400000 | bits);

      HRESULT hr = rob_->ExecSlaveMove(pose, joint_);
      if (SUCCEEDED(hr)) {
        if (recv_format_ & DensoRobot::RECVFMT_HANDIO) {
          std_msgs::msg::UInt32 msg;
          msg.data = rob_->get_HandIO();
          pub_hand_io_->publish(msg);
        }
        if (recv_format_ & DensoRobot::RECVFMT_CURRENT) {
          std_msgs::msg::Float64MultiArray msg;
          rob_->get_Current(msg.data);
          pub_current_->publish(msg);
        }
        if (recv_format_ & DensoRobot::RECVFMT_MINIIO) {
          std_msgs::msg::UInt32 msg;
          msg.data = rob_->get_MiniIO();
          pub_mini_io_->publish(msg);
        }
        if (recv_format_ & DensoRobot::RECVFMT_USERIO) {
          denso_robot_core_interfaces::msg::UserIO::SharedPtr msg(new denso_robot_core_interfaces::msg::UserIO());
          rob_->get_RecvUserIO(msg);
          pub_recv_user_io_->publish(*msg);
        }
      } else if (FAILED(hr) && (hr != DensoRobot::E_BUF_FULL)) {
        int error_count = 0;

        printErrorDescription(hr, "Failed to write");
        if (!hasError()) {
          return;
        }
        RCLCPP_FATAL(
          rclcpp::get_logger(node_->get_name()), "Automatically change to normal mode.");
        ChangeModeWithClearError(DensoRobot::SLVMODE_NONE);

        hr = ctrl_->ExecGetCurErrorCount(error_count);
        if (SUCCEEDED(hr)) {
          for (int i = error_count - 1; 0 <= i; i--) {
            HRESULT error_code;
            std::string error_message;
            hr = ctrl_->ExecGetCurErrorInfo(i, error_code, error_message);
            if (FAILED(hr)) {
              return;
            }
            RCLCPP_FATAL(
              rclcpp::get_logger(node_->get_name()), "  [%d] %s (%X)", i + 1, error_message.c_str(), error_code);
          }
        }
      }
    } else {
      for (int i = 0; i < robot_joints_; i++) {
        cmd_interface[i] = pos_[i];
      }
    }

    return;
  }

  void DensoRobotControl::Update()
  {
    ctrl_->Update();
  }

  void DensoRobotControl::Start()
  {
    RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "***** Starting DENSO robot_core thread ...");
    eng_->Start();
  }

  void DensoRobotControl::Stop()
  {
    eng_->Stop();
  }

}  // namespace denso_robot_control
