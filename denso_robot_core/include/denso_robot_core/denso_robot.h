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

#ifndef DENSO_ROBOT_H
#define DENSO_ROBOT_H

#include "denso_robot_core/denso_base.h"
#include "denso_robot_core/denso_variable.h"

#include "denso_robot_core_interfaces/action/move_string.hpp"
#include "denso_robot_core_interfaces/action/move_value.hpp"
#include "denso_robot_core_interfaces/action/drive_string.hpp"
#include "denso_robot_core_interfaces/action/drive_value.hpp"


using MoveString = denso_robot_core_interfaces::action::MoveString;
using MoveValue = denso_robot_core_interfaces::action::MoveValue;
using DriveString = denso_robot_core_interfaces::action::DriveString;
using DriveValue = denso_robot_core_interfaces::action::DriveValue;
using GoalHandleMoveString = rclcpp_action::ServerGoalHandle<MoveString>;
using GoalHandleMoveValue = rclcpp_action::ServerGoalHandle<MoveValue>;
using GoalHandleDriveString = rclcpp_action::ServerGoalHandle<DriveString>;
using GoalHandleDriveValue = rclcpp_action::ServerGoalHandle<DriveValue>;
using UserIO = denso_robot_core_interfaces::msg::UserIO;


namespace denso_robot_core {
class DensoRobot : public DensoBase
{
  friend class DensoRobotCore;

public:
  static constexpr HRESULT S_BUF_FULL = 0x0F200501;
  static constexpr HRESULT E_BUF_FULL = 0x83201483;
  static constexpr int BCAP_ROBOT_EXECUTE_ARGS = 3;
  static constexpr int BCAP_ROBOT_HALT_ARGS = 2;
  static constexpr int BCAP_ROBOT_MOVE_ARGS = 4;
  static constexpr int BCAP_ROBOT_SPEED_ARGS = 3;
  static constexpr int BCAP_ROBOT_CHANGE_ARGS = 2;
  static constexpr const char* XML_ROBOT_NAME = "Robot";
  static constexpr const char* NAME_ARMGROUP = "_armgroup";
  static constexpr const char* NAME_MOVESTRING = "_MoveString";
  static constexpr const char* NAME_MOVEVALUE = "_MoveValue";
  static constexpr const char* NAME_DRIVEEXSTRING = "_DriveExString";
  static constexpr const char* NAME_DRIVEEXVALUE = "_DriveExValue";
  static constexpr const char* NAME_DRIVEAEXSTRING = "_DriveAExString";
  static constexpr const char* NAME_DRIVEAEXVALUE = "_DriveAExValue";
  static constexpr const char* NAME_SPEED = "_Speed";
  static constexpr const char* NAME_CHANGETOOL = "_ChangeTool";
  static constexpr const char* NAME_CHANGEWORK = "_ChangeWork";
  enum
  {
    SLVMODE_NONE = 0,
    SLVMODE_POSE_P = 0x0001,
    SLVMODE_POSE_J = 0x0002,
    SLVMODE_POSE_T = 0x0003,
    SLVMODE_POSE = 0x000F,
    SLVMODE_ASYNC = 0x0100,
    SLVMODE_SYNC_WAIT = 0x0200,
  };

  enum
  {
    SENDFMT_NONE = 0,
    SENDFMT_HANDIO = 0x0020,
    SENDFMT_MINIIO = 0x0100,
    SENDFMT_USERIO = 0x0200,
  };

  enum
  {
    RECVFMT_NONE = 0,
    RECVFMT_POSE_P = 0x0001,
    RECVFMT_POSE_J = 0x0002,
    RECVFMT_POSE_T = 0x0003,
    RECVFMT_POSE_PJ = 0x0004,
    RECVFMT_POSE_TJ = 0x0005,
    RECVFMT_POSE = 0x000F,
    RECVFMT_TIME = 0x0010,
    RECVFMT_HANDIO = 0x0020,
    RECVFMT_CURRENT = 0x0040,
    RECVFMT_MINIIO = 0x0100,
    RECVFMT_USERIO = 0x0200,
  };

  enum
  {
    TSFMT_MILLISEC = 0,
    TSFMT_MICROSEC = 1,
  };

  enum
  {
    SLVMODE_TIMEOUT_SYNC = 16,
    SLVMODE_TIMEOUT_ASYNC = 8,
  };

public:
  virtual ~DensoRobot();
  virtual HRESULT InitializeBCAP(XMLElement * xmlElem);
  HRESULT StartService(rclcpp::Node::SharedPtr& node);
  HRESULT StopService();
  bool Update();
  HRESULT get_Variable(const std::string& name, DensoVariable_Ptr * var);
  HRESULT AddVariable(const std::string& name);
  HRESULT ExecTakeArm();
  HRESULT ExecGiveArm();
  void ChangeArmGroup(int number);
  HRESULT ExecMove(int comp, const VARIANT_Ptr& pose, const std::string& option);
  HRESULT ExecDrive(const std::string& name, const VARIANT_Ptr& option);
  HRESULT ExecSpeed(float value);
  HRESULT ExecChange(const std::string& value);
  HRESULT ExecHalt();
  HRESULT ExecCurJnt(std::vector<double>& pose);
  HRESULT ExecSlaveMove(const std::vector<double>& pose, std::vector<double>& joint);
  int get_SendFormat() const;
  void put_SendFormat(int format); /* deprecated */
  int get_RecvFormat() const;
  void put_RecvFormat(int format); /* deprecated */
  int get_TimeFormat() const;
  void put_TimeFormat(int format);
  void set_SendFormat(int format);
  void set_RecvFormat(int format);
  unsigned int get_MiniIO() const;
  void put_MiniIO(unsigned int value);
  unsigned int get_HandIO() const;
  void put_HandIO(unsigned int value);
  void put_SendUserIO(const denso_robot_core_interfaces::msg::UserIO::SharedPtr value);
  void put_RecvUserIO(const denso_robot_core_interfaces::msg::UserIO::SharedPtr value);
  void get_RecvUserIO(denso_robot_core_interfaces::msg::UserIO::SharedPtr value) const;
  void get_Current(std::vector<double>& current) const;
  int get_Timestamp() const;

protected:
  DensoRobot(
    rclcpp::Node::SharedPtr& node, DensoBase * parent,
    Service_Vec& service, Handle_Vec& handle, const std::string& name, const int * mode);
  void Callback_ArmGroup(const std_msgs::msg::Int32::SharedPtr msg);
  virtual HRESULT AddVariable(XMLElement * xmlElem);
  HRESULT CreatePoseData(const denso_robot_core_interfaces::msg::PoseData pose, VARIANT& vnt);
  HRESULT CreateExJoints(const denso_robot_core_interfaces::msg::ExJoints exjoints, VARIANT& vnt);

private:
  virtual HRESULT ChangeMode(int mode);
  HRESULT ExecSlaveMode(const std::string& name, int32_t format, int32_t option = 0);
  HRESULT CreateSendParameter(
    const std::vector<double>& pose, VARIANT_Ptr& send, const int miniio = 0,
    const int handio = 0, const int recv_userio_offset = 0, const int recv_userio_size = 0,
    const int send_userio_offset = 0, const int send_userio_size = 0,
    const std::vector<uint8_t>& send_userio = std::vector<uint8_t>());
  HRESULT ParseRecvParameter(
    const VARIANT_Ptr& recv, std::vector<double>& position, std::vector<double>& joint,
    std::vector<double>& trans, int& miniio, int& handio, int& timestamp,
    std::vector<uint8_t>& recv_userio, std::vector<double>& current);

private:
  rclcpp_action::GoalResponse Callback_MoveStringHandleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveString::Goal> goal);
  rclcpp_action::CancelResponse Callback_MoveStringHandleCancel(
    const std::shared_ptr<GoalHandleMoveString> goal_handle);
  void Callback_MoveStringHandleAccepted(const std::shared_ptr<GoalHandleMoveString> goal_handle);
  void Callback_MoveString(const std::shared_ptr<GoalHandleMoveString> goal_handle);
  rclcpp_action::GoalResponse Callback_MoveValueHandleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveValue::Goal> goal);
  rclcpp_action::CancelResponse Callback_MoveValueHandleCancel(
    const std::shared_ptr<GoalHandleMoveValue> goal_handle);
  void Callback_MoveValueHandleAccepted(
    const std::shared_ptr<GoalHandleMoveValue> goal_handle);
  void Callback_MoveValue(
    const std::shared_ptr<GoalHandleMoveValue> goal_handle);
  rclcpp_action::GoalResponse Callback_DriveStringHandleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DriveString::Goal> goal);
  rclcpp_action::CancelResponse Callback_DriveStringHandleCancel(
    const std::shared_ptr<GoalHandleDriveString> goal_handle);
  void Callback_DriveStringHandleAccepted(
    const std::string& name, const std::shared_ptr<GoalHandleDriveString> goal_handle);
  void Callback_DriveString(
    const std::string& name, const std::shared_ptr<GoalHandleDriveString> goal_handle);
  void Callback_DriveExStringHandleAccepted(
    const std::shared_ptr<GoalHandleDriveString> goal_handle);
  void Callback_DriveExString(const std::shared_ptr<GoalHandleDriveString> goal_handle);
  void Callback_DriveAExStringHandleAccepted(
    const std::shared_ptr<GoalHandleDriveString> goal_handle);
  void Callback_DriveAExString(const std::shared_ptr<GoalHandleDriveString> goal_handle);
  rclcpp_action::GoalResponse Callback_DriveValueHandleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DriveValue::Goal> goal);
  rclcpp_action::CancelResponse Callback_DriveValueHandleCancel(
    const std::shared_ptr<GoalHandleDriveValue> goal_handle);
  void Callback_DriveValueHandleAccepted(
    const std::string& name, const std::shared_ptr<GoalHandleDriveValue> goal_handle);
  void Callback_DriveValue(
    const std::string& name, const std::shared_ptr<GoalHandleDriveValue> goal_handle);
  void Callback_DriveExValueHandleAccepted(
    const std::shared_ptr<GoalHandleDriveValue> goal_handle);
  void Callback_DriveExValue(const std::shared_ptr<GoalHandleDriveValue> goal_handle);
  void Callback_DriveAExValueHandleAccepted(
    const std::shared_ptr<GoalHandleDriveValue> goal_handle);
  void Callback_DriveAExValue(const std::shared_ptr<GoalHandleDriveValue> goal_handle);
  void Callback_Speed(const std_msgs::msg::Float32::SharedPtr msg);
  void Callback_Change(const std::string& name, const std_msgs::msg::Int32::SharedPtr msg);
  void Callback_ChangeTool(const std_msgs::msg::Int32::SharedPtr msg);
  void Callback_ChangeWork(const std_msgs::msg::Int32::SharedPtr msg);
  void Callback_Cancel();
  void Action_Feedback();

protected:
  DensoVariable_Vec m_vecVar;
  int32_t m_ArmGroup;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subArmGroup;

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subSpeed;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subChangeTool;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subChangeWork;
  rclcpp_action::Server<MoveString>::SharedPtr m_actMoveString;
  rclcpp_action::Server<MoveValue>::SharedPtr m_actMoveValue;
  rclcpp_action::Server<DriveString>::SharedPtr m_actDriveExString;
  rclcpp_action::Server<DriveValue>::SharedPtr m_actDriveExValue;
  rclcpp_action::Server<DriveString>::SharedPtr m_actDriveAExString;
  rclcpp_action::Server<DriveValue>::SharedPtr m_actDriveAExValue;
  std::shared_ptr<GoalHandleMoveString> m_actMoveStringGoalHandle;
  std::shared_ptr<GoalHandleMoveValue> m_actMoveValueGoalHandle;
  std::shared_ptr<GoalHandleDriveString> m_actDriveExStringGoalHandle;
  std::shared_ptr<GoalHandleDriveValue> m_actDriveExValueGoalHandle;
  std::shared_ptr<GoalHandleDriveString> m_actDriveAExStringGoalHandle;
  std::shared_ptr<GoalHandleDriveValue> m_actDriveAExValueGoalHandle;
  int m_curAct;
  std::mutex m_mtxAct;
  uint32_t m_memTimeout;
  unsigned int m_memRetry;
  int m_tsfmt, m_timestamp;
  int m_sendfmt, m_send_miniio, m_send_handio;
  int m_recvfmt, m_recv_miniio, m_recv_handio;
  int m_send_userio_offset, m_send_userio_size;
  int m_recv_userio_offset, m_recv_userio_size;
  std::vector<uint8_t> m_send_userio, m_recv_userio;
  std::vector<double> m_position, m_joint, m_trans, m_current;
  // ROS2 Node Handle
  rclcpp::Node::SharedPtr& m_node;
};

typedef std::shared_ptr<DensoRobot> DensoRobot_Ptr;
typedef std::vector<DensoRobot_Ptr> DensoRobot_Vec;

}  // namespace denso_robot_core

#endif  // DENSO_ROBOT_H
