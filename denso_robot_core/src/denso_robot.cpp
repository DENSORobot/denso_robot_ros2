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

#include "denso_robot_core/denso_robot.h"

namespace denso_robot_core
{
enum
{
  NUM_POSITION = 7,
  NUM_JOINT = 8,
  NUM_TRANS = 10
};

enum
{
  ACT_RESET = -1,
  ACT_NONE = 0,
  ACT_MOVESTRING,
  ACT_MOVEVALUE,
  ACT_DRIVEEXSTRING,
  ACT_DRIVEEXVALUE,
  ACT_DRIVEAEXSTRING,
  ACT_DRIVEAEXVALUE,
};

DensoRobot::DensoRobot(
  rclcpp::Node::SharedPtr& node, DensoBase* parent, Service_Vec& service, Handle_Vec& handle,
  const std::string& name, const int* mode)
: DensoBase(parent, service, handle, name, mode), m_ArmGroup(0), m_curAct(ACT_RESET),
  m_memTimeout(0), m_memRetry(0), m_tsfmt(0), m_timestamp(0), m_sendfmt(0), m_send_miniio(0),
  m_send_handio(0), m_recvfmt(0), m_recv_miniio(0), m_recv_handio(0),
  m_send_userio_offset(UserIO::MIN_USERIO_OFFSET), m_send_userio_size(1),
  m_recv_userio_offset(UserIO::MIN_USERIO_OFFSET), m_recv_userio_size(1),
  m_node(node)
{
  m_tsfmt = TSFMT_MILLISEC;

  m_sendfmt = SENDFMT_MINIIO | SENDFMT_HANDIO;

  m_recvfmt = RECVFMT_POSE_PJ | RECVFMT_MINIIO | RECVFMT_HANDIO;
}

DensoRobot::~DensoRobot()
{
}

HRESULT DensoRobot::InitializeBCAP(XMLElement * xmlElem)
{
  return AddVariable(xmlElem);
}

HRESULT DensoRobot::StartService(rclcpp::Node::SharedPtr& node)
{
  std::string tmpName = DensoBase::RosName();

  if (*m_mode == 0) {

    m_subSpeed = node->create_subscription<std_msgs::msg::Float32>(
      tmpName + NAME_SPEED, MESSAGE_QUEUE, std::bind(&DensoRobot::Callback_Speed, this, _1));

    m_subChangeTool = node->create_subscription<std_msgs::msg::Int32>(
      tmpName + NAME_CHANGETOOL, MESSAGE_QUEUE,
      std::bind(&DensoRobot::Callback_ChangeTool, this, _1));

    m_subChangeWork = node->create_subscription<std_msgs::msg::Int32>(
      tmpName + NAME_CHANGEWORK, MESSAGE_QUEUE,
      std::bind(&DensoRobot::Callback_ChangeWork, this, _1));

    m_actMoveString = rclcpp_action::create_server<MoveString>(
      node,
      DensoBase::RosName() + NAME_MOVESTRING,
      std::bind(&DensoRobot::Callback_MoveStringHandleGoal, this, _1, _2),
      std::bind(&DensoRobot::Callback_MoveStringHandleCancel, this, _1),
      std::bind(&DensoRobot::Callback_MoveStringHandleAccepted, this, _1));

    m_actMoveValue = rclcpp_action::create_server<MoveValue>(
      node,
      DensoBase::RosName() + NAME_MOVEVALUE,
      std::bind(&DensoRobot::Callback_MoveValueHandleGoal, this, _1, _2),
      std::bind(&DensoRobot::Callback_MoveValueHandleCancel, this, _1),
      std::bind(&DensoRobot::Callback_MoveValueHandleAccepted, this, _1));

    m_actDriveExString = rclcpp_action::create_server<DriveString>(
      node,
      DensoBase::RosName() + NAME_DRIVEEXSTRING,
      std::bind(&DensoRobot::Callback_DriveStringHandleGoal, this, _1, _2),
      std::bind(&DensoRobot::Callback_DriveStringHandleCancel, this, _1),
      std::bind(&DensoRobot::Callback_DriveStringHandleAccepted, this, "DriveEx", _1));

    m_actDriveExValue = rclcpp_action::create_server<DriveValue>(
      node,
      DensoBase::RosName() + NAME_DRIVEEXVALUE,
      std::bind(&DensoRobot::Callback_DriveValueHandleGoal, this, _1, _2),
      std::bind(&DensoRobot::Callback_DriveValueHandleCancel, this, _1),
      std::bind(&DensoRobot::Callback_DriveValueHandleAccepted, this, "DriveEx", _1));

    m_actDriveAExString = rclcpp_action::create_server<DriveString>(
      node,
      DensoBase::RosName() + NAME_DRIVEAEXSTRING,
      std::bind(&DensoRobot::Callback_DriveStringHandleGoal, this, _1, _2),
      std::bind(&DensoRobot::Callback_DriveStringHandleCancel, this, _1),
      std::bind(&DensoRobot::Callback_DriveStringHandleAccepted, this, "DriveAEx", _1));

    m_actDriveAExValue = rclcpp_action::create_server<DriveValue>(
      node,
      DensoBase::RosName() + NAME_DRIVEAEXVALUE,
      std::bind(&DensoRobot::Callback_DriveValueHandleGoal, this, _1, _2),
      std::bind(&DensoRobot::Callback_DriveValueHandleCancel, this, _1),
      std::bind(&DensoRobot::Callback_DriveValueHandleAccepted, this, "DriveAEx", _1));

    m_subArmGroup = node->create_subscription<std_msgs::msg::Int32>(
      tmpName + NAME_ARMGROUP, MESSAGE_QUEUE,
      std::bind(&DensoRobot::Callback_ArmGroup, this, _1));
  }

  DensoVariable_Vec::iterator itVar;
  for (itVar = m_vecVar.begin(); itVar != m_vecVar.end(); itVar++) {
    (*itVar)->StartService(node);
  }

  m_serving = true;

  m_curAct = ACT_NONE;

  return S_OK;
}

HRESULT DensoRobot::StopService()
{
  m_mtxSrv.lock();
  m_serving = false;
  m_mtxSrv.unlock();

  m_subArmGroup.reset();

  DensoVariable_Vec::iterator itVar;
  for (itVar = m_vecVar.begin(); itVar != m_vecVar.end(); itVar++) {
    (*itVar)->StopService();
  }

  m_mtxAct.lock();
  m_curAct = ACT_RESET;
  m_mtxAct.unlock();

  m_subSpeed.reset();
  m_subChangeTool.reset();
  m_subChangeWork.reset();
  m_actMoveString.reset();
  m_actMoveValue.reset();
  m_actDriveExString.reset();
  m_actDriveExValue.reset();
  m_actDriveAExString.reset();
  m_actDriveAExValue.reset();

  return S_OK;
}

bool DensoRobot::Update()
{
  std::unique_lock<std::mutex> lockSrv(m_mtxSrv);
  if (!m_serving) {
    return false;
  }

  DensoVariable_Vec::iterator itVar;
  for (itVar = m_vecVar.begin(); itVar != m_vecVar.end(); itVar++) {
    (*itVar)->Update();
  }

  Action_Feedback();

  return true;
}

HRESULT DensoRobot::ExecTakeArm()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());
  int32_t * pval;

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"TakeArm");
        break;
      case 2:
        vntTmp->vt = (VT_ARRAY | VT_I4);
        vntTmp->parray = SafeArrayCreateVector(VT_I4, 0, 2);
        SafeArrayAccessData(vntTmp->parray, (void**)&pval);
        pval[0] = m_ArmGroup;  // Arm group
        pval[1] = 1L;          // Keep
        SafeArrayUnaccessData(vntTmp->parray);
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

HRESULT DensoRobot::ExecGiveArm()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"GiveArm");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

void DensoRobot::ChangeArmGroup(int number)
{
  m_ArmGroup = number;
}

void DensoRobot::Callback_ArmGroup(const std_msgs::msg::Int32::SharedPtr msg)
{
  ChangeArmGroup(msg->data);
}

HRESULT DensoRobot::get_Variable(const std::string& name, DensoVariable_Ptr* var)
{
  if (var == NULL) {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), m_vecVar.begin(), m_vecVar.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, name, &pBase);
  if (SUCCEEDED(hr)) {
    *var = std::dynamic_pointer_cast<DensoVariable>(pBase);
  }

  return hr;
}

HRESULT DensoRobot::AddVariable(const std::string& name)
{
  return DensoBase::AddVariable(ID_ROBOT_GETVARIABLE, name, m_vecVar);
}

HRESULT DensoRobot::AddVariable(XMLElement* xmlElem)
{
  HRESULT hr = S_OK;
  XMLElement* xmlVar;

  for (
    xmlVar = xmlElem->FirstChildElement(DensoVariable::XML_VARIABLE_NAME); xmlVar != NULL;
    xmlVar = xmlVar->NextSiblingElement(DensoVariable::XML_VARIABLE_NAME))
  {
    hr = DensoBase::AddVariable(ID_ROBOT_GETVARIABLE, xmlVar, m_vecVar);
    if (FAILED(hr)) {
      break;
    }
  }

  return hr;
}

HRESULT DensoRobot::ExecMove(int comp, const VARIANT_Ptr& pose, const std::string& option)
{
  HRESULT hr;

  hr = ExecTakeArm();
  if (SUCCEEDED(hr)) {
    int argc;
    VARIANT_Vec vntArgs;
    VARIANT_Ptr vntRet(new VARIANT());

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_ROBOT_MOVE_ARGS; argc++) {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc) {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
          break;
        case 1:
          vntTmp->vt = VT_I4;
          vntTmp->lVal = comp;
          break;
        case 2:
          VariantCopy(vntTmp.get(), pose.get());
          break;
        case 3:
          vntTmp->vt = VT_BSTR;
          vntTmp->bstrVal = ConvertStringToBSTR(option);
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_MOVE, vntArgs, vntRet);

    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecDrive(const std::string& name, const VARIANT_Ptr& option)
{
  HRESULT hr;

  hr = ExecTakeArm();
  if (SUCCEEDED(hr)) {
    int argc;
    VARIANT_Vec vntArgs;
    VARIANT_Ptr vntRet(new VARIANT());

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc) {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
          break;
        case 1:
          vntTmp->vt = VT_BSTR;
          vntTmp->bstrVal = ConvertStringToBSTR(name);
          break;
        case 2:
          VariantCopy(vntTmp.get(), option.get());
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);

    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecSpeed(float value)
{
  HRESULT hr;

  hr = ExecTakeArm();
  if (SUCCEEDED(hr)) {
    int argc;
    VARIANT_Vec vntArgs;
    VARIANT_Ptr vntRet(new VARIANT());

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_ROBOT_SPEED_ARGS; argc++) {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc) {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
          break;
        case 1:
          vntTmp->vt = VT_I4;
          vntTmp->lVal = -1;
          break;
        case 2:
          vntTmp->vt = VT_R4;
          vntTmp->fltVal = value;
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_SPEED, vntArgs, vntRet);

    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecChange(const std::string& value)
{
  HRESULT hr;

  hr = ExecTakeArm();
  if (SUCCEEDED(hr)) {
    int argc;
    VARIANT_Vec vntArgs;
    VARIANT_Ptr vntRet(new VARIANT());

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_ROBOT_CHANGE_ARGS; argc++) {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc) {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
          break;
        case 1:
          vntTmp->vt = VT_BSTR;
          vntTmp->bstrVal = ConvertStringToBSTR(value);
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_CHANGE, vntArgs, vntRet);

    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecHalt()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  for (argc = 0; argc < BCAP_ROBOT_HALT_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());

    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_WATCH]->ExecFunction(ID_ROBOT_HALT, vntArgs, vntRet);
}

HRESULT DensoRobot::ExecCurJnt(std::vector<double>& pose)
{
  HRESULT hr;

  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"HighCurJntEx");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = m_vecService[DensoBase::SRV_WATCH]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);

  if (SUCCEEDED(hr) && (vntRet->vt == (VT_ARRAY | VT_R8))) {
    uint32_t num;
    double* pdblval;

    num = vntRet->parray->rgsabound->cElements;
    SafeArrayAccessData(vntRet->parray, (void**)&pdblval);
    pose.resize(num - 1);
    std::copy(&pdblval[1], &pdblval[num], pose.begin());
    SafeArrayUnaccessData(vntRet->parray);
  }

  return hr;
}

HRESULT DensoRobot::ExecSlaveMove(const std::vector<double>& pose, std::vector<double>& joint)
{
  HRESULT hr = S_OK;
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"slvMove");
        break;
      case 2:
        hr = CreateSendParameter(pose, vntTmp, m_send_miniio, m_send_handio, m_recv_userio_offset, m_recv_userio_size,
                                 m_send_userio_offset, m_send_userio_size, m_send_userio);
        if (FAILED(hr)) {
          return hr;
        }
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
  if (SUCCEEDED(hr)) {
    HRESULT hrTmp = ParseRecvParameter(
      vntRet, m_position, m_joint, m_trans, m_recv_miniio, m_recv_handio,
      m_timestamp, m_recv_userio, m_current);

    joint = m_joint;

    if (FAILED(hrTmp)) {
      hr = hrTmp;
    }
  }

  return hr;
}

int DensoRobot::get_SendFormat() const
{
  return m_sendfmt;
}

/**
 * DensoRobot::put_SendFormat() has been deprecated.
 */
void DensoRobot::put_SendFormat(int format)
{
  ROS_WARN("DensoRobot::put_SendFormat() has been deprecated.");
  switch (format) {
    case SENDFMT_NONE:
    case SENDFMT_HANDIO:
    case SENDFMT_MINIIO:
    case SENDFMT_HANDIO | SENDFMT_MINIIO:
    case SENDFMT_USERIO:
    case SENDFMT_USERIO | SENDFMT_HANDIO:
      m_sendfmt = format;
      break;
    default:
      ROS_WARN("Failed to put_SendFormat.");
      break;
  }
}

void DensoRobot::set_SendFormat(int format)
{
  m_sendfmt = format;
}

int DensoRobot::get_RecvFormat() const
{
  return m_recvfmt;
}

/**
 * DensoRobot::put_RecvFormat() has been deprecated.
 */
void DensoRobot::put_RecvFormat(int format)
{
  ROS_WARN("DensoRobot::put_RecvFormat() has been deprecated.");
  int pose = format & RECVFMT_POSE;
  if ((RECVFMT_NONE <= pose) && (pose <= RECVFMT_POSE_TJ)) {
    switch (format & ~RECVFMT_POSE) {
      case RECVFMT_NONE:
      case RECVFMT_TIME:
      case RECVFMT_HANDIO:
      case RECVFMT_CURRENT:
      case RECVFMT_MINIIO:
      case RECVFMT_USERIO:
      case RECVFMT_TIME | RECVFMT_HANDIO:
      case RECVFMT_TIME | RECVFMT_MINIIO:
      case RECVFMT_HANDIO | RECVFMT_MINIIO:
      case RECVFMT_TIME | RECVFMT_HANDIO | RECVFMT_MINIIO:
      case RECVFMT_TIME | RECVFMT_USERIO:
      case RECVFMT_HANDIO | RECVFMT_USERIO:
      case RECVFMT_TIME | RECVFMT_HANDIO | RECVFMT_USERIO:
      case RECVFMT_CURRENT | RECVFMT_MINIIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_MINIIO:
      case RECVFMT_CURRENT | RECVFMT_HANDIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_HANDIO:
      case RECVFMT_CURRENT | RECVFMT_HANDIO | RECVFMT_MINIIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_HANDIO | RECVFMT_MINIIO:
      case RECVFMT_CURRENT | RECVFMT_USERIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_USERIO:
      case RECVFMT_CURRENT | RECVFMT_MINIIO | RECVFMT_USERIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_HANDIO | RECVFMT_USERIO:
        m_recvfmt = format;
        break;
      default:
        ROS_WARN("Failed to put_RecvFormat.");
        break;
    }
  }
  else
  {
    ROS_WARN("Failed to put_RecvFormat.");
  }
}

void DensoRobot::set_RecvFormat(int format)
{
  m_recvfmt = format;
}

int DensoRobot::get_TimeFormat() const
{
  return m_tsfmt;
}

void DensoRobot::put_TimeFormat(int format)
{
  if ((format == TSFMT_MILLISEC) || (format == TSFMT_MICROSEC)) {
    m_tsfmt = format;
  } else {
    ROS_WARN("Failed to put_TimeFormat.");
  }
}

unsigned int DensoRobot::get_MiniIO() const
{
  return m_recv_miniio;
}

void DensoRobot::put_MiniIO(unsigned int value)
{
  m_send_miniio = value;
}

unsigned int DensoRobot::get_HandIO() const
{
  return m_recv_handio;
}

void DensoRobot::put_HandIO(unsigned int value)
{
  m_send_handio = value;
}

void DensoRobot::put_SendUserIO(const denso_robot_core_interfaces::msg::UserIO::SharedPtr value)
{
  if (value->offset < UserIO::MIN_USERIO_OFFSET) {
    ROS_WARN("User I/O offset has to be greater than %d.", UserIO::MIN_USERIO_OFFSET - 1);
    return;
  }

  if (value->offset % UserIO::USERIO_ALIGNMENT) {
    ROS_WARN("User I/O offset has to be multiple of %d.", UserIO::USERIO_ALIGNMENT);
    return;
  }

  if (value->size <= 0) {
    ROS_WARN("User I/O size has to be greater than 0.");
    return;
  }

  if (static_cast<std::size_t>(value->size) < value->value.size()) {
    ROS_WARN("User I/O size has to be equal or greater than the value length.");
    return;
  }

  ROS_WARN("  ##########################   here send UserIO.");
  m_send_userio_offset = value->offset;
  m_send_userio_size = value->size;
  m_send_userio = value->value;
}

void DensoRobot::put_RecvUserIO(const denso_robot_core_interfaces::msg::UserIO::SharedPtr value)
{
  if (value->offset < denso_robot_core_interfaces::msg::UserIO::MIN_USERIO_OFFSET) {
    ROS_WARN(
      "User I/O offset has to be greater than %d.",
      denso_robot_core_interfaces::msg::UserIO::MIN_USERIO_OFFSET - 1);
    return;
  }

  if (value->offset % denso_robot_core_interfaces::msg::UserIO::USERIO_ALIGNMENT) {
    ROS_WARN(
      "User I/O offset has to be multiple of %d.",
      denso_robot_core_interfaces::msg::UserIO::USERIO_ALIGNMENT);
    return;
  }

  if (value->size <= 0) {
    ROS_WARN("User I/O size has to be greater than 0.");
    return;
  }

  m_recv_userio_offset = value->offset;
  m_recv_userio_size = value->size;
  ROS_WARN("  ##########################   here recv UserIO.");
}

void DensoRobot::get_RecvUserIO(denso_robot_core_interfaces::msg::UserIO::SharedPtr value) const
{
  value->offset = m_recv_userio_offset;
  value->size = m_recv_userio.size();
  value->value = m_recv_userio;
}

void DensoRobot::get_Current(std::vector<double>& current) const
{
  current = m_current;
}

int DensoRobot::get_Timestamp() const
{
  return m_timestamp;
}

HRESULT DensoRobot::CreatePoseData(
  const denso_robot_core_interfaces::msg::PoseData pose, VARIANT& vnt)
{
  uint32_t i;
  uint32_t num = 3 + (((pose.exjoints.mode != 0) && (pose.exjoints.joints.size() > 0)) ? 1 : 0);
  float* pfltval;
  VARIANT* pvntval;

  vnt.vt = (VT_ARRAY | VT_VARIANT);
  vnt.parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

  SafeArrayAccessData(vnt.parray, (void**)&pvntval);

  for (i = 0; i < num; i++) {
    switch (i) {
      case 0:
        pvntval[i].vt = (VT_ARRAY | VT_R4);
        pvntval[i].parray = SafeArrayCreateVector(VT_R4, 0, pose.value.size());
        SafeArrayAccessData(pvntval[i].parray, (void**)&pfltval);
        std::copy(pose.value.begin(), pose.value.end(), pfltval);
        SafeArrayUnaccessData(pvntval[i].parray);
        break;
      case 1:
        pvntval[i].vt = VT_I4;
        pvntval[i].lVal = pose.type;
        break;
      case 2:
        pvntval[i].vt = VT_I4;
        pvntval[i].lVal = pose.passflag;
        break;
      case 3:
        CreateExJoints(pose.exjoints, pvntval[i]);
        break;
    }
  }

  SafeArrayUnaccessData(vnt.parray);

  return S_OK;
}

HRESULT DensoRobot::CreateExJoints(
  const denso_robot_core_interfaces::msg::ExJoints exjoints, VARIANT& vnt)
{
  uint32_t i;
  uint32_t num = 1 + exjoints.joints.size();
  VARIANT *pvntval, *pjntval;

  vnt.vt = (VT_ARRAY | VT_VARIANT);
  vnt.parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

  SafeArrayAccessData(vnt.parray, (void**)&pvntval);

  for (i = 0; i < num; i++) {
    if (i == 0) {
      pvntval[0].vt = VT_I4;
      pvntval[0].lVal = exjoints.mode;
    } else {
      pvntval[i].vt = (VT_ARRAY | VT_VARIANT);
      pvntval[i].parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
      SafeArrayAccessData(pvntval[i].parray, (void**)&pjntval);
      pjntval[0].vt = VT_I4;
      pjntval[0].lVal = exjoints.joints.at(i - 1).joint;
      pjntval[1].vt = VT_R4;
      pjntval[1].fltVal = exjoints.joints.at(i - 1).value;
      SafeArrayUnaccessData(pvntval[i].parray);
    }
  }

  SafeArrayUnaccessData(vnt.parray);

  return S_OK;
}

HRESULT DensoRobot::ChangeMode(int mode)
{
  HRESULT hr = S_OK;

  if (*m_mode == 0) {
    // Change to slave mode
    if (mode != 0) {
      hr = ExecSlaveMode("slvSendFormat", m_sendfmt);
      if (FAILED(hr)) {
        ROS_ERROR("Invalid argument value (send_format = 0x%x)", m_sendfmt);
        return hr;
      }
      hr = ExecSlaveMode("slvRecvFormat", m_recvfmt, m_tsfmt);
      if (FAILED(hr)) {
        ROS_ERROR("Invalid argument value (recv_format = 0x%x)", m_recvfmt);
        return hr;
      }
      hr = ExecTakeArm();
      if (FAILED(hr)) {
        return hr;
      }

      hr = ExecSlaveMode("slvChangeMode", mode);
      if (FAILED(hr)) {
        return hr;
      }

      m_memTimeout = m_vecService[DensoBase::SRV_ACT]->get_Timeout();
      m_memRetry = m_vecService[DensoBase::SRV_ACT]->get_Retry();
      if (mode & DensoRobot::SLVMODE_SYNC_WAIT) {
        m_vecService[DensoBase::SRV_ACT]->put_Timeout(this->SLVMODE_TIMEOUT_SYNC);
      } else {
        m_vecService[DensoBase::SRV_ACT]->put_Timeout(this->SLVMODE_TIMEOUT_ASYNC);
      }
      ROS_INFO(
        "bcap-slave timeout changed to %d msec [mode: 0x%X]",
        m_vecService[DensoBase::SRV_ACT]->get_Timeout(), mode);
      m_vecService[DensoBase::SRV_ACT]->put_Retry(3);
    }
  } else {
    m_vecService[DensoBase::SRV_ACT]->put_Timeout(m_memTimeout);
    m_vecService[DensoBase::SRV_ACT]->put_Retry(m_memRetry);

    hr = ExecSlaveMode("slvChangeMode", mode);
    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecSlaveMode(const std::string& name, int32_t format, int32_t option)
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());
  int32_t* pval;

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc) {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = ConvertStringToBSTR(name);
        break;
      case 2:
        if (option == 0) {
          vntTmp->vt = VT_I4;
          vntTmp->lVal = format;
        } else {
          vntTmp->vt = (VT_ARRAY | VT_I4);
          vntTmp->parray = SafeArrayCreateVector(VT_I4, 0, 2);
          SafeArrayAccessData(vntTmp->parray, (void**)&pval);
          pval[0] = format;
          pval[1] = option;
          SafeArrayUnaccessData(vntTmp->parray);
        }
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

HRESULT DensoRobot::CreateSendParameter(
  const std::vector<double>& pose, VARIANT_Ptr& send, const int miniio,
  const int handio, const int recv_userio_offset, const int recv_userio_size,
  const int send_userio_offset, const int send_userio_size,
  const std::vector<uint8_t>& send_userio)
{
  int type = *m_mode & SLVMODE_POSE;

  // Check pose type
  int joints = 0;
  switch (type) {
    case SLVMODE_POSE_P:
      joints = NUM_POSITION;
      break;
    case SLVMODE_POSE_J:
      joints = NUM_JOINT;
      break;
    case SLVMODE_POSE_T:
      joints = NUM_TRANS;
      break;
    default:
      return E_FAIL;
  }

  // TODO: do we need this control ? Why?
  // if(joints < pose.size())
  // {
    // return E_FAIL;
  // }

  // Check send format
  bool send_hio, send_mio, send_uio, recv_uio;
  send_hio = m_sendfmt & SENDFMT_HANDIO;
  send_mio = m_sendfmt & SENDFMT_MINIIO;
  send_uio = m_sendfmt & SENDFMT_USERIO;

  if (send_uio) {
    if (static_cast<std::size_t>(send_userio_size) < send_userio.size()) {
      return E_FAIL;
    }
  }

  // Check receive format
  recv_uio = m_recvfmt & RECVFMT_USERIO;

  // Number of arguments
  int num = 1 + send_hio + send_mio + 3 * send_uio + 2 * recv_uio;

  VARIANT* pvnt;
  double* pdbl;
  uint8_t* pbool;

  // Number of joints + option
  joints += 1;
  if (num == 1) {
    // Pose only
    send->vt = (VT_ARRAY | VT_R8);
    send->parray = SafeArrayCreateVector(VT_R8, 0, joints);
    SafeArrayAccessData(send->parray, (void**)&pdbl);
    memset(pdbl, 0, joints * sizeof(double));
    std::copy(pose.begin(), pose.end(), pdbl);
    SafeArrayUnaccessData(send->parray);
  } else {
    send->vt = (VT_ARRAY | VT_VARIANT);
    send->parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

    SafeArrayAccessData(send->parray, (void**)&pvnt);

    int offset = 0;

    // Pose
    {
      pvnt[offset].vt = (VT_ARRAY | VT_R8);
      pvnt[offset].parray = SafeArrayCreateVector(VT_R8, 0, joints);
      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      memset(pdbl, 0, joints * sizeof(double));
      std::copy(pose.begin(), pose.end(), pdbl);
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Mini I/O
    if (send_mio) {
      pvnt[offset].vt = VT_I4;
      pvnt[offset].lVal = miniio;

      offset++;
    }

    // Send User I/O
    if (send_uio) {
      pvnt[offset + 0].vt = VT_I4;
      pvnt[offset + 0].lVal = send_userio_offset;

      pvnt[offset + 1].vt = VT_I4;
      pvnt[offset + 1].lVal = send_userio_size * UserIO::USERIO_ALIGNMENT;

      pvnt[offset + 2].vt = (VT_ARRAY | VT_UI1);
      pvnt[offset + 2].parray = SafeArrayCreateVector(VT_UI1, 0, send_userio_size);
      SafeArrayAccessData(pvnt[offset + 2].parray, (void**)&pbool);
      memset(pbool, 0, send_userio_size * sizeof(uint8_t));
      std::copy(send_userio.begin(), send_userio.end(), pbool);
      SafeArrayUnaccessData(pvnt[offset + 2].parray);

      offset += 3;
    }

    // Receive User I/O
    if (recv_uio) {
      pvnt[offset + 0].vt = VT_I4;
      pvnt[offset + 0].lVal = recv_userio_offset;

      pvnt[offset + 1].vt = VT_I4;
      pvnt[offset + 1].lVal = recv_userio_size * UserIO::USERIO_ALIGNMENT;

      offset += 2;
    }

    // Hand I/O
    if (send_hio) {
      pvnt[offset].vt = VT_I4;
      pvnt[offset].lVal = handio;

      offset++;
    }

    SafeArrayUnaccessData(send->parray);
  }

  return S_OK;
}

HRESULT DensoRobot::ParseRecvParameter(
  const VARIANT_Ptr& recv, std::vector<double>& position,
  std::vector<double>& joint, std::vector<double>& trans, int& miniio, int& handio,
  int& timestamp, std::vector<uint8_t>& recv_userio, std::vector<double>& current)
{
  int type = m_recvfmt & SLVMODE_POSE;

  // Check pose type
  int j1 = 0, j2 = 0;
  uint32_t joints = 0;
  std::vector<double>*pose1 = NULL, *pose2 = NULL;

  switch (type) {
    case RECVFMT_POSE_P:
      j1 = NUM_POSITION;
      pose1 = &position;
      break;
    case RECVFMT_POSE_J:
      j1 = NUM_JOINT;
      pose1 = &joint;
      break;
    case RECVFMT_POSE_T:
      j1 = NUM_TRANS;
      pose1 = &trans;
      break;
    case RECVFMT_POSE_PJ:
      j1 = NUM_POSITION;
      j2 = NUM_JOINT;
      pose1 = &position;
      pose2 = &joint;
      break;
    case RECVFMT_POSE_TJ:
      j1 = NUM_TRANS;
      j2 = NUM_JOINT;
      pose1 = &trans;
      pose2 = &joint;
      break;
    default:
      return E_FAIL;
  }

  joints = j1 + j2;

  // Check receive format
  bool recv_ts, recv_hio, recv_mio, recv_uio, recv_crt;
  recv_ts = m_recvfmt & RECVFMT_TIME;
  recv_hio = m_recvfmt & RECVFMT_HANDIO;
  recv_mio = m_recvfmt & RECVFMT_MINIIO;
  recv_uio = m_recvfmt & RECVFMT_USERIO;
  recv_crt = m_recvfmt & RECVFMT_CURRENT;

  // Number of arguments
  uint32_t num = 1 + recv_ts + recv_hio + recv_mio + recv_uio + recv_crt;

  HRESULT hr = S_OK;
  VARIANT* pvnt;
  double* pdbl;
  uint8_t* pbool;

  if (recv->vt == (VT_ARRAY | VT_R8)) {
    if (joints != recv->parray->rgsabound->cElements) {
      return E_FAIL;
    }

    // Pose only
    SafeArrayAccessData(recv->parray, (void**)&pdbl);
    pose1->resize(j1);
    std::copy(pdbl, &pdbl[j1], pose1->begin());
    if (pose2 != NULL) {
      pose2->resize(j2);
      std::copy(&pdbl[j1], &pdbl[joints], pose2->begin());
    }
    SafeArrayUnaccessData(recv->parray);
  } else if (recv->vt == (VT_ARRAY | VT_VARIANT)) {
    if (num != recv->parray->rgsabound->cElements) {
      return E_FAIL;
    }

    SafeArrayAccessData(recv->parray, (void**)&pvnt);

    int offset = 0;

    // Timestamp
    if (recv_ts) {
      if (pvnt[offset].vt != VT_I4) {
        hr = E_FAIL;
        goto exit_proc;
      }

      timestamp = pvnt[offset].lVal;

      offset++;
    }

    // Pose
    {
      if (
        (pvnt[offset].vt != (VT_ARRAY | VT_R8)) || (joints != pvnt[offset].parray->rgsabound->cElements))
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      pose1->resize(j1);
      std::copy(pdbl, &pdbl[j1], pose1->begin());
      if (pose2 != NULL) {
        pose2->resize(j2);
        std::copy(&pdbl[j1], &pdbl[joints], pose2->begin());
      }
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Mini I/O
    if (recv_mio) {
      if (pvnt[offset].vt != VT_I4) {
        hr = E_FAIL;
        goto exit_proc;
      }

      miniio = pvnt[offset].lVal;

      offset++;
    }

    // User I/O
    if (recv_uio) {
      if (pvnt[offset].vt != (VT_ARRAY | VT_UI1)) {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pbool);
      recv_userio.resize(pvnt[offset].parray->rgsabound->cElements);
      std::copy(pbool, &pbool[pvnt[offset].parray->rgsabound->cElements], recv_userio.begin());
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Hand I/O
    if (recv_hio) {
      if (pvnt[offset].vt != VT_I4) {
        hr = E_FAIL;
        goto exit_proc;
      }

      handio = pvnt[offset].lVal;

      offset++;
    }

    // Current
    if (recv_crt) {
      if (
        (pvnt[offset].vt != (VT_ARRAY | VT_R8)) || (8 != pvnt[offset].parray->rgsabound->cElements))
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      current.resize(8);
      std::copy(pdbl, &pdbl[8], current.begin());
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    exit_proc:
    SafeArrayUnaccessData(recv->parray);
  } else {
    return E_FAIL;
  }

  return hr;
}


rclcpp_action::GoalResponse DensoRobot::Callback_MoveStringHandleGoal(
  const rclcpp_action::GoalUUID & /* uuid */, std::shared_ptr<const MoveString::Goal> /* goal */)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DensoRobot::Callback_MoveStringHandleCancel(
  const std::shared_ptr<GoalHandleMoveString> goal_handle)
{
  auto res = std::make_shared<MoveString::Result>();

  this->ExecHalt();
  this->m_curAct = ACT_NONE;

  goal_handle->canceled(res);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DensoRobot::Callback_MoveStringHandleAccepted(
  const std::shared_ptr<GoalHandleMoveString> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DensoRobot::Callback_MoveString, this, _1), goal_handle}.detach();
}

void DensoRobot::Callback_MoveString(const std::shared_ptr<GoalHandleMoveString> goal_handle)
{
  HRESULT hr;
  auto res = std::make_shared<MoveString::Result>();

  const auto goal = goal_handle->get_goal();
  m_actMoveStringGoalHandle = goal_handle;

  // Set current action
  std::unique_lock<std::mutex> lockAct(m_mtxAct);
  if (m_curAct != ACT_NONE) {
    if (m_curAct != ACT_RESET) {
      res->hresult = E_FAIL;
      goal_handle->abort(res);
    }
    return;
  }
  m_curAct = ACT_MOVESTRING;
  lockAct.unlock();

  VARIANT_Ptr vntPose(new VARIANT());
  VariantInit(vntPose.get());
  vntPose->vt = VT_BSTR;
  vntPose->bstrVal = ConvertStringToBSTR(goal->pose);

  hr = ExecMove(goal->comp, vntPose, goal->option);

  // Reset current action
  m_mtxAct.lock();
  if (m_curAct == ACT_MOVESTRING) {
    if (SUCCEEDED(hr)) {
      res->hresult = S_OK;
      goal_handle->succeed(res);
    } else {
      res->hresult = hr;
      goal_handle->abort(res);
    }
    m_curAct = ACT_NONE;
  }
  m_mtxAct.unlock();
}

rclcpp_action::GoalResponse DensoRobot::Callback_MoveValueHandleGoal(
  const rclcpp_action::GoalUUID & /* uuid */, std::shared_ptr<const MoveValue::Goal> /* goal */)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DensoRobot::Callback_MoveValueHandleCancel(
  const std::shared_ptr<GoalHandleMoveValue> goal_handle)
{
  auto res = std::make_shared<MoveValue::Result>();

  this->ExecHalt();
  this->m_curAct = ACT_NONE;

  goal_handle->canceled(res);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DensoRobot::Callback_MoveValueHandleAccepted(
  const std::shared_ptr<GoalHandleMoveValue> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DensoRobot::Callback_MoveValue, this, _1), goal_handle}.detach();
}

void DensoRobot::Callback_MoveValue(
  const std::shared_ptr<GoalHandleMoveValue> goal_handle)
{
  HRESULT hr;
  auto res = std::make_shared<MoveValue::Result>();

  const auto goal = goal_handle->get_goal();
  m_actMoveValueGoalHandle = goal_handle;

  // Set current action
  std::unique_lock<std::mutex> lockAct(m_mtxAct);
  if (m_curAct != ACT_NONE) {
    if (m_curAct != ACT_RESET) {
      res->hresult = E_FAIL;
      goal_handle->abort(res);
    }
    return;
  }
  m_curAct = ACT_MOVEVALUE;
  lockAct.unlock();

  VARIANT_Ptr vntPose(new VARIANT());
  VariantInit(vntPose.get());
  CreatePoseData(goal->pose, *vntPose.get());

  hr = ExecMove(goal->comp, vntPose, goal->option);

  // Reset current action
  m_mtxAct.lock();
  if (m_curAct == ACT_MOVEVALUE) {
    if (SUCCEEDED(hr)) {
      res->hresult = S_OK;
      goal_handle->succeed(res);
    } else {
      res->hresult = hr;
      goal_handle->abort(res);
    }
    m_curAct = ACT_NONE;
  }
  m_mtxAct.unlock();
}

rclcpp_action::GoalResponse DensoRobot::Callback_DriveStringHandleGoal(
  const rclcpp_action::GoalUUID & /* uuid */, std::shared_ptr<const DriveString::Goal> /* goal */)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DensoRobot::Callback_DriveStringHandleCancel(
  const std::shared_ptr<GoalHandleDriveString> goal_handle)
{
  auto res = std::make_shared<DriveString::Result>();

  this->ExecHalt();
  this->m_curAct = ACT_NONE;

  goal_handle->canceled(res);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void DensoRobot::Callback_DriveStringHandleAccepted(
  const std::string& name, const std::shared_ptr<GoalHandleDriveString> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DensoRobot::Callback_DriveString, this, name, _1), goal_handle}.detach();
}

void DensoRobot::Callback_DriveString(
  const std::string& name, const std::shared_ptr<GoalHandleDriveString> goal_handle)
{
  HRESULT hr;
  auto res = std::make_shared<DriveString::Result>();
  const auto goal = goal_handle->get_goal();
  BSTR * pbstr;

  int act = 0;

  if (!name.compare("DriveEx")) {
    act = ACT_DRIVEEXSTRING;
    m_actDriveExStringGoalHandle = goal_handle;
  } else if (!name.compare("DriveAEx")) {
    act = ACT_DRIVEAEXSTRING;
    m_actDriveAExStringGoalHandle = goal_handle;
  } else {
    return;
  }

  // Set current action
  std::unique_lock<std::mutex> lockAct(m_mtxAct);
  if (m_curAct != ACT_NONE) {
    if (m_curAct != ACT_RESET) {
      res->hresult = E_FAIL;
      goal_handle->abort(res);
    }
    return;
  }
  m_curAct = act;
  lockAct.unlock();

  VARIANT_Ptr vntOpt(new VARIANT());
  VariantInit(vntOpt.get());
  vntOpt->vt = (VT_ARRAY | VT_BSTR);
  vntOpt->parray = SafeArrayCreateVector(VT_BSTR, 0, 2);
  SafeArrayAccessData(vntOpt->parray, (void**)&pbstr);
  pbstr[0] = ConvertStringToBSTR(goal->pose);
  pbstr[1] = ConvertStringToBSTR(goal->option);
  SafeArrayUnaccessData(vntOpt->parray);

  hr = ExecDrive(name, vntOpt);

  // Reset current action
  m_mtxAct.lock();
  if (m_curAct == act) {
    if (SUCCEEDED(hr)) {
      res->hresult = S_OK;
      goal_handle->succeed(res);
    } else {
      res->hresult = hr;
      goal_handle->abort(res);
    }
    m_curAct = ACT_NONE;
  }
  m_mtxAct.unlock();
}

void DensoRobot::Callback_DriveExStringHandleAccepted(
  const std::shared_ptr<GoalHandleDriveString> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DensoRobot::Callback_DriveExString, this, _1), goal_handle}.detach();
}

void DensoRobot::Callback_DriveExString(const std::shared_ptr<GoalHandleDriveString> goal_handle)
{
  HRESULT hr;
  auto res = std::make_shared<DriveString::Result>();
  const auto goal = goal_handle->get_goal();
  BSTR * pbstr;
  int act = 0;

  act = ACT_DRIVEEXSTRING;
  m_actDriveExStringGoalHandle = goal_handle;

  // Set current action
  std::unique_lock<std::mutex> lockAct(m_mtxAct);
  if (m_curAct != ACT_NONE) {
    if (m_curAct != ACT_RESET) {
      res->hresult = E_FAIL;
      goal_handle->abort(res);
    }
    return;
  }
  m_curAct = act;
  lockAct.unlock();

  VARIANT_Ptr vntOpt(new VARIANT());
  VariantInit(vntOpt.get());
  vntOpt->vt = (VT_ARRAY | VT_BSTR);
  vntOpt->parray = SafeArrayCreateVector(VT_BSTR, 0, 2);
  SafeArrayAccessData(vntOpt->parray, (void**)&pbstr);
  pbstr[0] = ConvertStringToBSTR(goal->pose);
  pbstr[1] = ConvertStringToBSTR(goal->option);
  SafeArrayUnaccessData(vntOpt->parray);

  hr = ExecDrive("DriveEx", vntOpt);

  // Reset current action
  m_mtxAct.lock();
  if (m_curAct == act) {
    if (SUCCEEDED(hr)) {
      res->hresult = S_OK;
      goal_handle->succeed(res);
    } else {
      res->hresult = hr;
      goal_handle->abort(res);
    }
    m_curAct = ACT_NONE;
  }
  m_mtxAct.unlock();
}

void DensoRobot::Callback_DriveAExStringHandleAccepted(
  const std::shared_ptr<GoalHandleDriveString> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DensoRobot::Callback_DriveAExString, this, _1), goal_handle}.detach();
}

void DensoRobot::Callback_DriveAExString(
  const std::shared_ptr<GoalHandleDriveString> goal_handle)
{
  HRESULT hr;
  auto res = std::make_shared<DriveString::Result>();
  const auto goal = goal_handle->get_goal();
  BSTR * pbstr;
  int act = 0;

  act = ACT_DRIVEAEXSTRING;
  m_actDriveAExStringGoalHandle = goal_handle;

  // Set current action
  std::unique_lock<std::mutex> lockAct(m_mtxAct);
  if (m_curAct != ACT_NONE) {
    if (m_curAct != ACT_RESET) {
      res->hresult = E_FAIL;
      goal_handle->abort(res);
    }
    return;
  }
  m_curAct = act;
  lockAct.unlock();

  VARIANT_Ptr vntOpt(new VARIANT());
  VariantInit(vntOpt.get());
  vntOpt->vt = (VT_ARRAY | VT_BSTR);
  vntOpt->parray = SafeArrayCreateVector(VT_BSTR, 0, 2);
  SafeArrayAccessData(vntOpt->parray, (void**)&pbstr);
  pbstr[0] = ConvertStringToBSTR(goal->pose);
  pbstr[1] = ConvertStringToBSTR(goal->option);
  SafeArrayUnaccessData(vntOpt->parray);

  hr = ExecDrive("DriveAEx", vntOpt);

  // Reset current action
  m_mtxAct.lock();
  if (m_curAct == act) {
    if (SUCCEEDED(hr)) {
      res->hresult = S_OK;
    goal_handle->succeed(res);
    } else {
      res->hresult = hr;
      goal_handle->abort(res);
    }
    m_curAct = ACT_NONE;
  }
  m_mtxAct.unlock();
}


rclcpp_action::GoalResponse DensoRobot::Callback_DriveValueHandleGoal(
  const rclcpp_action::GoalUUID & /* uuid */, std::shared_ptr<const DriveValue::Goal> /* goal */)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DensoRobot::Callback_DriveValueHandleCancel(
  const std::shared_ptr<GoalHandleDriveValue> goal_handle)
{
  auto res = std::make_shared<DriveValue::Result>();

  this->ExecHalt();
  this->m_curAct = ACT_NONE;

  goal_handle->canceled(res);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void DensoRobot::Callback_DriveValueHandleAccepted(
  const std::string& name, const std::shared_ptr<GoalHandleDriveValue> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DensoRobot::Callback_DriveValue, this, name, _1), goal_handle}.detach();
}

void DensoRobot::Callback_DriveValue(
  const std::string& name, const std::shared_ptr<GoalHandleDriveValue> goal_handle)
{
  HRESULT hr;
  auto res = std::make_shared<DriveValue::Result>();
  const auto goal = goal_handle->get_goal();
  VARIANT *pvntval, *pvntjnt;
  int act = 0;

  if (!name.compare("DriveEx")) {
    act = ACT_DRIVEEXVALUE;
    m_actDriveExValueGoalHandle = goal_handle;
  } else if (!name.compare("DriveAEx")) {
    act = ACT_DRIVEAEXVALUE;
    m_actDriveAExValueGoalHandle = goal_handle;
  } else {
    return;
  }

  // Set current action
  std::unique_lock<std::mutex> lockAct(m_mtxAct);
  if (m_curAct != ACT_NONE) {
    if (m_curAct != ACT_RESET) {
      res->hresult = E_FAIL;
      goal_handle->abort(res);
    }
    return;
  }
  m_curAct = act;
  lockAct.unlock();

  VARIANT_Ptr vntOpt(new VARIANT());
  VariantInit(vntOpt.get());

  vntOpt->vt = (VT_ARRAY | VT_VARIANT);
  vntOpt->parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);

  SafeArrayAccessData(vntOpt->parray, (void**)&pvntval);

  pvntval[0].vt = (VT_ARRAY | VT_VARIANT);
  pvntval[0].parray = SafeArrayCreateVector(VT_VARIANT, 0, goal->pose.size());

  SafeArrayAccessData(pvntval[0].parray, (void**)&pvntjnt);

  for (std::size_t i = 0; i < goal->pose.size(); i++) {
    denso_robot_core_interfaces::msg::PoseData pd;
    pd.value.push_back(goal->pose.at(i).joint);
    pd.value.push_back(goal->pose.at(i).value);
    pd.type = -1;
    pd.passflag = (i == 0) ? goal->passflag : 0;
    CreatePoseData(pd, pvntjnt[i]);
  }

  SafeArrayUnaccessData(pvntval[0].parray);

  pvntval[1].vt = VT_BSTR;
  pvntval[1].bstrVal = ConvertStringToBSTR(goal->option);

  SafeArrayUnaccessData(vntOpt->parray);

  hr = ExecDrive(name, vntOpt);

  // Reset current action
  m_mtxAct.lock();
  if (m_curAct == act) {
    if (SUCCEEDED(hr)) {
      res->hresult = S_OK;
    goal_handle->succeed(res);
    } else {
      res->hresult = hr;
    goal_handle->abort(res);
    }
    m_curAct = ACT_NONE;
  }
  m_mtxAct.unlock();
}

void DensoRobot::Callback_DriveExValueHandleAccepted(
  const std::shared_ptr<GoalHandleDriveValue> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DensoRobot::Callback_DriveExValue, this, _1), goal_handle}.detach();
}

void DensoRobot::Callback_DriveExValue(const std::shared_ptr<GoalHandleDriveValue> goal_handle)
{
  HRESULT hr;
  auto res = std::make_shared<DriveValue::Result>();
  const auto goal = goal_handle->get_goal();
  VARIANT * pvntval, * pvntjnt;
  int act = 0;
  act = ACT_DRIVEEXVALUE;
  m_actDriveExValueGoalHandle = goal_handle;

  // Set current action
  std::unique_lock<std::mutex> lockAct(m_mtxAct);
  if (m_curAct != ACT_NONE) {
    if (m_curAct != ACT_RESET) {
      res->hresult = E_FAIL;
      goal_handle->abort(res);
    }
    return;
  }
  m_curAct = act;
  lockAct.unlock();

  VARIANT_Ptr vntOpt(new VARIANT());
  VariantInit(vntOpt.get());

  vntOpt->vt = (VT_ARRAY | VT_VARIANT);
  vntOpt->parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);

  SafeArrayAccessData(vntOpt->parray, (void**)&pvntval);

  pvntval[0].vt = (VT_ARRAY | VT_VARIANT);
  pvntval[0].parray = SafeArrayCreateVector(VT_VARIANT, 0, goal->pose.size());

  SafeArrayAccessData(pvntval[0].parray, (void**)&pvntjnt);

  for (std::size_t i = 0; i < goal->pose.size(); i++) {
    denso_robot_core_interfaces::msg::PoseData pd;
    pd.value.push_back(goal->pose.at(i).joint);
    pd.value.push_back(goal->pose.at(i).value);
    pd.type = -1;
    pd.passflag = (i == 0) ? goal->passflag : 0;
    CreatePoseData(pd, pvntjnt[i]);
  }

  SafeArrayUnaccessData(pvntval[0].parray);

  pvntval[1].vt = VT_BSTR;
  pvntval[1].bstrVal = ConvertStringToBSTR(goal->option);

  SafeArrayUnaccessData(vntOpt->parray);

  hr = ExecDrive("DriveEx", vntOpt);

  // Reset current action
  m_mtxAct.lock();
  if (m_curAct == act) {
    if (SUCCEEDED(hr)) {
      res->hresult = S_OK;
    goal_handle->succeed(res);
    } else {
      res->hresult = hr;
      goal_handle->abort(res);
    }
    m_curAct = ACT_NONE;
  }
  m_mtxAct.unlock();
}

void DensoRobot::Callback_DriveAExValueHandleAccepted(
  const std::shared_ptr<GoalHandleDriveValue> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DensoRobot::Callback_DriveAExValue, this, _1), goal_handle}.detach();
}

void DensoRobot::Callback_DriveAExValue(
  const std::shared_ptr<GoalHandleDriveValue> goal_handle)
{
  HRESULT hr;
  auto res = std::make_shared<DriveValue::Result>();
  const auto goal = goal_handle->get_goal();
  VARIANT *pvntval, *pvntjnt;
  int act = 0;

  act = ACT_DRIVEAEXVALUE;
  m_actDriveAExValueGoalHandle = goal_handle;

  // Set current action
  std::unique_lock<std::mutex> lockAct(m_mtxAct);
  if (m_curAct != ACT_NONE) {
    if (m_curAct != ACT_RESET) {
      res->hresult = E_FAIL;
      goal_handle->abort(res);
    }
    return;
  }
  m_curAct = act;
  lockAct.unlock();

  VARIANT_Ptr vntOpt(new VARIANT());
  VariantInit(vntOpt.get());

  vntOpt->vt = (VT_ARRAY | VT_VARIANT);
  vntOpt->parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);

  SafeArrayAccessData(vntOpt->parray, (void**)&pvntval);

  pvntval[0].vt = (VT_ARRAY | VT_VARIANT);
  pvntval[0].parray = SafeArrayCreateVector(VT_VARIANT, 0, goal->pose.size());

  SafeArrayAccessData(pvntval[0].parray, (void**)&pvntjnt);

  for (std::size_t i = 0; i < goal->pose.size(); i++) {
    denso_robot_core_interfaces::msg::PoseData pd;
    pd.value.push_back(goal->pose.at(i).joint);
    pd.value.push_back(goal->pose.at(i).value);
    pd.type = -1;
    pd.passflag = (i == 0) ? goal->passflag : 0;
    CreatePoseData(pd, pvntjnt[i]);
  }

  SafeArrayUnaccessData(pvntval[0].parray);

  pvntval[1].vt = VT_BSTR;
  pvntval[1].bstrVal = ConvertStringToBSTR(goal->option);

  SafeArrayUnaccessData(vntOpt->parray);

  hr = ExecDrive("DriveAEx", vntOpt);

  // Reset current action
  m_mtxAct.lock();
  if (m_curAct == act) {
    if (SUCCEEDED(hr)) {
      res->hresult = S_OK;
      goal_handle->succeed(res);
    } else {
      res->hresult = hr;
      goal_handle->abort(res);
    }
    m_curAct = ACT_NONE;
  }
  m_mtxAct.unlock();
}

void DensoRobot::Callback_Speed(const std_msgs::msg::Float32::SharedPtr msg)
{
  ExecSpeed(msg->data);
}

void DensoRobot::Callback_Change(
  const std::string& name, const std_msgs::msg::Int32::SharedPtr msg)
{
  std::stringstream ss;
  ss << name << msg->data;
  ExecChange(ss.str());
}
void DensoRobot::Callback_ChangeTool(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::stringstream ss;
  ss << "Tool" << msg->data;
  ExecChange(ss.str());
}
void DensoRobot::Callback_ChangeWork(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::stringstream ss;
  ss << "Work" << msg->data;
  ExecChange(ss.str());
}

void DensoRobot::Callback_Cancel()
{
  std::unique_lock<std::mutex> lockAct(m_mtxAct);

  if (m_curAct > ACT_NONE) {
    ExecHalt();

  // TODO: check the equivalent for setPreempted() function in ROS2
    switch (m_curAct) {
      case ACT_MOVESTRING:
        // m_actMoveString->setPreempted();
        break;
      case ACT_MOVEVALUE:
        // m_actMoveValue->setPreempted();
        break;
      case ACT_DRIVEEXSTRING:
        // m_actDriveExString->setPreempted();
        break;
      case ACT_DRIVEEXVALUE:
        // m_actDriveExValue->setPreempted();
        break;
      case ACT_DRIVEAEXSTRING:
        // m_actDriveAExString->setPreempted();
        break;
      case ACT_DRIVEAEXVALUE:
        // m_actDriveAExValue->setPreempted();
        break;
    }
    m_curAct = ACT_NONE;
  }
}

void DensoRobot::Action_Feedback()
{
  std::unique_lock<std::mutex> lockAct(m_mtxAct);

  if (m_curAct > ACT_NONE) {
    HRESULT hr;
    std::vector<double> pose;

    auto fbMvStr = std::make_shared<MoveString::Feedback>();
    auto fbMvVal = std::make_shared<MoveValue::Feedback>();
    auto fbDrvStr = std::make_shared<DriveString::Feedback>();
    auto fbDrvVal = std::make_shared<DriveValue::Feedback>();

    hr = ExecCurJnt(pose);

    if (SUCCEEDED(hr)) {
      switch (m_curAct) {
        case ACT_MOVESTRING:
          fbMvStr->pose = pose;
          m_actMoveStringGoalHandle->publish_feedback(fbMvStr);
          break;
        case ACT_MOVEVALUE:
          fbMvVal->pose = pose;
          m_actMoveValueGoalHandle->publish_feedback(fbMvVal);
          break;
        case ACT_DRIVEEXSTRING:
          fbDrvStr->pose = pose;
          m_actDriveExStringGoalHandle->publish_feedback(fbDrvStr);
          break;
        case ACT_DRIVEEXVALUE:
          fbDrvVal->pose = pose;
          m_actDriveExValueGoalHandle->publish_feedback(fbDrvVal);
          break;
        case ACT_DRIVEAEXSTRING:
          fbDrvStr->pose = pose;
          m_actDriveAExStringGoalHandle->publish_feedback(fbDrvStr);
          break;
        case ACT_DRIVEAEXVALUE:
          fbDrvVal->pose = pose;
          m_actDriveAExValueGoalHandle->publish_feedback(fbDrvVal);
          break;
      }
    }
  }
}

}  // namespace denso_robot_core
