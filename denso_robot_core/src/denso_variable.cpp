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

#include "denso_robot_core/denso_variable.h"
#include "rclcpp/macros.hpp"

namespace denso_robot_core {
DensoVariable::DensoVariable(
  DensoBase* parent, Service_Vec& service, Handle_Vec& handle, const std::string& name,
  const int* mode, int16_t vt, bool Read, bool Write, bool ID, int Duration)
  : DensoBase(parent, service, handle, name, mode), m_vt(vt), m_bRead(Read), m_bWrite(Write), m_bID(ID)
{
  m_Duration = Duration * 1000000;
  m_pubTimePrev = rclcpp::Clock().now();
}

DensoVariable::~DensoVariable()
{
}

HRESULT DensoVariable::StartService(rclcpp::Node::SharedPtr& node)
{
  if (*m_mode != 0) {
    return S_FALSE;
  }

  // Message name
  std::string tmpName = m_parent->RosName();
  if (tmpName != "") {
    tmpName.append("/");
  }
  tmpName.append(DensoBase::RosName());

  if (m_bRead) {
    switch (m_vt) {
      case VT_I4:
        m_pubInt32Value = node->create_publisher<std_msgs::msg::Int32>(
          tmpName + NAME_READ, MESSAGE_QUEUE);
        break;
      case VT_R4:
        m_pubFloat32Value = node->create_publisher<std_msgs::msg::Float32>(
          tmpName + NAME_READ, MESSAGE_QUEUE);
        break;
      case VT_R8:
        m_pubFloat64Value = node->create_publisher<std_msgs::msg::Float64>(
          tmpName + NAME_READ, MESSAGE_QUEUE);
        break;
      case VT_BSTR:
        m_pubStringValue = node->create_publisher<std_msgs::msg::String>(
          tmpName + NAME_READ, MESSAGE_QUEUE);
        break;
      case VT_BOOL:
        m_pubBoolValue = node->create_publisher<std_msgs::msg::Bool>(
          tmpName + NAME_READ, MESSAGE_QUEUE);
        break;
      case (VT_ARRAY | VT_R4):
        m_pubFloat32MultiArrayValue = node->create_publisher<std_msgs::msg::Float32MultiArray>(
          tmpName + NAME_READ, MESSAGE_QUEUE);
        break;
      case (VT_ARRAY | VT_R8):
        m_pubFloat64MultiArrayValue = node->create_publisher<std_msgs::msg::Float64MultiArray>(
          tmpName + NAME_READ, MESSAGE_QUEUE);
        break;
      default:
        return E_FAIL;
    }
  }

  if (m_bWrite) {
    switch (m_vt) {
      case VT_I4:
        m_subInt32Value = node->create_subscription<std_msgs::msg::Int32>(
          tmpName + NAME_WRITE, MESSAGE_QUEUE, std::bind(&DensoVariable::Callback_I32, this, _1));
        break;
      case VT_R4:
        m_subFloat32Value = node->create_subscription<std_msgs::msg::Float32>(
          tmpName + NAME_WRITE, MESSAGE_QUEUE, std::bind(&DensoVariable::Callback_F32, this, _1));
        break;
      case VT_R8:
        m_subFloat64Value = node->create_subscription<std_msgs::msg::Float64>(
          tmpName + NAME_WRITE, MESSAGE_QUEUE, std::bind(&DensoVariable::Callback_F64, this, _1));
        break;
      case VT_BSTR:
        m_subStringValue = node->create_subscription<std_msgs::msg::String>(
          tmpName + NAME_WRITE, MESSAGE_QUEUE,
          std::bind(&DensoVariable::Callback_String, this, _1));
        break;
      case VT_BOOL:
        m_subBoolValue = node->create_subscription<std_msgs::msg::Bool>(
          tmpName + NAME_WRITE, MESSAGE_QUEUE,
          std::bind(&DensoVariable::Callback_Bool, this, _1));
        break;
      case (VT_ARRAY | VT_R4):
        m_subFloat32MultiArrayValue = node->create_subscription<std_msgs::msg::Float32MultiArray>(
          tmpName + NAME_WRITE, MESSAGE_QUEUE,
          std::bind(&DensoVariable::Callback_F32Array, this, _1));
        break;
      case (VT_ARRAY | VT_R8):
        m_subFloat64MultiArrayValue = node->create_subscription<std_msgs::msg::Float64MultiArray>(
          tmpName + NAME_WRITE, MESSAGE_QUEUE,
          std::bind(&DensoVariable::Callback_F64Array, this, _1));
        break;
      default:
        return E_FAIL;
    }
  }

  if (m_bID) {
    m_subID = node->create_subscription<std_msgs::msg::Int32>(
      tmpName + NAME_ID, MESSAGE_QUEUE, std::bind(&DensoVariable::Callback_ID, this, _1));
  }

  m_serving = true;

  return S_OK;
}

HRESULT DensoVariable::StopService()
{
  m_mtxSrv.lock();
  m_serving = false;
  m_mtxSrv.unlock();

  m_pubInt32Value.reset();
  m_pubFloat32Value.reset();
  m_pubFloat64Value.reset();
  m_pubStringValue.reset();
  m_pubBoolValue.reset();
  m_pubFloat32MultiArrayValue.reset();
  m_pubFloat64MultiArrayValue.reset();

  m_subInt32Value.reset();
  m_subFloat32Value.reset();
  m_subFloat64Value.reset();
  m_subStringValue.reset();
  m_subBoolValue.reset();
  m_subFloat32MultiArrayValue.reset();
  m_subFloat64MultiArrayValue.reset();
  m_subID.reset();

  return S_OK;
}

bool DensoVariable::Update()
{
  std::unique_lock<std::mutex> lockSrv(m_mtxSrv);
  if (!m_serving) {
    return false;
  }

  if (m_bRead) {
    HRESULT hr;

    auto varI = std_msgs::msg::Int32();
    auto varF = std_msgs::msg::Float32();
    auto varD = std_msgs::msg::Float64();
    auto varS = std_msgs::msg::String();
    auto varIO = std_msgs::msg::Bool();
    auto varFArray = std_msgs::msg::Float32MultiArray();
    auto varDArray = std_msgs::msg::Float64MultiArray();

    uint32_t num;
    float* pfltval;
    double* pdblval;

    rclcpp::Time pubTimeCur = rclcpp::Clock().now();

    if ((pubTimeCur - m_pubTimePrev).nanoseconds() > m_Duration) {
      VARIANT_Ptr vntRet(new VARIANT());
      VariantInit(vntRet.get());

      hr = ExecGetValue(vntRet);
      if (SUCCEEDED(hr)) {
        if (vntRet->vt == m_vt) {
          switch (m_vt) {
            case VT_I4:
              varI.data = vntRet->lVal;
              m_pubInt32Value->publish(varI);
              break;
            case VT_R4:
              varF.data = vntRet->fltVal;
              m_pubFloat32Value->publish(varF);
              break;
            case VT_R8:
              varD.data = vntRet->dblVal;
              m_pubFloat64Value->publish(varD);
              break;
            case VT_BSTR:
              varS.data = ConvertBSTRToString(vntRet->bstrVal);
              m_pubStringValue->publish(varS);
              break;
            case VT_BOOL:
              varIO.data = (vntRet->boolVal != VARIANT_FALSE) ? true : false;
              m_pubBoolValue->publish(varIO);
              break;
            case (VT_ARRAY | VT_R4):
              num = vntRet->parray->rgsabound->cElements;
              SafeArrayAccessData(vntRet->parray, (void**)&pfltval);
              varFArray.data.resize(num);
              std::copy(pfltval, &pfltval[num], varFArray.data.begin());
              SafeArrayUnaccessData(vntRet->parray);
              m_pubFloat32MultiArrayValue->publish(varFArray);
              break;
            case (VT_ARRAY | VT_R8):
              num = vntRet->parray->rgsabound->cElements;
              SafeArrayAccessData(vntRet->parray, (void**)&pdblval);
              varDArray.data.resize(num);
              std::copy(pdblval, &pdblval[num], varDArray.data.begin());
              SafeArrayUnaccessData(vntRet->parray);
              m_pubFloat64MultiArrayValue->publish(varDArray);
              break;
          }
        }
      }

      m_pubTimePrev = pubTimeCur;
    }
  }

  return true;
}

HRESULT DensoVariable::ExecGetValue(VARIANT_Ptr& value)
{
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntHandle(new VARIANT());

  VariantInit(vntHandle.get());

  vntHandle->vt = VT_UI4;
  vntHandle->ulVal = m_vecHandle[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  return m_vecService[DensoBase::SRV_WATCH]->ExecFunction(ID_VARIABLE_GETVALUE, vntArgs, value);
}

HRESULT DensoVariable::ExecPutValue(const VARIANT_Ptr& value)
{
  HRESULT hr;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntHandle(new VARIANT());
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  vntHandle->vt = VT_UI4;
  vntHandle->ulVal = m_vecHandle[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  vntArgs.push_back(*value.get());

  hr = m_vecService[DensoBase::SRV_WATCH]->ExecFunction(ID_VARIABLE_PUTVALUE, vntArgs, vntRet);
  if (SUCCEEDED(hr)) {
    Update();
  }

  return hr;
}

HRESULT DensoVariable::ExecPutID(const int id)
{
  HRESULT hr;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntHandle(new VARIANT());
  VARIANT_Ptr vntValue(new VARIANT());
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  vntHandle->vt = VT_UI4;
  vntHandle->ulVal = m_vecHandle[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  vntValue->vt = VT_I4;
  vntValue->lVal = id;
  vntArgs.push_back(*vntValue.get());

  hr = m_vecService[DensoBase::SRV_WATCH]->ExecFunction(ID_VARIABLE_PUTID, vntArgs, vntRet);
  if (SUCCEEDED(hr)) {
    Update();
  }

  return hr;
}

void DensoVariable::Callback_I32(const std_msgs::msg::Int32::SharedPtr msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_I4;
  vntVal->lVal = msg->data;

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_F32(const std_msgs::msg::Float32::SharedPtr msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_R4;
  vntVal->fltVal = msg->data;

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_F64(const std_msgs::msg::Float64::SharedPtr msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_R8;
  vntVal->dblVal = msg->data;

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_String(const std_msgs::msg::String::SharedPtr msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_BSTR;
  vntVal->bstrVal = ConvertStringToBSTR(msg->data);

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_Bool(const std_msgs::msg::Bool::SharedPtr msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_BOOL;
  vntVal->boolVal = (msg->data != 0) ? VARIANT_TRUE : VARIANT_FALSE;

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_F32Array(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  float* pval;

  vntVal->vt = (VT_ARRAY | VT_R4);
  vntVal->parray = SafeArrayCreateVector(VT_R4, 0, msg->data.size());

  SafeArrayAccessData(vntVal->parray, (void**)&pval);
  std::copy(msg->data.begin(), msg->data.end(), pval);
  SafeArrayUnaccessData(vntVal->parray);

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_F64Array(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  double* pval;

  vntVal->vt = (VT_ARRAY | VT_R8);
  vntVal->parray = SafeArrayCreateVector(VT_R8, 0, msg->data.size());

  SafeArrayAccessData(vntVal->parray, (void**)&pval);
  std::copy(msg->data.begin(), msg->data.end(), pval);
  SafeArrayUnaccessData(vntVal->parray);

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_ID(const std_msgs::msg::Int32::SharedPtr msg)
{
  ExecPutID(msg->data);
}

}  // namespace denso_robot_core
