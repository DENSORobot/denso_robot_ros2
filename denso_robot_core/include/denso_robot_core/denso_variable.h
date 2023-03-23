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

#ifndef DENSO_VARIABLE_H
#define DENSO_VARIABLE_H

#include "denso_robot_core/denso_base.h"

namespace denso_robot_core {
class DensoVariable : public DensoBase
{
public:
  static constexpr const char* NAME_READ = "_Read";
  static constexpr const char* NAME_WRITE = "_Write";
  static constexpr const char* NAME_ID = "_ID";

  static constexpr const char* XML_VARIABLE_NAME = "Variable";

  static constexpr const char* XML_ATTR_VARTYPE = "vt";
  static constexpr const char* XML_ATTR_READ = "read";
  static constexpr const char* XML_ATTR_WRITE = "write";
  static constexpr const char* XML_ATTR_ID = "id";
  static constexpr const char* XML_ATTR_DURATION = "duration";

  DensoVariable(
    DensoBase* parent, Service_Vec& service, Handle_Vec& handle, const std::string& name,
    const int* mode, int16_t vt, bool Read, bool Write, bool ID, int Duration);

  virtual ~DensoVariable();

  HRESULT StartService(rclcpp::Node::SharedPtr& node);
  HRESULT StopService();

  bool Update();

  HRESULT ExecGetValue(VARIANT_Ptr& value);
  HRESULT ExecPutValue(const VARIANT_Ptr& value);
  HRESULT ExecPutID(const int id);

private:
  // Callback functions
  void Callback_I32(const std_msgs::msg::Int32::SharedPtr msg);
  void Callback_F32(const std_msgs::msg::Float32::SharedPtr msg);
  void Callback_F64(const std_msgs::msg::Float64::SharedPtr msg);
  void Callback_String(const std_msgs::msg::String::SharedPtr msg);
  void Callback_Bool(const std_msgs::msg::Bool::SharedPtr msg);
  void Callback_F32Array(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void Callback_F64Array(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void Callback_ID(const std_msgs::msg::Int32::SharedPtr msg);

private:
  int16_t m_vt;
  bool m_bRead;
  bool m_bWrite;
  bool m_bID;
  double m_Duration;
  rclcpp::Time m_pubTimePrev;

  rclcpp::Clock m_rosClock;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_pubInt32Value;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubFloat32Value;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_pubFloat64Value;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pubStringValue;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_pubBoolValue;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_pubFloat32MultiArrayValue;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_pubFloat64MultiArrayValue;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subInt32Value;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subFloat32Value;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_subFloat64Value;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subStringValue;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subBoolValue;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_subFloat32MultiArrayValue;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_subFloat64MultiArrayValue;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subID;
};

typedef std::shared_ptr<DensoVariable> DensoVariable_Ptr;
typedef std::vector<DensoVariable_Ptr> DensoVariable_Vec;

}  // namespace denso_robot_core

#endif  // DENSO_VARIABLE_H
