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

#ifndef DENSO_CONTROLLER_H
#define DENSO_CONTROLLER_H

#include "denso_robot_core/denso_base.h"
#include "denso_robot_core/denso_robot.h"
#include "denso_robot_core/denso_task.h"
#include "denso_robot_core/denso_variable.h"

namespace denso_robot_core {

class Version
{
private:
  // Define four member variables major, minor, revision and build
  int major, minor, revision, build;

public:
  // Parametarized constructor. Pass string to it and it will
  // extract version-tag from it.
  Version(const std::string& version)
  : major(0), minor(0), revision(0), build(0)
  {
    std::sscanf(version.c_str(), "%d.%d.%d.%d", &major, &minor, &revision, &build);

    // version-tag must be >=0, if it is less than zero, then make it zero.
    if (major < 0) {major = 0;}
    if (minor < 0) {minor = 0;}
    if (revision < 0) {revision = 0;}
    if (build < 0) {build = 0;}
  }

  // Overload "greater than" (>) operator to compare two version objects
  bool operator > (const Version& other)
  {
    // Start comparing version tag from left most

    // Compare major version-tag
    if (major > other.major) {
      return true;
    } else if (major < other.major) {
      return false;
    }

    // Compare minor version-tag
    // If control came here it means that above version-tag(s) are equal
    if (minor > other.minor) {
      return true;
    } else if (minor < other.minor) {
      return false;
    }

    // Compare revision version-tag
    // If control came here it means that above version-tag(s) are equal
    if (revision > other.revision) {
      return true;
    } else if (revision < other.revision) {
      return false;
    }

    // Compare build version-tag
    // If control came here it means that above version-tag(s) are equal
    if (build > other.build) {
      return true;
    } else if (build < other.build) {
      return false;
    }

    return false;
  }

  // Overload equal to(==) operator to compare two version
  bool operator == (const Version& other)
  {
    return (
      major == other.major && minor == other.minor
      && revision == other.revision && build == other.build);
  }
};


class DensoController : public DensoBase
{
public:
  static constexpr int BCAP_CONTROLLER_CONNECT_ARGS = 4;
  static constexpr int BCAP_CONTROLLER_EXECUTE_ARGS = 3;
  static constexpr const char * XML_CTRL_NAME = "Controller";

  virtual ~DensoController();
  virtual HRESULT InitializeBCAP(const std::string& filename);
  virtual HRESULT StartService(rclcpp::Node::SharedPtr& node);
  virtual HRESULT StopService();
  virtual bool Update();

  HRESULT get_Robot(int index, DensoRobot_Ptr * robot);
  HRESULT get_Task(const std::string& name, DensoTask_Ptr * task);
  HRESULT get_Variable(const std::string& name, DensoVariable_Ptr * var);
  HRESULT AddVariable(const std::string& name);

  virtual HRESULT ExecClearError();
  virtual HRESULT ExecResetStoState() = 0;
  virtual HRESULT ExecGetCurErrorCount(int& count);
  virtual HRESULT ExecGetCurErrorInfo(int error_index, HRESULT& error_code, std::string& error_message);
  virtual HRESULT ExecGetErrorDescription(HRESULT error_code, std::string& error_description);

  HRESULT GetControllerVersion(std::string& ctrl_version);

  rclcpp::Duration get_Duration() const
  {
    return m_duration;
  }

private:
  std::string m_addr;

protected:
  DensoController(
    rclcpp::Node::SharedPtr& node, const std::string& name, const int * mode,
    const std::string& ip_address, const rclcpp::Duration dt);
  virtual HRESULT AddController() = 0;
  virtual HRESULT AddRobot(XMLElement * xmlElem) = 0;
  virtual HRESULT AddTask(XMLElement * xmlElem);
  virtual HRESULT AddVariable(XMLElement * xmlElem);

protected:
  std::string m_ctrlVer;
  DensoRobot_Vec m_vecRobot;
  DensoTask_Vec m_vecTask;
  DensoVariable_Vec m_vecVar;
  rclcpp::Duration m_duration;

  // ROS2 Node Handle
  rclcpp::Node::SharedPtr& m_node;
};

typedef std::shared_ptr<DensoController> DensoController_Ptr;

}  // namespace denso_robot_core

#endif  // DENSO_CONTROLLER_H
