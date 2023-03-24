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

#ifndef DENSO_ROBOT_CONTROL__DENSO_ROBOT_HW_HPP_
#define DENSO_ROBOT_CONTROL__DENSO_ROBOT_HW_HPP_


#include "denso_robot_control/denso_robot_control.hpp"

// // System
// #include <memory>
// #include <string>
// #include <vector>
// #include <limits>

// #include <boost/thread.hpp>

// // ros2_control hardware_interface
// #include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/types/hardware_interface_status_values.hpp"
// #include "hardware_interface/visibility_control.h"
// // ROS2
// #include "rclcpp/macros.hpp"
// // Message (std_msgs)
// #include "std_msgs/msg/u_int32.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"
// // DENSO libraries
// #include "denso_robot_core/denso_robot_core.h"
// #include "denso_robot_core/denso_controller.h"
// #include "denso_robot_core/denso_robot.h"
// #include "denso_robot_core/denso_variable.h"
// #include "denso_robot_core_interfaces/msg/user_io.hpp"

// using hardware_interface::HardwareInfo;
// using hardware_interface::return_type;
// using hardware_interface::status;
// using namespace std_msgs;
// using namespace denso_robot_core;


namespace denso_robot_control {

class DensoRobotHW : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DensoRobotHW)

  HRESULT Initialize();

  std::string get_name() const override
  {
    return info_.name;
  }

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void SpinNode(rclcpp::Node::SharedPtr& node, DensoRobotControl_Ptr drobo);

  HardwareInfo info_;
  // Store the commands for the real robot
  std::vector<double> cmd_interface_;
  std::vector<double> pos_interface_;
  std::vector<double> vel_interface_;
  std::vector<double> eff_interface_;

private:
  DensoRobotControl_Ptr drobo_;

  std::mutex mtx_mode_;

};

}  // namespace denso_robot_control

#endif  // DENSO_ROBOT_CONTROL__DENSO_ROBOT_HW_HPP_
