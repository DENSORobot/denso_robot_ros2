/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, DENSO WAVE INCORPORATED
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*********************************************************************
Author: DENSO WAVE INCORPORATED
Description: A simple demo node running MoveItCpp for planning and execution
                   of a pick-and-place cycle
*********************************************************************/

#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger kLogger = rclcpp::get_logger("denso_robot_moveit_demo");
static const std::string kPlanningGroup = "arm";
static const std::string kRobotNameHsr065 = "hsr065";

class DensoRobotPickAndPlaceDemo
{
public:
  DensoRobotPickAndPlaceDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node), robotStatePublisher_(
      node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
    node_->declare_parameter("model", "");
    node_->declare_parameter("scale_factor", "");
    node_->declare_parameter("num_cycles", "");
  }

  void run()
  {
    RCLCPP_INFO(kLogger, "***** Initializing DensoRobotPickAndPlaceDemo ...");
    if(!node_->get_parameter("scale_factor", scaleFactor_)) {
      scaleFactor_ = 0.1;
    }
    RCLCPP_INFO(kLogger, "***** DensoRobotPickAndPlaceDemo - Scale factor: %.2f", scaleFactor_);

    if(!node_->get_parameter("num_cycles", numCycles_)) {
      numCycles_ = 1;
    }
    RCLCPP_INFO(kLogger, "***** DensoRobotPickAndPlaceDemo - Number of cycles: %d", numCycles_);

    if(!node_->get_parameter("model", robotModel_)) {
      RCLCPP_FATAL(kLogger, "no model parameter specified !!");
      return;
    }
    RCLCPP_INFO(kLogger, "***** DensoRobotPickAndPlaceDemo - Robot model: %s", robotModel_.c_str());

    moveit::planning_interface::MoveGroupInterface moveGroup(node_, kPlanningGroup);
    RCLCPP_INFO(kLogger, "***** Initializing MoveIt Components ...");

    moveGroup.setMaxVelocityScalingFactor(scaleFactor_);
    moveGroup.setMaxAccelerationScalingFactor(scaleFactor_);

    // Print the name of the reference frame for this robot.
    RCLCPP_INFO(kLogger, "***** Planning frame: %s", moveGroup.getPlanningFrame().c_str());

    // Print the name of the end-effector link for this group.
    RCLCPP_INFO(kLogger, "***** End effector link: %s", moveGroup.getEndEffectorLink().c_str());

    // Get and print a list of all the groups in the robot:
    RCLCPP_INFO(kLogger, "***** Available Planning Groups:");
    std::copy(
      moveGroup.getJointModelGroupNames().begin(), moveGroup.getJointModelGroupNames().end(),
      std::ostream_iterator<std::string>(std::cout, ", "));

    // A little delay before starting the movements
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Planning to a Pose goal
    std::vector<double> approachPickPos, targetPickPos, approachPlacePos, targetPlacePos;

    std::string robotModel = kRobotNameHsr065;
    if (
      std::equal(robotModel.begin(),robotModel.end(),
        robotModel_.begin(), robotModel_.begin() + robotModel.length()))
    {
      // hsr065 robot
      // Home position in joint values
      // joint 1 [rad] , joint 2 [rad], joint 3 [m], joint 4 [rad]
      std::vector<double> jointGroupHomeHsr065 = {
        0.7, -1.75, 0.1, 0};
      moveGroup.setJointValueTarget(jointGroupHomeHsr065);

      // Call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking moveGroup to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan movePlanHomeHsr065;

      bool successHomeHsr065 = (
        moveGroup.plan(movePlanHomeHsr065) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger, "***** Plan to Home (joint space goal) %s",
        successHomeHsr065 ? "SUCCEEDED" : "FAILED");
      moveGroup.move();

      // Approach Position for Pick Operation in joint space
      approachPickPos.resize(4);
      approachPickPos[0] = -0.59; // joint1 [rad]
      approachPickPos[1] = -1.9; // joint2 [rad]
      approachPickPos[2] = 0.18; // joint3 [m]
      approachPickPos[3] = 2.48; // joint4 [rad]
      // Target Position for Pick Operation
      targetPickPos.resize(4);
      targetPickPos[0] = -0.59;
      targetPickPos[1] = -1.9;
      targetPickPos[2] = 0;
      targetPickPos[3] = 2.48;
      // Approach Position for Place Operation
      approachPlacePos.resize(4);
      approachPlacePos[0] = 1.43;
      approachPlacePos[1] = -1.31;
      approachPlacePos[2] = 0.18;
      approachPlacePos[3] = 0;
      // Target Position for Place Operation
      targetPlacePos.resize(4);
      targetPlacePos[0] = 1.43;
      targetPlacePos[1] = -1.31;
      targetPlacePos[2] = 0;
      targetPlacePos[3] = 0;
      // End Positions
    } else {
      RCLCPP_FATAL(kLogger, "ERROR: positions for the specified robot model not implemented !!");
      return;
    }

    geometry_msgs::msg::Pose currentPose = moveGroup.getCurrentPose().pose;
    RCLCPP_INFO(kLogger, "***** Initial robot pose: *****");
    RCLCPP_INFO(kLogger, "***** X: %.3f *****", currentPose.position.x);
    RCLCPP_INFO(kLogger, "***** Y: %.3f *****", currentPose.position.y);
    RCLCPP_INFO(kLogger, "***** Z: %.3f *****", currentPose.position.z);
    RCLCPP_INFO(kLogger, "***** QX: %.3f *****", currentPose.orientation.x);
    RCLCPP_INFO(kLogger, "***** QY: %.3f *****", currentPose.orientation.y);
    RCLCPP_INFO(kLogger, "***** QZ: %.3f *****", currentPose.orientation.z);
    RCLCPP_INFO(kLogger, "***** QW: %.3f *****", currentPose.orientation.w);

    moveit::planning_interface::MoveGroupInterface::Plan movePlan;

    // ********************************************************
    // Place here code for pick-and-place initialization
    // (e.g. open gripper / vacuum OFF)
    // ********************************************************

    for(int i=0; i < numCycles_; i++) {
      RCLCPP_INFO(kLogger, "***** Starting cycle#%d ...", i+1);

      // Planning to Approach Position for Pick Operation
      moveGroup.setJointValueTarget(approachPickPos);
      bool success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger,
        "***** Plan to approach picking pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving to Approach Position for Pick Operation
      moveGroup.move();

      // Planning to Target Position for Pick Operation
      moveGroup.setJointValueTarget(targetPickPos);
      success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger,
        "***** Plan to target picking pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving to Target Position for Pick Operation
      moveGroup.move();

      // ********************************************************
      // Place here code for pick operation
      // (e.g. close gripper / vacuum ON)
      // ********************************************************

      // Planning to Approach Position for Pick Operation
      moveGroup.setJointValueTarget(approachPickPos);
      success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger,
        "***** Plan to approach picking pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving back to Approach Position for Pick Operation
      moveGroup.move();

      // Planning to Approach Position for Place Operation
      moveGroup.setJointValueTarget(approachPlacePos);
      success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger,
        "***** Plan to approach placing pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving to Approach Position for Place Operation
      moveGroup.move();

      // Planning to Target Position for Place Operation
      moveGroup.setJointValueTarget(targetPlacePos);
      success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger, "***** Plan to target placing pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving to Target Position for Place Operation
      moveGroup.move();

      // ********************************************************
      // Place here code for place operation
      // (e.g. open gripper / vacuum OFF)
      // ********************************************************

      // Planning to Approach Position for Place Operation
      moveGroup.setJointValueTarget(approachPlacePos);
      success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger,
        "***** Plan to approach placing pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving back to Approach Position for Place Operation
      moveGroup.move();

      RCLCPP_INFO(kLogger, "***** Cycle#%d ended ...", i+1);
    }

    RCLCPP_INFO(kLogger, "***** Cycles ended !!");
    return;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robotStatePublisher_;
  int numCycles_;
  double scaleFactor_;
  std::string robotModel_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(kLogger, "***** Initializing node ...");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions nodeOptions;

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
    "denso_robot_moveit_demo", "", nodeOptions);

  DensoRobotPickAndPlaceDemo demo(node);
  std::thread run_demo([&demo]() {
    // Sleep 5 seconds before running demo
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();
  return 0;
}
