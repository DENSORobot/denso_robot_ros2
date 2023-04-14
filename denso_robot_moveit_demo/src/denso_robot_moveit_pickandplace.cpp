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
static const std::string kRobotNameCobotta = "cobotta";
static const std::string kRobotNameVs060 = "vs060";

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
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

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
    geometry_msgs::msg::Pose approachPickPos, targetPickPos, approachPlacePos, targetPlacePos;

    std::string robotModel = kRobotNameCobotta;
    if (
      std::equal(robotModel.begin(),robotModel.end(),
        robotModel_.begin(), robotModel_.begin() + robotModel.length()))
    {
      // cobotta robot
      // Home position in joint values
      // joint 1 [rad], joint 2 [rad], joint 3 [rad], joint 4 [rad], joint 5 [rad], joint 6 [rad]
      std::vector<double> jointGroupHomeCobotta = {
        0, -0.703146527727, 2.18346103477, 0, 1.56990645489, 0};
      moveGroup.setJointValueTarget(jointGroupHomeCobotta);

      // Call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking moveGroup to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan movePlanHomeCobotta;

      bool successHomeCobotta = (
        moveGroup.plan(movePlanHomeCobotta) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger, "***** Plan to Home (joint space goal) %s",
        successHomeCobotta ? "SUCCEEDED" : "FAILED");
      moveGroup.move();

      // Approach Position for Pick Operation: x-y-z in [m], orientation in quaternion format
      approachPickPos.position.x = 0.180;
      approachPickPos.position.y = -0.150;
      approachPickPos.position.z = 0.195;
      approachPickPos.orientation.x = 0;
      approachPickPos.orientation.y = 1;
      approachPickPos.orientation.z = 0;
      approachPickPos.orientation.w = 0;
      // Target Position for Pick Operation
      targetPickPos.position.x = 0.180;
      targetPickPos.position.y = -0.150;
      targetPickPos.position.z = 0.170;
      targetPickPos.orientation.x = 0;
      targetPickPos.orientation.y = 1;
      targetPickPos.orientation.z = 0;
      targetPickPos.orientation.w = 0;
      // Approach Position for Place Operation
      approachPlacePos.position.x = 0.180;
      approachPlacePos.position.y = 0.150;
      approachPlacePos.position.z = 0.195;
      approachPlacePos.orientation.x = 0;
      approachPlacePos.orientation.y = 1;
      approachPlacePos.orientation.z = 0;
      approachPlacePos.orientation.w = 0;
      // Target Position for Place Operation
      targetPlacePos.position.x = 0.180;
      targetPlacePos.position.y = 0.150;
      targetPlacePos.position.z = 0.170;
      targetPlacePos.orientation.x = 0;
      targetPlacePos.orientation.y = 1;
      targetPlacePos.orientation.z = 0;
      targetPlacePos.orientation.w = 0;
      // End Positions
    } else {
      robotModel = kRobotNameVs060;
      if (std::equal(robotModel.begin(),robotModel.end(),
        robotModel_.begin(), robotModel_.begin() + robotModel.length()))
      {
        // vs060 robot
        // Home position in joint values
        // joint 1 [rad], joint 2 [rad], joint 3 [rad], joint 4 [rad], joint 5 [rad], joint 6 [rad]
        std::vector<double> jointGroupHomeVs060 = {
          1.56990645, 0, 1.56990645, 0, 1.56990645489, 0};
        moveGroup.setJointValueTarget(jointGroupHomeVs060);

        // Call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking moveGroup to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan movePlanHomeVs060;

        bool successHomeVs060 = (
          moveGroup.plan(movePlanHomeVs060) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(
          kLogger, "***** Plan to Home (joint space goal) %s",
          successHomeVs060 ? "SUCCEEDED" : "FAILED");
        moveGroup.move();

        // Approach Position for Pick Operation: x-y-z in [m], orientation in quaternion format
        approachPickPos.position.x = 0.2;
        approachPickPos.position.y = -0.4;
        approachPickPos.position.z = 0.5;
        approachPickPos.orientation.x = 0;
        approachPickPos.orientation.y = 1;
        approachPickPos.orientation.z = 0;
        approachPickPos.orientation.w = 0;
        // Target Position for Pick Operation
        targetPickPos.position.x = 0.2;
        targetPickPos.position.y = -0.4;
        targetPickPos.position.z = 0.3;
        targetPickPos.orientation.x = 0;
        targetPickPos.orientation.y = 1;
        targetPickPos.orientation.z = 0;
        targetPickPos.orientation.w = 0;
        // Approach Position for Place Operation
        approachPlacePos.position.x = 0.2;
        approachPlacePos.position.y = 0.4;
        approachPlacePos.position.z = 0.5;
        approachPlacePos.orientation.x = 0;
        approachPlacePos.orientation.y = 1;
        approachPlacePos.orientation.z = 0;
        approachPlacePos.orientation.w = 0;
        // Target Position for Place Operation
        targetPlacePos.position.x = 0.2;
        targetPlacePos.position.y = 0.4;
        targetPlacePos.position.z = 0.3;
        targetPlacePos.orientation.x = 0;
        targetPlacePos.orientation.y = 1;
        targetPlacePos.orientation.z = 0;
        targetPlacePos.orientation.w = 0;
        // End Positions
      } else {
        RCLCPP_FATAL(kLogger, "ERROR: positions for the specified robot model not implemented !!");
        return;
      }
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
      moveGroup.setPoseTarget(approachPickPos);
      bool success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger,
        "***** Plan to approach picking pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving to Approach Position for Pick Operation
      moveGroup.move();

      // Planning to Target Position for Pick Operation
      moveGroup.setPoseTarget(targetPickPos);
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
      moveGroup.setPoseTarget(approachPickPos);
      success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger,
        "***** Plan to approach picking pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving back to Approach Position for Pick Operation
      moveGroup.move();

      // Planning to Approach Position for Place Operation
      moveGroup.setPoseTarget(approachPlacePos);
      success = (
        moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(
        kLogger,
        "***** Plan to approach placing pose %s", success ? "SUCCEEDED" : "FAILED");
      // Moving to Approach Position for Place Operation
      moveGroup.move();

      // Planning to Target Position for Place Operation
      moveGroup.setPoseTarget(targetPlacePos);
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
      moveGroup.setPoseTarget(approachPlacePos);
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
