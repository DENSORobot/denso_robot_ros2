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
static const std::string kRobotNameHsr065 = "hsr065";

class DensoRobotCppDemo
{
public:
  DensoRobotCppDemo(const rclcpp::Node::SharedPtr& node)
  : node_(node), robotStatePublisher_(
      node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
    node_->declare_parameter("model", "cobotta");
    node_->declare_parameter("scale_factor", "1.0");
  }

  void run()
  {
    RCLCPP_INFO(kLogger, "***** Initializing DensoRobotCppDemo ...");
    if(!node_->get_parameter("scale_factor", scaleFactor_)) {
      scaleFactor_ = 0.1;
    }
    RCLCPP_INFO(kLogger, "***** DensoRobotCppDemo - Scale factor: %.2f", scaleFactor_);

    if(!node_->get_parameter("model", robotModel_)) {
      RCLCPP_FATAL(kLogger, "no model parameter specified !!");
      return;
    }
    RCLCPP_INFO(kLogger, "***** DensoRobotCppDemo - Robot model: %s", robotModel_.c_str());

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
    geometry_msgs::msg::Pose targetPos0, targetPos1, targetPos2, targetPos3;
    geometry_msgs::msg::Pose targetPos4, targetPos5, targetPos6, targetPos7;

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

      // Position #0: x-y-z in [m], orientation in quaternion format
      targetPos0.position.x = 0.180;
      targetPos0.position.y = -0.150;
      targetPos0.position.z = 0.170;
      targetPos0.orientation.x = 0;
      targetPos0.orientation.y = 1;
      targetPos0.orientation.z = 0;
      targetPos0.orientation.w = 0;
      // Position #1
      targetPos1.position.x = 0.180;
      targetPos1.position.y = -0.150;
      targetPos1.position.z = 0.195;
      targetPos1.orientation.x = 0;
      targetPos1.orientation.y = 1;
      targetPos1.orientation.z = 0;
      targetPos1.orientation.w = 0;
      // Position #2
      targetPos2.position.x = 0.180;
      targetPos2.position.y = 0.150;
      targetPos2.position.z = 0.195;
      targetPos2.orientation.x = 0;
      targetPos2.orientation.y = 1;
      targetPos2.orientation.z = 0;
      targetPos2.orientation.w = 0;
      // Position #3
      targetPos3.position.x = 0.180;
      targetPos3.position.y = 0.150;
      targetPos3.position.z = 0.170;
      targetPos3.orientation.x = 0;
      targetPos3.orientation.y = 1;
      targetPos3.orientation.z = 0;
      targetPos3.orientation.w = 0;
      // Position #4
      targetPos4.position.x = 0.190;
      targetPos4.position.y = 0.150;
      targetPos4.position.z = 0.170;
      targetPos4.orientation.x = 0;
      targetPos4.orientation.y = 1;
      targetPos4.orientation.z = 0;
      targetPos4.orientation.w = 0;
      // Position #5
      targetPos5.position.x = 0.190;
      targetPos5.position.y = 0.150;
      targetPos5.position.z = 0.195;
      targetPos5.orientation.x = 0;
      targetPos5.orientation.y = 1;
      targetPos5.orientation.z = 0;
      targetPos5.orientation.w = 0;
      // Position #6
      targetPos6.position.x = 0.190;
      targetPos6.position.y = -0.150;
      targetPos6.position.z = 0.195;
      targetPos6.orientation.x = 0;
      targetPos6.orientation.y = 1;
      targetPos6.orientation.z = 0;
      targetPos6.orientation.w = 0;
      // Position #7
      targetPos7.position.x = 0.190;
      targetPos7.position.y = -0.150;
      targetPos7.position.z = 0.170;
      targetPos7.orientation.x = 0;
      targetPos7.orientation.y = 1;
      targetPos7.orientation.z = 0;
      targetPos7.orientation.w = 0;
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

        // Position #0: x-y-z in [m], orientation in quaternion format
        targetPos0.position.x = -0.1;
        targetPos0.position.y = 0.400;
        targetPos0.position.z = 0.400;
        targetPos0.orientation.x = 0.7071068;
        targetPos0.orientation.y = -0.7071068;
        targetPos0.orientation.z = 0;
        targetPos0.orientation.w = 0;
        // Position #1
        targetPos1.position.x = -0.1;
        targetPos1.position.y = 0.400;
        targetPos1.position.z = 0.425;
        targetPos1.orientation.x = 0.7071068;
        targetPos1.orientation.y = -0.7071068;
        targetPos1.orientation.z = 0;
        targetPos1.orientation.w = 0;
        // Position #2
        targetPos2.position.x = 0.2;
        targetPos2.position.y = 0.400;
        targetPos2.position.z = 0.425;
        targetPos2.orientation.x = 0.7071068;
        targetPos2.orientation.y = -0.7071068;
        targetPos2.orientation.z = 0;
        targetPos2.orientation.w = 0;
        // Position #3
        targetPos3.position.x = 0.2;
        targetPos3.position.y = 0.400;
        targetPos3.position.z = 0.400;
        targetPos3.orientation.x = 0.7071068;
        targetPos3.orientation.y = -0.7071068;
        targetPos3.orientation.z = 0;
        targetPos3.orientation.w = 0;
        // Position #4
        targetPos4.position.x = 0.200;
        targetPos4.position.y = 0.500;
        targetPos4.position.z = 0.400;
        targetPos4.orientation.x = 0.7071068;
        targetPos4.orientation.y = -0.7071068;
        targetPos4.orientation.z = 0;
        targetPos4.orientation.w = 0;
        // Position #5
        targetPos5.position.x = 0.200;
        targetPos5.position.y = 0.500;
        targetPos5.position.z = 0.425;
        targetPos5.orientation.x = 0.7071068;
        targetPos5.orientation.y = -0.7071068;
        targetPos5.orientation.z = 0;
        targetPos5.orientation.w = 0;
        // Position #6
        targetPos6.position.x = -0.100;
        targetPos6.position.y = 0.500;
        targetPos6.position.z = 0.425;
        targetPos6.orientation.x = 0.7071068;
        targetPos6.orientation.y = -0.7071068;
        targetPos6.orientation.z = 0;
        targetPos6.orientation.w = 0;
        // Position #7
        targetPos7.position.x = -0.1;
        targetPos7.position.y = 0.500;
        targetPos7.position.z = 0.400;
        targetPos7.orientation.x = 0.7071068;
        targetPos7.orientation.y = -0.7071068;
        targetPos7.orientation.z = 0;
        targetPos7.orientation.w = 0;
        // End Positions
      } else {
        robotModel = kRobotNameHsr065;
        if (std::equal(robotModel.begin(),robotModel.end(),
          robotModel_.begin(), robotModel_.begin() + robotModel.length()))
        {
          // hsr065 robot
          RCLCPP_INFO(
            kLogger,
            "***** WARNING: HSR robot model selected !! "
            "Be sure that a proper IK plugin is installed,"
            " otherwise the cartesian movements will fail !!");

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

          // Position #0: x-y-z in [m], orientation in quaternion format
          targetPos0.position.x = 0.4;
          targetPos0.position.y = -0.2;
          targetPos0.position.z = 0.2;
          targetPos0.orientation.x = 0;
          targetPos0.orientation.y = 0;
          targetPos0.orientation.z = 0;
          targetPos0.orientation.w = 1;
          // Position #1
          targetPos1.position.x = 0.4;
          targetPos1.position.y = -0.2;
          targetPos1.position.z = 0.1;
          targetPos1.orientation.x = 0;
          targetPos1.orientation.y = 0;
          targetPos1.orientation.z = 0;
          targetPos1.orientation.w = 1;
          // Position #2
          targetPos2.position.x = 0.6;
          targetPos2.position.y = -0.2;
          targetPos2.position.z = 0.1;
          targetPos2.orientation.x = 0;
          targetPos2.orientation.y = 0;
          targetPos2.orientation.z = 0;
          targetPos2.orientation.w = 1;
          // Position #3
          targetPos3.position.x = 0.6;
          targetPos3.position.y = -0.2;
          targetPos3.position.z = 0.3;
          targetPos3.orientation.x = 0.7071068;
          targetPos3.orientation.y = -0.7071068;
          targetPos3.orientation.z = 0;
          targetPos3.orientation.w = 0;
          // Position #4
          targetPos4.position.x = 0.6;
          targetPos4.position.y = 0.2;
          targetPos4.position.z = 0.2;
          targetPos4.orientation.x = 0;
          targetPos4.orientation.y = 0;
          targetPos4.orientation.z = 0;
          targetPos4.orientation.w = 1;
          // Position #5
          targetPos5.position.x = 0.6;
          targetPos5.position.y = 0.2;
          targetPos5.position.z = 0.1;
          targetPos5.orientation.x = 0;
          targetPos5.orientation.y = 0;
          targetPos5.orientation.z = 0;
          targetPos5.orientation.w = 1;
          // Position #6
          targetPos6.position.x = 0.4;
          targetPos6.position.y = 0.2;
          targetPos6.position.z = 0.1;
          targetPos6.orientation.x = 0;
          targetPos6.orientation.y = 0;
          targetPos6.orientation.z = 0;
          targetPos6.orientation.w = 1;
          // Position #7
          targetPos7.position.x = 0.4;
          targetPos7.position.y = 0.2;
          targetPos7.position.z = 0.2;
          targetPos7.orientation.x = 0;
          targetPos7.orientation.y = 0;
          targetPos7.orientation.z = 0;
          targetPos7.orientation.w = 1;
          // End Positions
        } else {
        RCLCPP_FATAL(kLogger, "ERROR: positions for the specified robot model not implemented !!");
        return;
        }
      }
    }

    moveit::planning_interface::MoveGroupInterface::Plan movePlan;

    // Planning to Pose goal #0
    moveGroup.setPoseTarget(targetPos0);
    bool success = (
      moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(kLogger, "***** Plan to pose goal #0 %s", success ? "SUCCEEDED" : "FAILED");
    // Moving to pose goal #0
    moveGroup.move();

    // Planning to Pose goal #1
    moveGroup.setPoseTarget(targetPos1);
    success = (moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(kLogger, "***** Plan to pose goal #1 %s", success ? "SUCCEEDED" : "FAILED");
    // Moving to pose goal #1
    moveGroup.move();

    // Planning to Pose goal #2
    moveGroup.setPoseTarget(targetPos2);
    success = (moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(kLogger, "***** Plan to pose goal #2 %s", success ? "SUCCEEDED" : "FAILED");
    // Moving to pose goal #2
    moveGroup.move();

    // Planning to a Pose goal #3
    moveGroup.setPoseTarget(targetPos3);
    success = (moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(kLogger, "***** Plan to pose goal #3 %s", success ? "SUCCEEDED" : "FAILED");
    // Moving to a pose goal #3
    moveGroup.move();

    // Planning to a Pose goal #4
    moveGroup.setPoseTarget(targetPos4);
    success = (moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(kLogger, "***** Plan to pose goal #4 %s", success ? "SUCCEEDED" : "FAILED");
    // Moving to a pose goal #4
    moveGroup.move();

    // Planning to a Pose goal #5
    moveGroup.setPoseTarget(targetPos5);
    success = (moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(kLogger, "***** Plan to pose goal #5 %s", success ? "SUCCEEDED" : "FAILED");
    // Moving to a pose goal #5
    moveGroup.move();

    // Planning to a Pose goal #6
    moveGroup.setPoseTarget(targetPos6);
    success = (moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(kLogger, "***** Plan to pose goal #6 %s", success ? "SUCCEEDED" : "FAILED");
    // Moving to a pose goal #6
    moveGroup.move();

    // Planning to a Pose goal #7
    moveGroup.setPoseTarget(targetPos7);
    success = (moveGroup.plan(movePlan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(kLogger, "***** Plan to pose goal #7 %s", success ? "SUCCEEDED" : "FAILED");
    // Moving to a pose goal #7
    moveGroup.move();

    RCLCPP_INFO(kLogger, "***** Cycle ended !!");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robotStatePublisher_;
  double scaleFactor_;
  std::string robotModel_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(kLogger, "***** Initializing node ...");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions nodeOptions;

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("denso_robot_moveit_demo", "", nodeOptions);

  DensoRobotCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Sleep 5 seconds before running demo
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();
  return 0;
}
