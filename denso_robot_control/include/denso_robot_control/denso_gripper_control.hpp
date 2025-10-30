#pragma once

#include <rclcpp/rclcpp.hpp>
#include <bcap_service_interfaces/msg/variant.hpp>
#include <bcap_service_interfaces/srv/bcap.hpp>
#include <bcap_service_interfaces/srv/gripper.hpp>

namespace denso_gripper_control {
    class DensoGripperControl : public rclcpp::Node {
    public:
        DensoGripperControl();
        ~DensoGripperControl();
    private:
        void on_gripper_service(const std::shared_ptr<bcap_service_interfaces::srv::Gripper::Request> request,
                                std::shared_ptr<bcap_service_interfaces::srv::Gripper::Response> response);
        rclcpp::Service<bcap_service_interfaces::srv::Gripper>::SharedPtr gripper_service_;
        rclcpp::Client<bcap_service_interfaces::srv::Bcap>::SharedPtr bcap_client_;
        std::shared_ptr<bcap_service_interfaces::srv::Bcap::Request> bcap_request = std::make_shared<bcap_service_interfaces::srv::Bcap::Request>();
        std::shared_ptr<bcap_service_interfaces::srv::Bcap::Request> gripper_request = std::make_shared<bcap_service_interfaces::srv::Bcap::Request>();
        std::string robot_id_;
    };
}