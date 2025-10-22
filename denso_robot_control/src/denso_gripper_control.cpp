#include "denso_robot_control/denso_gripper_control.hpp"

namespace denso_gripper_control {
    DensoGripperControl::DensoGripperControl() : Node("denso_gripper_control") {
        bcap_client_ = this->create_client<bcap_service_interfaces::srv::Bcap>("/cobotta/bcap_service");
        while (!bcap_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for BCAP service to be available...");
        }
        // prepare bcap connect request
        bcap_request->func_id = 3;
        bcap_request->vnt_args.resize(4);
        bcap_request->vnt_args[0].vt = 8;
        bcap_request->vnt_args[0].value = "b-CAP";
        bcap_request->vnt_args[1].vt = 8;
        bcap_request->vnt_args[1].value = "CaoProv.DENSO.VRC";
        bcap_request->vnt_args[2].vt = 8;
        bcap_request->vnt_args[2].value = "localhost";
        bcap_request->vnt_args[3].vt = 8;
        bcap_request->vnt_args[3].value = "";

        

        RCLCPP_INFO(this->get_logger(), "Sending BCAP connect request...");
        auto future = bcap_client_->async_send_request(bcap_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto bcap_response = future.get();
            robot_id_ = bcap_response->vnt_ret.value;
            if (robot_id_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "BCAP connect failed. Please relaunch the bcap_service launch file");
                exit(1);
                return;
            }
            else {
                RCLCPP_INFO(this->get_logger(), "BCAP connected. Robot ID: %s",
                            robot_id_.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to BCAP service.");
        }

        // prepare gripper request
        gripper_request->func_id = 17;
        gripper_request->vnt_args.resize(3);
        gripper_request->vnt_args[0].vt = 19;
        gripper_request->vnt_args[0].value = robot_id_;
        gripper_request->vnt_args[1].vt = 8;
        gripper_request->vnt_args[1].value = "HandMoveA";
        gripper_request->vnt_args[2].vt = 8195;  // VT_UI1
        gripper_request->vnt_args[2].value = "";  // Placeholder for gripper position

        gripper_service_ = this->create_service<bcap_service_interfaces::srv::Gripper>(
            "gripper_service", std::bind(&DensoGripperControl::on_gripper_service, this,
                                         std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Denso Gripper Control Node has been started.");
    }

    DensoGripperControl::~DensoGripperControl() {
        RCLCPP_INFO(this->get_logger(), "Denso Gripper Control Node has been stopped.");
    }

    void DensoGripperControl::on_gripper_service(
        const std::shared_ptr<bcap_service_interfaces::srv::Gripper::Request> request,
        std::shared_ptr<bcap_service_interfaces::srv::Gripper::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Gripper service called with value: %d, speed: %d", request->value, request->speed);
        gripper_request->vnt_args[2].value = std::to_string(request->value) + "," + std::to_string(request->speed);
        auto future = bcap_client_->async_send_request(gripper_request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto bcap_response = future.get();
        }
        response->success = true;
        response->message = "Gripper moved successfully.";
        RCLCPP_INFO(this->get_logger(), "Gripper moved successfully.");
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<denso_gripper_control::DensoGripperControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}