#include "motor/motor.h"

Motor::Motor() : Node("motor_controller") {
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    
    motor_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
        "/mavros/rc/override", 10);
    
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "mavros/cmd/arming");
    
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");
    
    arming_state_ = false;
}

bool Motor::arming(bool state) {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = state;

    while (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
    }

    auto result = arming_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->success) {
            RCLCPP_INFO(this->get_logger(), state ? "Armed successfully" : "Disarmed successfully");
            return true;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to arm/disarm");
    return false;
}

bool Motor::set_mode(const std::string& mode) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;

    while (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
    }

    auto result = set_mode_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "Mode set successfully to: %s", mode.c_str());
            return true;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to set mode");
    return false;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto motor = std::make_shared<Motor>();
    rclcpp::Rate rate(100);

    while (rclcpp::ok()) {
        rclcpp::spin_some(motor);
        motor->testing();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}