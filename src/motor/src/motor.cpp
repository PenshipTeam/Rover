#include "motor/motor.h"

Motor::Motor() : Node("motor_controller") {
    // local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    //     "mavros/setpoint_position/local", 10);
    
    motor_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
        "/mavros/rc/override", 10);
    
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "mavros/cmd/arming");
    
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");

    // state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    //     "/mavros/state", 10, std::bind(&Motor::state_calback, this, std::placeholders::_1));
    
    arming_state_ = false;
}

// void Motor::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
//     current_state_ = *msg;
// }

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

void Motor::testing() {
    static auto start_time = this->now();
    auto elapsed_time = (this->now() - start_time).second() * 1000;

    mavros_msgs::msg::OverrideRCIn rc_msg;

    if (elapsed_time <= 6000) {
        RCLCPP_INFO(this->get_logger(), "SESI 1: Arming and MANUAL mode");

        if (!arming_state_) {
            arming(true);
            arming_state_ = true;
        }

        set_mode("MANUAL");

        rc_msg.channels[2] = 12000;
        rc_msg.channels[0] = 1500;
        rc_msg.channels[1] = 1500;
    }
    else if (elapsed_time <= 20000) {
        RCLCPP_INFO(this->get_logger(), "SESI 2: Switching to ALT_HOLD mode");

        rc_msg.channels[2] = 1500;
        rc_msg.channels[0] = 1500;
        rc_msg.channels[1] = 1800;

        set_mode("ALT_HOLD");
    }
    motor_pub_->publish(rc_msg);
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