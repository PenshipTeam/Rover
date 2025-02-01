#include "main/main.h"

MainController::MainController() : Node("main_controller") {
    // Subscribe to cmd_vel from joy controller
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, 
        std::bind(&MainController::twist_callback, this, std::placeholders::_1));

    // Subscribe to mavros state
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10,
        std::bind(&MainController::state_callback, this, std::placeholders::_1));

    // Publisher for motor commands
    rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
        "mavros/rc/override", 10);

    armed_ = false;
    current_mode_ = "MANUAL";
    
    RCLCPP_INFO(this->get_logger(), "Main controller initialized");
}

void MainController::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
    armed_ = msg->armed;
    current_mode_ = msg->mode;
}

void MainController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!armed_) {
        return;
    }

    auto rc_msg = mavros_msgs::msg::OverrideRCIn();
    
    // Convert twist messages to RC values (1000-2000 range)
    rc_msg.channels[0] = 1500 + (msg->linear.y * 500);  // Roll (lateral)
    rc_msg.channels[1] = 1500 + (msg->linear.x * 500);  // Pitch (forward)
    rc_msg.channels[2] = 1500 + (msg->linear.z * 500);  // Throttle (vertical)
    
    // Publish RC override message
    rc_override_pub_->publish(rc_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}