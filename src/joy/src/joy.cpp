#include "joy/joy.h"

ROVJoyController::ROVJoyController() : Node("rov_joy_controller") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&ROVJoyController::joy_callback, this, std::placeholders::_1)
    );

    lateral_value_ = 1500;
    forward_value_ = 1500;
    throttle_value_ = 1500;
    armed_ = false;
    current_mode_ = "MANUAL";
}

void ROVJoyController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    bool start_pressed = joy_msg->buttons[7];
    bool cross_pressed = joy_msg->buttons[0];
    bool circle_pressed = joy_msg->buttons[1];
    bool triangle_pressed = joy_msg->buttons[2];
    bool square_pressed = joy_msg->buttons[3];
    float l2_analog = joy_msg->axes[2];
    float r2_analog = joy_msg->axes[5];
    float left_analog_y = joy_msg->axes[1];  // Stick kiri atas/bawah
    float left_analog_x = joy_msg->axes[0];  // Stick kiri kiri/kanan

    if (start_pressed) {
        armed_ = true;
        RCLCPP_INFO(this->get_logger(), "ROV Armed");
    } else if (cross_pressed) {
        armed_ = false;
        RCLCPP_INFO(this->get_logger(), "ROV Disarmed");
    }

    if (square_pressed) {
        set_mode("MANUAL");
    } else if (circle_pressed) {
        set_mode("STABILIZE");
    } else if (triangle_pressed) {
        set_mode("ALT_HOLD");
    }

    if (armed_) {
        const int value = 500;

        forward_value_ = 1500 + static_cast<int>(value * left_analog_y);
        lateral_value_ = 1500 + static_cast<int>(value * left_analog_x);

        if (l2_analog > 0.1) {
            throttle_value_ = 1500 + static_cast<int>(value * l2_analog);
        } else if (r2_analog > 0.1) {
            throttle_value_ = 1500 - static_cast<int>(value * r2_analog);
        } else {
            throttle_value_ = 1500;
        }

        publish_command();
    }
}

void ROVJoyController::set_mode(const std::string& mode) {
    current_mode_ = mode;
    RCLCPP_INFO(this->get_logger(), "Menggunakan Mode: %s", mode.c_str());
}

void ROVJoyController::publish_command() {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = (forward_value_ - 1500) / 500.0;
    twist_msg.linear.y = (lateral_value_ - 1500) / 500.0;
    twist_msg.linear.z = (throttle_value_ - 1500) / 500.0;

    cmd_vel_pub_->publish(twist_msg);

    RCLCPP_DEBUG(this->get_logger(), 
        "Values - Lateral: %d, Forward: %d, Throttle: %d, Mode: %s",
        lateral_value_, forward_value_, throttle_value_, current_mode_.c_str());
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("joy_node");
    RCLCPP_INFO(node->get_logger(), "Joy Node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
