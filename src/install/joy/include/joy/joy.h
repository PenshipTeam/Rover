#ifndef rover_src_joy_src_joy_h_
#define rover_src_joy_src_joy_h_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ROVJoyController : public rclcpp::Node {
public:
    ROVJoyController();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    void set_mode(const std::string& mode);
    void publish_command();

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // State variables
    int lateral_value_;
    int forward_value_;
    int throttle_value_;
    bool armed_;
    std::string current_mode_;
};

#endif 
