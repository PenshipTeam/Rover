#ifndef MAIN_H
#define MAIN_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>

class MainController : public rclcpp::Node {
public:
    MainController();

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
    
    bool armed_;
    std::string current_mode_;
};

#endif