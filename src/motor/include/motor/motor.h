#ifndef MOTOR_H
#define MOTOR_H

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

class Motor : public rclcpp::Node {
public:
    Motor();
    void main_loop();
    void testing();
    bool arming(bool state);
    bool set_mode(const std::string& mode);

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr motor_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    bool arming_state_;
};

#endif