/**
 * Created: Oct. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROL_NODE_HPP
#define AP1_CONTROL_NODE_HPP

#include <iostream>
#include <string>
#include <memory>
#include <filesystem>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

// ---- AP1 Messages ----
#include "ap1_msgs/msg/motor_power_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"

// ---- Your Reset Service ----
#include "std_srvs/srv/trigger.hpp"

// ---- Controllers (upstream) ----
#include "ap1/control/ackermann_controller.hpp"
#include "ap1/control/icontroller.hpp"

namespace ap1::control
{

class ControlNode : public rclcpp::Node
{
public:
    // Constructor with config path + default rate
    ControlNode(const std::string &cfg_path, float rate_hz = 60);

private:
    // --------------------------------------------------------------------
    // CONFIG + INTERNAL STATE
    // --------------------------------------------------------------------
    void load_config();

    std::string config_path_;
    int rate_hz_{60};

    float last_velocity_ = 0.0f;
    float last_error_ = 0.0f;
    float integral_term_ = 0.0f;

    // --------------------------------------------------------------------
    // UPSTREAM CONTROLLER ARCHITECTURE
    // --------------------------------------------------------------------
    std::unique_ptr<IController> controller_;
    AckermannController ackermann_controller_;

    // Cached “latest values”
    ap1_msgs::msg::SpeedProfileStamped speed_profile_;
    ap1_msgs::msg::TargetPathStamped target_path_;
    ap1_msgs::msg::VehicleSpeedStamped vehicle_speed_;
    ap1_msgs::msg::TurnAngleStamped vehicle_turn_angle_;

    // --------------------------------------------------------------------
    // SUBSCRIPTIONS
    // --------------------------------------------------------------------
    rclcpp::Subscription<ap1_msgs::msg::TargetPathStamped>::SharedPtr target_path_sub_;
    rclcpp::Subscription<ap1_msgs::msg::SpeedProfileStamped>::SharedPtr speed_profile_sub_;
    rclcpp::Subscription<ap1_msgs::msg::VehicleSpeedStamped>::SharedPtr vehicle_speed_sub_;
    rclcpp::Subscription<ap1_msgs::msg::TurnAngleStamped>::SharedPtr vehicle_turn_angle_sub_;

    void on_speed_profile(const ap1_msgs::msg::SpeedProfileStamped msg);
    void on_path(const ap1_msgs::msg::TargetPathStamped msg);
    void on_speed(const ap1_msgs::msg::VehicleSpeedStamped msg);
    void on_turn_angle(const ap1_msgs::msg::TurnAngleStamped msg);

    // --------------------------------------------------------------------
    // PUBLISHERS
    // --------------------------------------------------------------------
    rclcpp::Publisher<ap1_msgs::msg::TurnAngleStamped>::SharedPtr turning_angle_pub_;
    rclcpp::Publisher<ap1_msgs::msg::MotorPowerStamped>::SharedPtr motor_power_pub_;

    // --------------------------------------------------------------------
    // CONTROL LOOP
    // --------------------------------------------------------------------
    rclcpp::TimerBase::SharedPtr timer_;
    void control_loop_callback();

    // --------------------------------------------------------------------
    // RESET SERVICE
    // --------------------------------------------------------------------
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    void handle_reset(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

} // namespace ap1::control

#endif // AP1_CONTROL_NODE_HPP
