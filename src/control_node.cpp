/**
 * Created: Nov. 23, 2025
 * Author(s): Aly Ashour
 */

#include <format>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/motor_power_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"

#include "ap1/control/control_node.hpp"
#include "ap1/control/icontroller.hpp"
#include "ap1/control/pd_controller.hpp"
#include "vectors.hpp"

using namespace ap1_msgs::msg;
using namespace ap1::control;

void ControlNode::on_speed_profile(const SpeedProfileStamped msg)
{
    speed_profile_ = msg;
}

void ControlNode::on_path(const TargetPathStamped msg)
{
    target_path_ = msg;
}

void ControlNode::on_speed(const VehicleSpeedStamped msg)
{
    vehicle_speed_ = msg;
}

void ControlNode::on_turn_angle(const TurnAngleStamped msg)
{
    vehicle_turn_angle_ = msg;
}

void ControlNode::control_loop_callback()
{
    const vec3f velocity(this->vehicle_speed_.speed, 0, 0);

    // Require valid path and speed profile
    if (target_path_.path.empty())
        return;

    if (speed_profile_.speeds.empty())
        return;

    // Compute acceleration
    auto next_waypoint = target_path_.path.at(0);
    const vec3f acc = controller_->compute_acceleration(
        velocity,
        vec2f(next_waypoint.x, next_waypoint.y),
        speed_profile_.speeds.at(0));

    RCLCPP_INFO(this->get_logger(), "ACC: %.2f %.2f %.2f", acc.x, acc.y, acc.z);

    // Convert to Ackermann steering + throttle
    AckermannController::Command cmd =
        ackermann_controller_.compute_command(acc, velocity);

    RCLCPP_INFO(this->get_logger(),
        "CMD: {throttle: %.2f, steering: %.2f}",
        cmd.throttle, cmd.steering);

    // Publish steering
    TurnAngleStamped turn_msg;
    turn_msg.header.stamp = this->now();
    turn_msg.header.frame_id = "base_link";
    turn_msg.angle = cmd.steering;
    turning_angle_pub_->publish(turn_msg);

    // Publish throttle
    MotorPowerStamped pwr_msg;
    pwr_msg.header.stamp = this->now();
    pwr_msg.header.frame_id = "base_link";
    pwr_msg.power = cmd.throttle;
    motor_power_pub_->publish(pwr_msg);
}

void ControlNode::load_config()
{
    YAML::Node config = YAML::LoadFile(config_path_);

    if (config["rate_hz"])
        rate_hz_ = config["rate_hz"].as<int>();

    RCLCPP_INFO(this->get_logger(), "Config reloaded: rate_hz=%d", rate_hz_);
}

ControlNode::ControlNode(const std::string &cfg_path, float rate_hz)
    : Node("control_node"),
      config_path_(cfg_path),
      rate_hz_(rate_hz),
      controller_(new PDController()),
      ackermann_controller_(
          AckermannController(AckermannController::load_config(cfg_path)))
{
    // Subscriptions
    speed_profile_sub_ = this->create_subscription<SpeedProfileStamped>(
        "ap1/planning/speed_profile", 10,
        std::bind(&ControlNode::on_speed_profile, this, std::placeholders::_1));

    target_path_sub_ = this->create_subscription<TargetPathStamped>(
        "ap1/planning/target_path", 10,
        std::bind(&ControlNode::on_path, this, std::placeholders::_1));

    vehicle_speed_sub_ = this->create_subscription<VehicleSpeedStamped>(
        "ap1/actuation/speed_actual", 10,
        std::bind(&ControlNode::on_speed, this, std::placeholders::_1));

    vehicle_turn_angle_sub_ = this->create_subscription<TurnAngleStamped>(
        "ap1/actuation/turn_angle_actual", 10,
        std::bind(&ControlNode::on_turn_angle, this, std::placeholders::_1));

    // Publishers
    turning_angle_pub_ =
        this->create_publisher<TurnAngleStamped>("ap1/control/turn_angle", 10);

    motor_power_pub_ =
        this->create_publisher<MotorPowerStamped>("ap1/control/motor_power", 10);

    // Load config at startup
    load_config();

    // Reset service
    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "ap1/control/reset",
        std::bind(
            &ControlNode::handle_reset,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    // Create control loop timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        std::bind(&ControlNode::control_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
}

void ControlNode::handle_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    try
    {
        load_config();
        response->success = true;
        response->message = "Control reset: config reloaded";
    }
    catch (const std::exception &e)
    {
        response->success = false;
        response->message = std::string("Reset failed: ") + e.what();
    }

    RCLCPP_INFO(this->get_logger(), "Control reset triggered.");
}