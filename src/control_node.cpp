/**
 * Created: Nov. 23, 2025
 * Author(s): Aly Ashour
 */

#include <format>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"

#include "ap1/control/control_node.hpp"
#include "ap1/control/icontroller.hpp"
#include "ap1/control/pd_controller.hpp"
#include "vectors.hpp"

using namespace ap1_msgs::msg;
using namespace ap1::control;

void ControlNode::on_speed_profile(const SpeedProfileStamped speed_profile)
{
    speed_profile_ = speed_profile;
}

void ControlNode::on_path(const TargetPathStamped target_path)
{
    target_path_ = target_path;
}

void ControlNode::on_speed(const FloatStamped & msg)
{
    vehicle_speed_ = msg.value;
}

void ControlNode::on_turn_angle(const FloatStamped & msg)
{
    vehicle_turn_angle = msg.value;
}

void ControlNode::control_loop_callback()
{
    // the car's current velocity. we only support moving forward atp
    const vec3f velocity(this->vehicle_speed_, 0, 0); // +x is always forward on the car

    const bool PATH_IS_STALE = false, SPEED_PROFILE_IS_STALE = false; // TEMP

    // if path has no waypoints OR path is too old
    if (target_path_.path.size() < 1 || PATH_IS_STALE)
        return; // don't update
                // todo: ideally this should actually still
                // send through the speed control
                // but we'll implement that later

    // if speed profile has no waypoints or speed profile is too old
    if (speed_profile_.speeds.size() < 1 || SPEED_PROFILE_IS_STALE)
        return;

    // ask the controller to calculate the acceleration needed
    // for now we'll only consider the very next waypoint & speed value
    auto next_waypoint = target_path_.path.at(0);
    const vec3f acc = controller_->compute_acceleration(
        velocity, vec2f(next_waypoint.x, next_waypoint.y), speed_profile_.speeds.at(0));

    // log
    RCLCPP_INFO(this->get_logger(), "ACC: %.2f, %.2f, %.2f", acc.x, acc.y, acc.z);

    // compute acc and throttle using ackermann controller
    AckermannController::Command cmd = ackermann_controller_.compute_command(acc, velocity);

    // log
    RCLCPP_INFO(this->get_logger(), "CMD: {throttle: %.2f, steering: %.2f}", cmd.throttle, cmd.steering);

    // pack the turn angle into a message
    FloatStamped turn_msg;
    turn_msg.header.stamp = this->now();
    turn_msg.header.frame_id = "base_link";
    turn_msg.value = cmd.steering; // rads

    // pack the power into a message
    FloatStamped pwr_msg;
    pwr_msg.header.stamp = this->now();
    pwr_msg.header.frame_id = "base_link";
    pwr_msg.value = cmd.throttle; // [-1, 1]
    // pwr_msg.power = 1.0f;

    // send both messages out
    turning_angle_pub_->publish(turn_msg);
    motor_power_pub_->publish(pwr_msg);
}

ControlNode::ControlNode(const std::string& cfg_path, float rate_hz)
    : Node("control_node"), rate_hz_(rate_hz), controller_(new PDController()),
      ackermann_controller_(AckermannController(AckermannController::load_config(cfg_path)))
{
    // Subs
    speed_profile_sub_ = this->create_subscription<SpeedProfileStamped>(
        "ap1/planning/speed_profile", 10,
        std::bind(&ControlNode::on_speed_profile, this, std::placeholders::_1));
    target_path_sub_ = this->create_subscription<TargetPathStamped>(
        "ap1/planning/target_path", 10,
        std::bind(&ControlNode::on_path, this, std::placeholders::_1));
    vehicle_speed_sub_ = this->create_subscription<FloatStamped>(
        "ap1/actuation/speed_actual", 10,
        std::bind(&ControlNode::on_speed, this, std::placeholders::_1));
    vehicle_turn_angle_sub_ = this->create_subscription<FloatStamped>(
        "ap1/actuation/turn_angle_actual", 10,
        std::bind(&ControlNode::on_turn_angle, this, std::placeholders::_1));

    // Pubs
    turning_angle_pub_ = this->create_publisher<FloatStamped>("ap1/control/turn_angle", 10);
    motor_power_pub_ = this->create_publisher<FloatStamped>("ap1/control/motor_power", 10);

    // Create Control Loop
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz),
                                     std::bind(&ControlNode::control_loop_callback, this));

    // Log completion
    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
}
