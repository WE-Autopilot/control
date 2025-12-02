/**
 * Created: Nov. 23, 2025
 * Author(s): Aly Ashour
 */

#include <format>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

// messages
#include "ap1_msgs/msg/motor_power_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"

// control
#include "ap1/control/control_node.hpp"
#include "ap1/control/icontroller.hpp"
#include "ap1/control/pd_controller.hpp"
#include "ap1/control/ackermann_controller.hpp"
#include "ap1/control/icontroller.hpp"
#include "vectors.hpp"

// special types
#ifdef AP1_CONTROL_SUPPORT_TWIST
#include "geometry_msgs/msg/twist.hpp"
#elif defined(AP1_CONTROL_SUPPORT_ACKERMANN)
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#endif

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

void ControlNode::on_speed(const VehicleSpeedStamped speed)
{
    vehicle_speed_ = speed;
}

void ControlNode::on_turn_angle(const TurnAngleStamped turn_angle)
{
    vehicle_turn_angle = turn_angle;
}

void ControlNode::control_loop_callback()
{
    // the car's current velocity. we only support moving forward atp
    const vec3f velocity(this->vehicle_speed_.speed, 0, 0); // +x is always forward on the car

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
    std::string s = "ACC: " + std::to_string(acc.x) + ", " + std::to_string(acc.y) + ", " +
                    std::to_string(acc.z);
    RCLCPP_INFO(this->get_logger(), s.c_str());

    // compute acc and throttle using ackermann controller
    AckermannController::Command cmd = ackermann_controller_.compute_command(acc, velocity);

    // log
    s = "CMD: {throttle:" + std::to_string(cmd.throttle) +
        ", steering:" + std::to_string(cmd.steering) + "}";
    RCLCPP_INFO(this->get_logger(), s.c_str());

#ifdef AP1_CONTROL_SUPPORT_ACKERMANN
    // only send out an ackermann message
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = this->now();
    msg.drive.speed = speed_profile_.speeds.at(0);
    msg.drive.steering_angle = cmd.steering; // rads
    ackermann_pub_->publish(msg);
#elif defined(AP1_CONTROL_SUPPORT_TWIST)
    // only send out a twist message
    geometry_msgs::msg::Twist msg;
    msg.linear.x = speed_profile_.speeds.at(0);
    // msg.angular.z = cmd.steering; STILL NEEDS TO BE IMPLEMENTED. THIS IS TARGET ANGLE NOT RAD/S
    // ANGULAR CONTROL.
    throw "Not yet implemented";
    twist_pub_->publish(msg);
#else
    // send out both a TurnAngleStamped and MotorPowerStamped message
    // pack the turn angle into a message
    TurnAngleStamped turn_msg;
    turn_msg.header.stamp = this->now();
    turn_msg.header.frame_id = "base_link";
    turn_msg.angle = cmd.steering; // rads

    // pack the power into a message
    MotorPowerStamped pwr_msg;
    pwr_msg.header.stamp = this->now();
    pwr_msg.header.frame_id = "base_link";
    pwr_msg.power = cmd.throttle; // [-1, 1]

    // send both messages out
    turning_angle_pub_->publish(turn_msg);
    motor_power_pub_->publish(pwr_msg);
#endif
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
    vehicle_speed_sub_ = this->create_subscription<VehicleSpeedStamped>(
        "ap1/actuation/speed_actual", 10,
        std::bind(&ControlNode::on_speed, this, std::placeholders::_1));
    vehicle_turn_angle_sub_ = this->create_subscription<TurnAngleStamped>(
        "ap1/actuation/turn_angle_actual", 10,
        std::bind(&ControlNode::on_turn_angle, this, std::placeholders::_1));

    // Pubs
    turning_angle_pub_ = this->create_publisher<TurnAngleStamped>("ap1/control/turn_angle", 10);
    motor_power_pub_ = this->create_publisher<MotorPowerStamped>("ap1/control/motor_power", 10);
    #ifdef AP1_CONTROL_SUPPORT_ACKERMANN
    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/ap1/control/ackermann_cmd", 10);
    #endif

    #ifdef AP1_CONTROL_SUPPORT_TWIST
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/ap1/control/twist_cmd", 10);
    #endif

    // Create Control Loop
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz),
                                     std::bind(&ControlNode::control_loop_callback, this));

    // Log completion
    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
}
