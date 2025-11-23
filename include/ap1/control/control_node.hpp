/**
 * Created: Oct. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROL_NODE_HPP
#define AP1_CONTROL_NODE_HPP

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/motor_power_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"

#include "ap1/control/ackermann_controller.hpp"
#include "ap1/control/icontroller.hpp"

namespace ap1::control
{
class ControlNode : public rclcpp::Node
{
  private:
    // Fields

    // Control Loop
    const double rate_hz_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Controller
    std::unique_ptr<IController> controller_;
    AckermannController ackermann_controller_;

    // Memory
    // half these types are very unnecessary, we should just have stampedfloat or
    // stamped double or something
    ap1_msgs::msg::SpeedProfileStamped speed_profile_;
    ap1_msgs::msg::TargetPathStamped target_path_;
    ap1_msgs::msg::VehicleSpeedStamped vehicle_speed_;
    ap1_msgs::msg::TurnAngleStamped vehicle_turn_angle;

    // Subs
    rclcpp::Subscription<ap1_msgs::msg::TargetPathStamped>::SharedPtr target_path_sub_;
    rclcpp::Subscription<ap1_msgs::msg::SpeedProfileStamped>::SharedPtr speed_profile_sub_;
    rclcpp::Subscription<ap1_msgs::msg::VehicleSpeedStamped>::SharedPtr vehicle_speed_sub_;
    rclcpp::Subscription<ap1_msgs::msg::TurnAngleStamped>::SharedPtr vehicle_turn_angle_sub_;

    // Pubs
    rclcpp::Publisher<ap1_msgs::msg::TurnAngleStamped>::SharedPtr turning_angle_pub_;
    rclcpp::Publisher<ap1_msgs::msg::MotorPowerStamped>::SharedPtr motor_power_pub_; // between -1 and 1? probably

    // Methods
    void on_speed_profile(const ap1_msgs::msg::SpeedProfileStamped speed_profile);
    void on_path(const ap1_msgs::msg::TargetPathStamped target_path);
    void on_speed(const ap1_msgs::msg::VehicleSpeedStamped speed);
    void on_turn_angle(const ap1_msgs::msg::TurnAngleStamped turn_angle);
    void control_loop_callback();

  public:
    ControlNode(const std::string& cfg_path, float rate_hz = 60);
};
} // namespace ap1::control

#endif // AP1_CONTROL_NODE_HPP
