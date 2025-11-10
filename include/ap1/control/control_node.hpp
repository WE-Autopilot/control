/**
 * Created: Oct. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROL_NODE_HPP
#define AP1_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/motor_power_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"
#include "ap1_msgs/msg/vehicle_speed_stamped.hpp"

#include "ap1/control/controller.hpp"
#include "ap1/control/pd_controller.hpp"

namespace ap1::control {
class ControlNode : public rclcpp::Node {
private:
  // Fields

  // # Create Control Loop
  // fire at rate_hz
  const double rate_hz_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Controller
  // this should be a sharedptr or something NOT raw.
  ap1::control::IController *controller_;

  // Memory
  // half these types are very unnecessary, we should just have stampedfloat or
  // stamped double or something
  ap1_msgs::msg::SpeedProfileStamped speed_profile_;
  ap1_msgs::msg::TargetPathStamped target_path_;
  ap1_msgs::msg::VehicleSpeedStamped vehicle_speed_;
  ap1_msgs::msg::TurnAngleStamped vehicle_turn_angle;

  // target speed

  // Subs
  rclcpp::Subscription<ap1_msgs::msg::TargetPathStamped>::SharedPtr
      target_path_sub_;
  rclcpp::Subscription<ap1_msgs::msg::SpeedProfileStamped>::SharedPtr
      speed_profile_sub_;
  rclcpp::Subscription<ap1_msgs::msg::VehicleSpeedStamped>::SharedPtr
      vehicle_speed_sub_;
  rclcpp::Subscription<ap1_msgs::msg::TurnAngleStamped>::SharedPtr
      vehicle_turn_angle_sub_;

  // Pubs
  rclcpp::Publisher<ap1_msgs::msg::TurnAngleStamped>::SharedPtr
      turning_angle_pub_;
  rclcpp::Publisher<ap1_msgs::msg::MotorPowerStamped>::SharedPtr
      motor_power_pub_; // between -1 and 1? probably

  void
  on_speed_profile(const ap1_msgs::msg::SpeedProfileStamped speed_profile) {
    speed_profile_ = speed_profile;
  }

  void on_path(const ap1_msgs::msg::TargetPathStamped target_path) {
    target_path_ = target_path;
  }

  void on_speed(const ap1_msgs::msg::VehicleSpeedStamped speed) {
    vehicle_speed_ = speed;
  }

  void on_turn_angle(const ap1_msgs::msg::TurnAngleStamped turn_angle) {
    vehicle_turn_angle = turn_angle;
  }

  void control_loop_callback() {
    const std::vector<float> velocity{this->vehicle_speed_.speed, 0,
                                      0}; // which direction is up? assuming +x

    const std::vector<float> acc = controller_->compute_acceleration(
        velocity,
        target_path_.path.at(0),    // for now get the first path waypoint
        speed_profile_.speeds.at(0) // and the first speed value
    );

    // send the acceleration through publisher topics
    // we need to implement ackermann steering here.
  }

public:
  ControlNode(float rate_hz = 60) : Node("control_node"), rate_hz_(rate_hz) {
    controller_ = new ap1::control::PDController();

    // # All inputs shabooya
    // - SPEED PROFILE
    speed_profile_sub_ =
        this->create_subscription<ap1_msgs::msg::SpeedProfileStamped>(
            "ap1/planning/speed_profile", 10,
            std::bind(&ControlNode::on_speed_profile, this,
                      std::placeholders::_1));
    // - TARGET PATH
    target_path_sub_ =
        this->create_subscription<ap1_msgs::msg::TargetPathStamped>(
            "ap1/planning/speed_profile", 10,
            std::bind(&ControlNode::on_path, this, std::placeholders::_1));
    vehicle_speed_sub_ =
        this->create_subscription<ap1_msgs::msg::VehicleSpeedStamped>(
            "ap1/actuation/speed_actual", 10,
            std::bind(&ControlNode::on_speed, this, std::placeholders::_1));
    vehicle_turn_angle_sub_ =
        this->create_subscription<ap1_msgs::msg::TurnAngleStamped>(
            "ap1/actuation/turn_angle_actual", 10,
            std::bind(&ControlNode::on_turn_angle, this,
                      std::placeholders::_1));

    // # Publishers
    // - TURNING ANGLE
    turning_angle_pub_ =
        this->create_publisher<ap1_msgs::msg::TurnAngleStamped>(
            "ap1/control/motor_power", 10);
    // - MOTOR POWER
    motor_power_pub_ = this->create_publisher<ap1_msgs::msg::MotorPowerStamped>(
        "ap1/control/turn_angle", 10);

    // # Create Control Loop
    // fire at rate_hz
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        std::bind(&ControlNode::control_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
  }
};
} // namespace ap1::control

#endif // AP1_CONTROL_NODE_HPP
