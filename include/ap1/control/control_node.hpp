/**
 * Created: Oct. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROL_NODE_HPP
#define AP1_CONTROL_NODE_HPP

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "ap1/control/controller.hpp"
#include "ap1/control/pd_controller.hpp"

namespace ap1::control {
class ControlNode : public rclcpp::Node {
private:
  // Fields

  // Controller
  // this should be a sharedptr or something NOT raw.
  ap1::control::IController *controller_;

  // Memory

  // Subs
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      speed_profile_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      target_path_sub_;

  // Pubs
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turning_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      motor_power_pub_; // between -1 and 1? probably

  void on_speed_profile(const std_msgs::msg::Float32MultiArray::SharedPtr) {
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Received Speed Profile from Planning");
  }

  void on_path(const std_msgs::msg::Float32MultiArray::SharedPtr) {
    // todo: implement
    RCLCPP_INFO(this->get_logger(), "Received new path from Planning");
  }

public:
  ControlNode() : Node("control_node") {
    controller_ = new ap1::control::PDController();

    // # All inputs shabooya
    // - SPEED PROFILE
    speed_profile_sub_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "speed_profile", 10,
            std::bind(&ControlNode::on_speed_profile, this,
                      std::placeholders::_1));
    // - TARGET PATH
    target_path_sub_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "target_path", 10,
            std::bind(&ControlNode::on_path, this, std::placeholders::_1));

    // # Publishers
    // - TURNING ANGLE
    turning_angle_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("turning_angle", 10);
    // - MOTOR POWER
    motor_power_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("motor_power", 10);

    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
  }
};
} // namespace ap1::control

#endif // AP1_CONTROL_NODE_HPP
