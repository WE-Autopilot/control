#include "ap1/control/control_node.hpp"

namespace ap1::control {

ControlNode::ControlNode()
: Node("control_node")
{
    speed_profile_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "speed_profile", 10,
        std::bind(&ControlNode::on_speed_profile, this, std::placeholders::_1));

    target_path_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "target_path", 10,
        std::bind(&ControlNode::on_path, this, std::placeholders::_1));

    turning_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("turning_angle", 10);
    motor_power_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor_power", 10);

#ifdef AP1_CONTROL_SUPPORT_ACKERMANN
    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/ap1/control/ackermann_cmd", 10);
#endif

    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
}

void ControlNode::on_speed_profile(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Speed Profile from Planning");

    float motor_output = 0.0f;   // TODO compute
    float turning_output = 0.0f; // TODO compute

    // Standard outputs
    std_msgs::msg::Float32 motor_msg;
    motor_msg.data = motor_output;
    motor_power_pub_->publish(motor_msg);

    std_msgs::msg::Float32 turn_msg;
    turn_msg.data = turning_output;
    turning_angle_pub_->publish(turn_msg);

#ifdef AP1_CONTROL_SUPPORT_ACKERMANN
    ackermann_msgs::msg::AckermannDriveStamped ack_msg;
    ack_msg.header.stamp = this->now();
    ack_msg.drive.speed = motor_output;
    ack_msg.drive.steering_angle = turning_output;
    ackermann_pub_->publish(ack_msg);
#endif
}

} // namespace ap1::control
