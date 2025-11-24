/**
 * Created: Oct. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROL_NODE_HPP
#define AP1_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

#ifdef AP1_CONTROL_SUPPORT_ACKERMANN
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#endif

namespace ap1::control {
    class ControlNode : public rclcpp::Node {
    private:
        void on_speed_profile(const std_msgs::msg::Float32MultiArray::SharedPtr) {
            // todo: implement
            RCLCPP_INFO(this->get_logger(), "Received Speed Profile from Planning");
        }

        void on_path(const std_msgs::msg::Float32MultiArray::SharedPtr) {
            // todo: implement
            RCLCPP_INFO(this->get_logger(), "Received new path from Planning");
        }

        // Fields
        // Subs
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr speed_profile_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_path_sub_;

        // Pubs
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr turning_angle_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_power_pub_; // between -1 and 1? probably
        #ifdef AP1_CONTROL_SUPPORT_ACKERMANN
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
        // timer for publishing outputs
        rclcpp::TimerBase::SharedPtr timer_; 
        #endif
    public:
        ControlNode() : Node("control_node") {
            // # All inputs shabooya
            // - SPEED PROFILE
            speed_profile_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "speed_profile", 10, std::bind(&ControlNode::on_speed_profile, this, std::placeholders::_1)
            );
            // - TARGET PATH
            target_path_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "target_path", 10, std::bind(&ControlNode::on_path, this, std::placeholders::_1)
            );
        
            // # Publishers
            // - TURNING ANGLE
            turning_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("turning_angle", 10);
            // - MOTOR POWER
            motor_power_pub_ = this->create_publisher<std_msgs::msg::Float32>("motor_power", 10);
            
            #ifdef AP1_CONTROL_SUPPORT_ACKERMANN
            ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ap1/control/ackermann_cmd", 10
            );

            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { publish_outputs(1.0, 0.1); }
            );
            #endif

        }

        void publish_outputs(float motor_output, float turning_output) {
            #ifdef AP1_CONTROL_SUPPORT_ACKERMANN
            ackermann_msgs::msg::AckermannDriveStamped msg;
            msg.header.stamp = this->now();
            msg.drive.speed = motor_output;
            msg.drive.steering_angle = turning_output;
            ackermann_pub_->publish(msg);
            #endif

            std_msgs::msg::Float32 motor_msg;
            motor_msg.data = motor_output;
            motor_power_pub_->publish(motor_msg);

            std_msgs::msg::Float32 turning_msg;
            turning_msg.data = turning_output;
            turning_angle_pub_->publish(turning_msg);
        }

    };
}

#endif // AP1_CONTROL_NODE_HPP