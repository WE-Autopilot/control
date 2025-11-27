/**
 * Created: Oct. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROL_NODE_HPP
#define AP1_CONTROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ap1_msgs/msg/motor_power_stamped.hpp"
#include "ap1_msgs/msg/turn_angle_stamped.hpp"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <chrono>

namespace ap1::control
{
    class ControlNode : public rclcpp::Node
    {
    public:
        ControlNode()
            : Node("control_node")
        {
            // ---------------------------------------------------------
            // CONFIG PATH
            // ---------------------------------------------------------
            config_path_ =
                std::string(
                    std::filesystem::path(__FILE__)
                        .parent_path()      // include/ap1/control
                        .parent_path()      // include/ap1
                        .parent_path()      // include
                        .parent_path()      // package root
                ) + "/config.yaml";

            load_config();   // initial load ✔

            // ---------------------------------------------------------
            // SUBSCRIPTIONS
            // ---------------------------------------------------------
            speed_profile_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                "speed_profile", 10,
                std::bind(&ControlNode::on_speed_profile, this, std::placeholders::_1));

            target_path_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
                "target_path", 10,
                std::bind(&ControlNode::on_path, this, std::placeholders::_1));

            // ---------------------------------------------------------
            // PUBLISHERS  (fixed to use AP1 messages)
            // ---------------------------------------------------------
            motor_power_pub_ =
                create_publisher<ap1_msgs::msg::MotorPowerStamped>(
                    "/ap1/control/motor_power", 10);

            turning_angle_pub_ =
                create_publisher<ap1_msgs::msg::TurnAngleStamped>(
                    "/ap1/control/turn_angle", 10);

            // ---------------------------------------------------------
            // RESET SERVICE
            // ---------------------------------------------------------
            reset_service_ = create_service<std_srvs::srv::Trigger>(
                "/control/reset",
                std::bind(
                    &ControlNode::handle_reset, this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            // ---------------------------------------------------------
            // TIMER
            // ---------------------------------------------------------
            timer_ = create_wall_timer(
                std::chrono::milliseconds(1000 / rate_hz_),
                std::bind(&ControlNode::on_timer, this));

            RCLCPP_INFO(get_logger(), "ControlNode running at %d Hz", rate_hz_);
        }

    private:
        // ============================================================
        // ------------------- CONFIG LOADER ---------------------------
        // ============================================================

        void load_config()
        {
            YAML::Node config = YAML::LoadFile(config_path_);

            if (!config["rate_hz"])
            {
                RCLCPP_FATAL(get_logger(), "[Control] Missing rate_hz in config");
                throw std::runtime_error("Missing control rate_hz");
            }

            rate_hz_ = config["rate_hz"].as<int>();

            RCLCPP_INFO(get_logger(), "[Control] Config loaded: rate_hz=%d", rate_hz_);
        }

        // ============================================================
        // ------------------- SUBSCRIPTIONS ---------------------------
        // ============================================================

        void on_speed_profile(const std_msgs::msg::Float32MultiArray::SharedPtr)
        {
            RCLCPP_INFO(get_logger(), "Received Speed Profile");
        }

        void on_path(const std_msgs::msg::Float32MultiArray::SharedPtr)
        {
            RCLCPP_INFO(get_logger(), "Received New Path");
        }

        // ============================================================
        // ------------------- CONTROL TIMER ---------------------------
        // ============================================================

        void on_timer()
        {
            // Fake test output (simulator will read these)

            ap1_msgs::msg::MotorPowerStamped motor_msg;
            motor_msg.header.stamp = now();
            motor_msg.power = 0.0f;  // fake throttle
            motor_power_pub_->publish(motor_msg);

            ap1_msgs::msg::TurnAngleStamped steer_msg;
            steer_msg.header.stamp = now();
            steer_msg.angle = 0.1f;  // fake steering
            turning_angle_pub_->publish(steer_msg);
        }

        // ============================================================
        // ------------------- RESET SERVICE ---------------------------
        // ============================================================

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

        void handle_reset(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            // Reset controller state
            last_velocity_  = 0.0f;
            last_error_     = 0.0f;
            integral_term_  = 0.0f;

            // Reload config after reset ✔
            load_config();

            response->success = true;
            response->message = "Control reset + config reloaded";

            RCLCPP_INFO(get_logger(), "[Control] Reset complete.");
        }

        // ============================================================
        // ------------------- INTERNAL STATE --------------------------
        // ============================================================

        float last_velocity_ = 0.0f;
        float last_error_ = 0.0f;
        float integral_term_ = 0.0f;

        int rate_hz_;
        std::string config_path_;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr speed_profile_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_path_sub_;

        rclcpp::Publisher<ap1_msgs::msg::MotorPowerStamped>::SharedPtr motor_power_pub_;
        rclcpp::Publisher<ap1_msgs::msg::TurnAngleStamped>::SharedPtr turning_angle_pub_;

        rclcpp::TimerBase::SharedPtr timer_;
    };
}

#endif