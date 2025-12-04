#include <rclcpp/logger.hpp>
#include <string>

#include "ap1/control/control_node.hpp"

const char* LOGGER_NAME = "main";

int main(int argc, char** argv)
{
    // init ros & ros logger
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<ap1::control::ControlNode>(config_path);

    // Spin node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
