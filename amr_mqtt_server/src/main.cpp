/**
 * @file main.cpp
 * @brief Entry point for the AMR MQTT bridge node.
 */

#include "amr_mqtt_server/node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = amr::mqtt::server::make_node();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
