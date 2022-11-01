#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/resource_manager.hpp>
#include <controller_manager/controller_manager.hpp>
#include "hoverboard.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hoverboard_driver");

    Hoverboard hoverboard;

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    controller_manager::ControllerManager cm(
        std::make_unique<hardware_interface::ResourceManager>(),
        executor, "hoverboard_manager");

    rclcpp::Time prev_time = node->now();
    rclcpp::Rate rate(100.0);

    while (rclcpp::ok()) {
        const rclcpp::Time time = node->now();
        const rclcpp::Duration period = time - prev_time;
        prev_time = time;

        hoverboard.read();
        cm.update();
        hoverboard.write(time, period);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    return 0;
}
