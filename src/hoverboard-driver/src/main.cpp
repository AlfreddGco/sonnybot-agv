#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/resource_manager.hpp>
#include <controller_manager/controller_manager.hpp>
#include "hoverboard.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hoverboard_driver");

    auto hoverboard = std::make_unique<Hoverboard>();
    hardware_interface::ComponentInfo hoverboard_component = {
        .name = "wheels",
        .type = "joint",
        .command_interfaces = {
            {.name = "left_wheel_vel"},
            {.name = "right_wheel_vel"}
        },
        .state_interfaces = {
            {.name = "left_wheel_pos"},
            {.name = "left_wheel_vel"},
            {.name = "right_wheel_pos"},
            {.name = "right_wheel_vel"},
        },
    };
    std::vector<hardware_interface::ComponentInfo> components = {
        hoverboard_component
    };
    auto rm = std::make_unique<hardware_interface::ResourceManager>();
    rm->import_component(hoverboard, {
        .name = "hoverboard",
        .type = "system",
        .hardware_class_type = "",
        .joints = components
    });

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    controller_manager::ControllerManager cm(
        std::move(rm), executor, "hoverboard_manager");

    rclcpp::Time prev_time = node->now();
    rclcpp::Rate rate(100.0);

    while (rclcpp::ok()) {
        const rclcpp::Time time = node->now();
        const rclcpp::Duration period = time - prev_time;
        prev_time = time;

        cm.read(time, period);
        cm.update(time, period);
        cm.write(time, period);

        rclcpp::spin_some(node);
        rate.sleep();
    }

    return 0;
}
