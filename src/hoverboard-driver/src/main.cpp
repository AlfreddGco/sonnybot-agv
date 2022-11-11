#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/resource_manager.hpp>
#include <controller_manager/controller_manager.hpp>
#include "hoverboard.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hoverboard_driver");

    auto rm = std::make_unique<hardware_interface::ResourceManager>();
    rm->import_component(std::make_unique<Hoverboard>());

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto cm = std::make_shared<controller_manager::ControllerManager>(
        std::move(rm), executor, "hoverboard_manager");
    executor->add_node(cm);

    // NOTE: Threading model could be improved.
    auto updater = std::thread([&](void) -> void {
        while(rclcpp::ok()){
            cm->read();
            cm->update();
            cm->write();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });
    executor->spin();
    updater.join();
    rclcpp::shutdown();
    return 0;
}
