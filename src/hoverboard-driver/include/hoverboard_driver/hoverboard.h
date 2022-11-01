#ifndef HOVERBOARD_H_
#define HOVERBOARD_H_

#include "rclcpp/rclcpp.hpp"
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>
#include <memory>
// #include "hoverboard_driver/HoverboardConfig.h"
#include "hoverboard_driver/pid.h"
#include "protocol.h"

constexpr double WHEEL_RADIUS = 0.08255;
constexpr double MAX_VELOCITY = 1;
constexpr int DIRECTION_CORRECTION = 1;
constexpr char PORT[] = "/dev/ttyTHS1";

class HoverboardAPI;

class Hoverboard : public hardware_interface::SystemInterface {
public:
    Hoverboard();
    ~Hoverboard();
    
    hardware_interface::return_type start() {
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type stop() {
        return hardware_interface::return_type::OK;
    }
    std::string get_name() const {
        return "hoverboard_driver";
    }

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period){
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type update(){
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type configure(
        const hardware_interface::HardwareInfo &system_info){
            return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type read();
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period);
    void tick();
    std::vector<hardware_interface::StateInterface> export_state_interfaces();
    std::vector<hardware_interface::CommandInterface> export_command_interfaces();
 private:
    void protocol_recv (char c);
    void on_encoder_update (int16_t right, int16_t left);

    // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
    struct Joint {
        std_msgs::msg::Float64 pos;
        std_msgs::msg::Float64 vel;
        std_msgs::msg::Float64 eff;
        std_msgs::msg::Float64 cmd;
    } joints[2];

    // Publishers
    rclcpp::Node node;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> vel_pub[2];
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pos_pub[2];    
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> cmd_pub[2];
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> voltage_pub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> temp_pub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> connected_pub;

    double wheel_radius = WHEEL_RADIUS;
    double max_velocity = MAX_VELOCITY;
    int direction_correction = DIRECTION_CORRECTION;
    std::string port = PORT;

    rclcpp::Time last_read;
    // Last known encoder values
    int16_t last_wheelcountR;
    int16_t last_wheelcountL;
    // Count of full encoder wraps
    int multR;
    int multL;
    // Thresholds for calculating the wrap
    int low_wrap;
    int high_wrap;

    // Hoverboard protocol
    int port_fd;
    int msg_len = 0;
    char prev_byte = 0;
    uint16_t start_frame = 0;
    char* p;
    SerialFeedback msg, prev_msg;

    PID pids[2];
};

#endif // HOVERBOARD_H_
