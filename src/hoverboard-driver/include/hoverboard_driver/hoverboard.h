#include "rclcpp/rclcpp.hpp"
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/bool.h>
#include <string>
#include "hoverboard_driver/HoverboardConfig.h"
#include "hoverboard_driver/pid.h"
#include "protocol.h"

class HoverboardAPI;

class Hoverboard : public hardware_interface::SystemInterface {
public:
    Hoverboard();
    ~Hoverboard();
    
    void read();
    void write(const ros::Time& time, const ros::Duration& period);
    void tick();
 private:
    void protocol_recv (char c);
    void on_encoder_update (int16_t right, int16_t left);
 
    hardware_interface::StateInterface joint_state_interface;
    hardware_interface::ActuatorInterface velocity_joint_interface;

    // The units for wheels are radians (pos), radians per second (vel,cmd), and Netwton metres (eff)
    struct Joint {
        std_msgs::Float64 pos;
        std_msgs::Float64 vel;
        std_msgs::Float64 eff;
        std_msgs::Float64 cmd;
    } joints[2];

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher vel_pub[2];
    ros::Publisher pos_pub[2];    
    ros::Publisher cmd_pub[2];
    ros::Publisher voltage_pub;
    ros::Publisher temp_pub;
    ros::Publisher connected_pub;

    double wheel_radius;
    double max_velocity = 0.0;
    int direction_correction = 1;
    std::string port;

    ros::Time last_read;
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
