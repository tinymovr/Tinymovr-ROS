
#pragma once

#include <vector>
#include "socketcan_cpp/socketcan_cpp.hpp"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace tinymovr_ros
{

class Diffbot : public hardware_interface::RobotHW
{
public:
    Diffbot();

    bool read(const ros::Duration& period);
    bool write();

    bool write_frame(uint32_t arb_id, const uint8_t *data, uint8_t data_len);
    uint32_t make_arbitration_id(uint32_t node_id, uint32_t command_id);

private:

    const double radius = 0.075;  // radius of the wheels
    const double dist_w = 0.38;   // distance between the wheels
    
    scpp::SocketCan socket_can;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    std::vector<double> hw_commands_ {0.0, 0.0};
    std::vector<double> hw_positions_ {0.0, 0.0};
    std::vector<double> hw_velocities_ {0.0, 0.0};
    std::vector<double> hw_efforts_ {0.0, 0.0};
    std::vector<uint8_t> hw_node_ids_ {1, 2};

    // Store the wheeled robot position
    double base_x_, base_y_, base_theta_;
};

}