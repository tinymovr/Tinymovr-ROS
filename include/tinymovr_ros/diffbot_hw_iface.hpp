
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
    Diffbot() 
    { 
        auto status = socket_can.open("can0");
        if (scpp::STATUS_OK == status)
        {
            ROS_INFO("Socketcan opened successfully");
        }
        else
        {
            ROS_ERROR("Cant' open Socketcan: %d", status);
            exit(1);
        }

        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_a("A", &hw_positions_[0], &hw_velocities_[0], &hw_efforts_[0]);
        jnt_state_interface.registerHandle(state_handle_a);

        hardware_interface::JointStateHandle state_handle_b("B", &hw_positions_[1], &hw_velocities_[1], &hw_efforts_[1]);
        jnt_state_interface.registerHandle(state_handle_b);

        registerInterface(&jnt_state_interface);

        // connect and register the joint velocity interface
        hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("A"), &hw_commands_[0]);
        jnt_vel_interface.registerHandle(vel_handle_a);

        hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("B"), &hw_commands_[1]);
        jnt_vel_interface.registerHandle(vel_handle_b);

        registerInterface(&jnt_vel_interface);
    }
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