
#pragma once

#include <vector>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <tinymovr_ros/tinymovr_can.hpp>

namespace tinymovr_ros
{

class Diffbot : public hardware_interface::RobotHW
{
public:
    Diffbot();

    bool read(const ros::Duration& period);
    bool write();

private:

    const double radius = 0.075;  // radius of the wheels
    const double dist_w = 0.38;   // distance between the wheels
    
    TinymovrCAN tmcan;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    std::vector<double> hw_commands_ {0.0, 0.0}; // rad/s
    std::vector<double> hw_positions_ {0.0, 0.0}; // rad
    std::vector<double> hw_velocities_ {0.0, 0.0}; // rad/s
    std::vector<double> hw_efforts_ {0.0, 0.0}; // Nm?
    std::vector<uint8_t> hw_node_ids_ {1, 2};
};

}