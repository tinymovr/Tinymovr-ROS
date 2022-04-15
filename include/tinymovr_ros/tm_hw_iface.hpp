
#pragma once

#define HAVE_SOCKETCAN_HEADERS

#include "socketcan_cpp/socketcan_cpp.hpp"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace tinymovr_ros
{

class Tinymovr : public hardware_interface::RobotHW
{
public:
    Tinymovr() 
    { 
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle("State", &pos, &vel, &eff);
        jnt_state_interface.registerHandle(state_handle);

        registerInterface(&jnt_state_interface);

        // connect and register the joint position interface
        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle("State"), &cmd_pos);
        jnt_pos_interface.registerHandle(pos_handle);

        // connect and register the joint velocity interface
        hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle("State"), &cmd_vel);
        jnt_vel_interface.registerHandle(vel_handle);

        // connect and register the joint effort interface
        hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle("State"), &cmd_eff);
        jnt_eff_interface.registerHandle(eff_handle);

        registerInterface(&jnt_pos_interface);
    }
    bool read(const ros::Duration& period);
    bool write();

private:
    scpp::SocketCan socket_can;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;
    double cmd_pos;
    double cmd_vel;
    double cmd_eff;
    double pos;
    double vel;
    double eff;
};

}