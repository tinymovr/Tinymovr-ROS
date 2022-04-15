
#pragma once

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
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_a);

        hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_b);

        registerInterface(&jnt_state_interface);

        // connect and register the joint velocity interface
        hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
        jnt_vel_interface.registerHandle(vel_handle_a);

        hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
        jnt_vel_interface.registerHandle(vel_handle_b);

        registerInterface(&jnt_vel_interface);
    }
    bool read(const ros::Duration& period);
    bool write();

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];
};

}