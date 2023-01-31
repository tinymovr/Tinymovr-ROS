
#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <tinymovr_can.hpp>

namespace tinymovr_ros
{

class TinymovrJoint : public hardware_interface::RobotHW
{
public:
    TinymovrJoint();
    ~TinymovrJoint();
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

protected:
    ros::NodeHandle nh_;
    
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_pos_interface;
    hardware_interface::VelocityJointInterface joint_vel_interface;
    hardware_interface::EffortJointInterface joint_eff_interface;

    int num_joints;
    std::vector<string> joint_names;
    std::vector<uint8_t> joint_ids;
    
    std::vector<double> joint_position_command;
    std::vector<double> joint_velocity_command;
    std::vector<double> joint_effort_command;
    std::vector<double> joint_position_state;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;

    std::vector<Tinymovr> servos;
};

}