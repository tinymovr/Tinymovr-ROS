
#pragma once

// ROS
#include <ros/ros.h>
#include <XmlRpcException.h>
#include <XmlRpcValue.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <tinymovr_can.hpp>
#include <tinymovr/tinymovr.hpp>

using namespace std;

namespace tinymovr_ros
{

class TinymovrJoint : public hardware_interface::RobotHW
{
public:
    TinymovrJoint();
    ~TinymovrJoint();
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    void write(const ros::Time& /*time*/, const ros::Duration& /*period*/);

protected:
    ros::NodeHandle nh_;
    
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface joint_pos_interface;
    hardware_interface::VelocityJointInterface joint_vel_interface;
    hardware_interface::EffortJointInterface joint_eff_interface;

    int num_joints;
    std::vector<string> joint_name;
    std::vector<int> joint_id;
    
    std::vector<double> joint_position_command;
    std::vector<double> joint_velocity_command;
    std::vector<double> joint_effort_command;
    std::vector<double> joint_position_state;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;

    std::vector<Tinymovr> servos;
    std::vector<std::string> servo_modes;

    uint8_t _str2mode(const std::string& mode_string);
};

}