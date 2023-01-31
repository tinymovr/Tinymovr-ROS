
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <tm_joint_iface.hpp>

using namespace tinymovr_ros;

TinymovrJoint::TinymovrJoint() {}

bool TinymovrJoint::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    tmcan.init();

    //get joint names and num of joint
    robot_hw_nh.getParam("joints/names", joint_names);
    robot_hw_nh.getParam("joints/ids", joint_ids);
    num_joints = joint_name.size();

    joint_position_command.resize(num_joints, 0.0);
    joint_velocity_command.resize(num_joints, 0.0);
    joint_effort_command.resize(num_joints, 0.0);
    joint_position_state.resize(num_joints, 0.0);
    joint_velocity_state.resize(num_joints, 0.0);
    joint_effort_state.resize(num_joints, 0.0);

    // initialize servos with correct mode
    for (int i=0; i<num_joints; i++)
    {
        servos.push_back(Tinymovr(joint_ids[i], &read_cb, &write_cb);)
        ROS_ASSERT((servos[i].encoder.calibrated == true) && (servos[i].motor.calibrated == true))
        servos[i].controller.set_state(2);
        servos[i].controller.set_mode(2);
        ros::Duration(0.001).sleep();
        ROS_ASSERT((servos[i].controller.get_state() == 2) && (servos[i].controller.get_mode() == 2))
    }

    for (int i=0; i<num_joints; i++)
    {
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle(
            joint_name[i],
            &joint_position_state[i],
            &joint_velocity_state[i],
            &joint_effort_state[i]
            );
        joint_state_interface.registerHandle(state_handle);
        
        // connect and register the joint position interface
        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle("Joint"), &cmd_pos_[i]);
        joint_pos_interface.registerHandle(pos_handle);
        
        // connect and register the joint velocity interface
        hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle("Joint"), &cmd_vel_[i]);
        joint_vel_interface.registerHandle(vel_handle);
        
        // connect and register the joint effort interface
        hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle("Joint"), &cmd_eff_[i]);
        joint_eff_interface.registerHandle(eff_handle);
    }

    registerInterface(&joint_state_interface);
    registerInterface(&joint_pos_interface);
    registerInterface(&joint_vel_interface);
    registerInterface(&joint_eff_interface);

    return true;
}

bool TinymovrJoint::read(const ros::Duration& dt)
{
    try {
        for (int i=0; i<servos_.size(); i++)
        {
            joint_position_state[i] = servos[i].encoder.get_position_estimate();
            joint_velocity_state[i] = servos[i].encoder.get_velocity_estimate();
            joint_effort_state[i] = servos[i].controller.current.get_Iq_estimate();
        }
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Error while reading Tinymovr CAN:\n" << e.what());
        return false;
    }
}

bool TinymovrJoint::write()
{
    try {
        for (int i=0; i<servos_.size(); i++)
        {
            servos[i].controller.position.set_setpoint(joint_position_command[i]);
            servos[i].controller.velocity.set_setpoint(joint_velocity_command[i]);
            servos[i].controller.current.set_Iq_setpoint(joint_effort_command[i]);
        }
        return true;
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Error while reading Tinymovr CAN:\n" << e.what());
        return false;
    }
}