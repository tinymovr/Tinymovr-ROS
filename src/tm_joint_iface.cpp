
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdexcept>

#include <ros/console.h>

#include "socketcan_cpp/socketcan_cpp.hpp"
#include <tm_joint_iface.hpp>

using namespace std;

namespace tinymovr_ros
{

 
scpp::SocketCan socket_can;

// ---------------------------------------------------------------
/*
 * Function:  send_cb 
 * --------------------
 *  Is called to send a CAN frame
 *
 *  arbitration_id: the frame arbitration id
 *  data: pointer to the data array to be transmitted
 *  data_length: the size of transmitted data
 *  rtr: if the frame is of request transmit type (RTR)
 */
void send_cb(uint32_t arbitration_id, uint8_t *data, uint8_t data_length, bool rtr)
{
    ROS_DEBUG_STREAM("Attempting to write CAN frame with arbitration_id: " << arbitration_id);

    scpp::CanFrame cf_to_write;

    cf_to_write.id = arbitration_id;
    cf_to_write.len = data_length;
    for (int i = 0; i < data_length; ++i)
        cf_to_write.data[i] = data[i];
    auto write_sc_status = socket_can.write(cf_to_write);
    if (write_sc_status != scpp::STATUS_OK)
    {
        throw std::runtime_error("CAN write error");
    }
    else
    {
        ROS_DEBUG_STREAM("CAN frame with arbitration_id: " << arbitration_id << " written successfully.");
    }
}

/*
 * Function:  recv_cb 
 * --------------------
 *  Is called to receive a CAN frame
 *
 *  arbitration_id: the frame arbitration id pointer
 *  data: pointer to the data array to be received
 *  data_length: pointer to the variable that will hold the size of received data
 */
bool recv_cb(uint32_t *arbitration_id, uint8_t *data, uint8_t *data_length)
{
    ROS_DEBUG_STREAM("Attempting to read CAN frame...");

    scpp::CanFrame fr;
    scpp::SocketCanStatus read_status = socket_can.read(fr);
    if (read_status == scpp::STATUS_OK)
    {
        *arbitration_id = fr.id;
        *data_length = fr.len;
        std::copy(fr.data, fr.data + fr.len, data);
        ROS_DEBUG_STREAM("CAN frame with arbitration_id: " << *arbitration_id << " read successfully.");
        return true;
    }
    else
    {
        switch(read_status)
        {
            case scpp::STATUS_READ_ERROR:
                throw std::runtime_error("SocketCAN read error");
                break;
            // Removed STATUS_TIMEOUT case
            default:
                throw std::runtime_error("Unknown SocketCAN error");
                break;
        }
        return false;
    }
}

/*
 * Function:  delay_us_cb 
 * --------------------
 *  Is called to perform a delay
 *
 *  us: the microseconds to wait for
 */
void delay_us_cb(uint32_t us)
{
  ros::Duration(us * 1e-6).sleep();
}
// ---------------------------------------------------------------

TinymovrJoint::TinymovrJoint() {}

TinymovrJoint::~TinymovrJoint() {}

bool TinymovrJoint::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    
    XmlRpc::XmlRpcValue servos_param;

    bool got_all_params = true;

    // build servo instances from configuration
    if (got_all_params &= robot_hw_nh.getParam("/tinymovr_joint_iface/joints", servos_param)) {
            ROS_ASSERT(servos_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        try {
            for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = servos_param.begin(); it != servos_param.end(); ++it) {

                ROS_INFO_STREAM("servo: " << (std::string)(it->first));

                id_t id;
                int delay_us;
                if (it->second.hasMember("id"))
                {
                    id = static_cast<int>(servos_param[it->first]["id"]);
                    delay_us = static_cast<int>(servos_param[it->first]["delay_us"]);
                    ROS_INFO_STREAM("\tid: " << (int)id);
                    servos.push_back(Tinymovr(id, &send_cb, &recv_cb, &delay_us_cb, delay_us));
                    servo_modes.push_back(servos_param[it->first]["command_interface"]);
                }
                else
                {
                    ROS_ERROR_STREAM("servo " << it->first << " has no associated servo ID.");
                    continue;
                }
            }
        }
        catch (XmlRpc::XmlRpcException& e) 
        {
            ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                << "configuration: " << e.getMessage() << ".\n"
                << "Please check the configuration, particularly parameter types.");
            return false;
        }
    }

    if (!got_all_params) {
        std::string sub_namespace = robot_hw_nh.getNamespace();
        std::string error_message = "One or more of the following parameters "
                            "were not set:\n"
            + sub_namespace + "/tinymovr_joint_iface/joints";
        ROS_FATAL_STREAM(error_message);
        return false;
    }

    num_joints = servos_param.size();

    joint_position_command.resize(num_joints, 0.0);
    joint_velocity_command.resize(num_joints, 0.0);
    joint_effort_command.resize(num_joints, 0.0);
    joint_position_state.resize(num_joints, 0.0);
    joint_velocity_state.resize(num_joints, 0.0);
    joint_effort_state.resize(num_joints, 0.0);

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

    // initialize servos with correct mode
    for (int i=0; i<num_joints; i++)
    {
        ROS_INFO("Asserting calibrated");
        ROS_ASSERT((servos[i].encoder.get_calibrated() == true) && (servos[i].motor.get_calibrated() == true));
        ROS_INFO("Setting state");
        servos[i].controller.set_state(2);
        ROS_INFO("Setting mode");
        servos[i].controller.set_mode(_str2mode(servo_modes[i]));
        ros::Duration(0.001).sleep();
        ROS_INFO("Asserting state and mode");
        ROS_ASSERT((servos[i].controller.get_state() == 2) && (servos[i].controller.get_mode() == 2));
    }

    // register state interfaces
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
        hardware_interface::JointHandle pos_handle(joint_state_interface.getHandle("Joint"), &joint_position_command[i]);
        joint_pos_interface.registerHandle(pos_handle);
        
        // connect and register the joint velocity interface
        hardware_interface::JointHandle vel_handle(joint_state_interface.getHandle("Joint"), &joint_velocity_command[i]);
        joint_vel_interface.registerHandle(vel_handle);
        
        // connect and register the joint effort interface
        hardware_interface::JointHandle eff_handle(joint_state_interface.getHandle("Joint"), &joint_effort_command[i]);
        joint_eff_interface.registerHandle(eff_handle);
    }

    registerInterface(&joint_state_interface);
    registerInterface(&joint_pos_interface);
    registerInterface(&joint_vel_interface);
    registerInterface(&joint_eff_interface);

    return true;
}

 /** Convert a string to an operating mode
        @param mode_string name of the operating mode (current or effort, velocity or position)
        @return mode index corresponding to the index of the mode
    **/
    uint8_t TinymovrJoint::_str2mode(std::string& mode_string)
    {
        if ("current" == mode_string || "effort" == mode_string)
            return 0;
        if ("velocity" == mode_string)
            return 1;
        else if ("position" == mode_string)
            return 2;
        else {
            ROS_ERROR_STREAM("The command mode " << mode_string << " is not available");
            return 0;
        }
    }

void TinymovrJoint::read(const ros::Time& time, const ros::Duration& period)
{
    try {
        for (int i=0; i<servos.size(); i++)
        {
            joint_position_state[i] = servos[i].encoder.get_position_estimate();
            joint_velocity_state[i] = servos[i].encoder.get_velocity_estimate();
            joint_effort_state[i] = servos[i].controller.current.get_Iq_estimate();
        }
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Error while reading Tinymovr CAN:\n" << e.what());
    }
}

void TinymovrJoint::write(const ros::Time& time, const ros::Duration& period)
{
    try {
        for (int i=0; i<servos.size(); i++)
        {
            servos[i].controller.position.set_setpoint(joint_position_command[i]);
            servos[i].controller.velocity.set_setpoint(joint_velocity_command[i]);
            servos[i].controller.current.set_Iq_setpoint(joint_effort_command[i]);
        }
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Error while writing Tinymovr CAN:\n" << e.what());
    }
}

}