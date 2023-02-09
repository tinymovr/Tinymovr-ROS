
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <tm_joint_iface.hpp>

using namespace std;

namespace tinymovr_ros
{

 
TinymovrCAN tmcan;

// ---------------------------------------------------------------
/*
 * Function:  send_cb 
 * --------------------
 *  Is called to send a CAN frame
 *
 *  arbitration_id: the frame arbitration id
 *  data: pointer to the data array to be transmitted
 *  data_size: the size of transmitted data
 *  rtr: if the frame is of request transmit type (RTR)
 */
void send_cb(uint32_t arbitration_id, uint8_t *data, uint8_t data_size, bool rtr)
{
    if (!tmcan.write_frame(arbitration_id, data, data_size))
    {
        throw "CAN write error";
    }
}

/*
 * Function:  recv_cb 
 * --------------------
 *  Is called to receive a CAN frame
 *
 *  arbitration_id: the frame arbitration id
 *  data: pointer to the data array to be received
 *  data_size: pointer to the variable that will hold the size of received data
 */
bool recv_cb(uint32_t arbitration_id, uint8_t *data, uint8_t *data_size)
{
    (void)arbitration_id;
    if (!tmcan.read_frame(arbitration_id, 0, data, data_size))
    {
        throw "CAN read error";
    }
    return true;
}
// ---------------------------------------------------------------

TinymovrJoint::TinymovrJoint() {}

TinymovrJoint::~TinymovrJoint() {}

bool TinymovrJoint::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    
    XmlRpc::XmlRpcValue servos_param;

    bool got_all_params = true;

    // build servo instances from configuration
    if (got_all_params &= robot_hw_nh.getParam("joints", servos_param)) {
            ROS_ASSERT(servos_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        try {
            for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = servos_param.begin(); it != servos_param.end(); ++it) {

                ROS_DEBUG_STREAM("servo: " << (std::string)(it->first));

                id_t id;
                if (it->second.hasMember("id"))
                {
                    id = static_cast<int>(servos_param[it->first]["id"]);
                    ROS_DEBUG_STREAM("\tid: " << (int)id);
                    servos.push_back(Tinymovr(id, &send_cb, &recv_cb));
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
            + sub_namespace + "/servos";
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

    tmcan.init();

    // initialize servos with correct mode
    for (int i=0; i<num_joints; i++)
    {
        ROS_ASSERT((servos[i].encoder.get_calibrated() == true) && (servos[i].motor.get_calibrated() == true));
        servos[i].controller.set_state(2);
        servos[i].controller.set_mode(_str2mode(servo_modes[i]));
        ros::Duration(0.001).sleep();
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
    template <class Protocol>
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