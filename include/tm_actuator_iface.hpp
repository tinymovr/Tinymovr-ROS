
#pragma once

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <tinymovr_ros/tinymovr_can.hpp>

namespace tinymovr_ros
{

// ---------------------------------------------------------------
/*
 * Function:  send_cb 
 * --------------------
 *  Is called to send a CAN frame
 *
 *  arbitration_id: the frame arbitration id
 *  data: pointer to the data array to be transmitted
 *  data_size: the size of transmitted data
 *  rtr: if the ftame is of request transmit type (RTR)
 */
void send_cb(uint32_t arbitration_id, uint8_t *data, uint8_t data_size, bool rtr)
{
    return tmcan.write_frame(arbitration_id, data, data_size)
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
    return tmcan.read_frame(hw_node_ids_[i], CMD_GET_ENC_ESTIMATES, data, &data_size);
}
// ---------------------------------------------------------------


class TinymovrActuator : public hardware_interface::RobotHW
{
public:
    TinymovrActuator();
    bool read(const ros::Duration& period);
    bool write();

private:
    hardware_interface::ActuatorStateInterface act_state_interface;
    hardware_interface::PositionActuatorInterface act_pos_interface;
    hardware_interface::VelocityActuatorInterface act_vel_interface;
    hardware_interface::EffortActuatorInterface act_eff_interface;
    double cmd_pos;
    double cmd_vel;
    double cmd_eff;
    double pos;
    double vel;
    double eff;
    uint8_t node_id;

    Tinymovr tm;
};

}