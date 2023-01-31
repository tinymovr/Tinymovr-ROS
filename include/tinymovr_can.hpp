
#pragma once

#include "socketcan_cpp/socketcan_cpp.hpp"
#include <ros/ros.h>

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
    if (!tmcan.read_frame(hw_node_ids_[i], CMD_GET_ENC_ESTIMATES, data, &data_size))
    {
        throw "CAN read error";
    }
}
// ---------------------------------------------------------------

class TinymovrCAN
{
public:
    void init();
    bool read_frame(uint32_t node_id, uint32_t ep_id, uint8_t* data, uint8_t* data_len);
    bool write_frame(uint32_t node_id, uint32_t ep_id, const uint8_t *data, uint8_t data_len);
    bool write_frame(uint32_t arbitration_id, const uint8_t *data, uint8_t data_len);
    uint32_t make_arbitration_id(uint32_t node_id, uint32_t command_id);
private:
    scpp::SocketCan socket_can;
};

}
