
#pragma once

#include "socketcan_cpp/socketcan_cpp.hpp"
#include <ros/ros.h>

namespace tinymovr_ros
{

class TinymovrCAN
{
public:
    void init();
    bool read_frame(uint32_t node_id, uint32_t ep_id, uint8_t* data, uint8_t* data_len);
    bool write_frame(uint32_t node_id, uint32_t ep_id, const uint8_t *data, uint8_t data_len);
    uint32_t make_arbitration_id(uint32_t node_id, uint32_t command_id);
private:
    scpp::SocketCan socket_can;
};

}
