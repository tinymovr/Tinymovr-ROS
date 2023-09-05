
#include <tinymovr_can.hpp>
#include <ros/console.h>


namespace tinymovr_ros
{

void TinymovrCAN::init()
{
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
}

bool TinymovrCAN::read_frame(uint32_t* can_id, uint8_t* data, uint8_t* len)
{
    ROS_INFO("in read frame");
    scpp::CanFrame fr;
    scpp::SocketCanStatus read_status = socket_can.read(fr);
    if (read_status == scpp::STATUS_OK)
    {
        *can_id = fr.id;
        *len = fr.len;
        std::copy(fr.data, fr.data + fr.len, data);
        ROS_DEBUG("Successfully read a CAN frame");
        return true;
    }
    else
    {
        ROS_WARN("Failed to read a CAN frame. SocketCanStatus: %d", static_cast<int>(read_status));
        switch(read_status)
        {
            case scpp::STATUS_READ_ERROR:
                ROS_ERROR("SocketCan read error!");
                break;
            // Removed STATUS_TIMEOUT case
            default:
                ROS_ERROR("Unknown SocketCan error!");
                break;
        }
        return false;
    }
}



bool TinymovrCAN::write_frame(uint32_t node_id, uint32_t ep_id, const uint8_t *data, uint8_t data_len)
{
    ROS_INFO("in write frame1");
    return write_frame(make_arbitration_id(node_id, ep_id), data, data_len);
}

bool TinymovrCAN::write_frame(uint32_t arbitration_id, const uint8_t *data, uint8_t data_len)
{
    ROS_INFO("in write frame2");
    scpp::CanFrame cf_to_write;

    cf_to_write.id = arbitration_id;
    cf_to_write.len = data_len;
    for (int i = 0; i < data_len; ++i)
        cf_to_write.data[i] = data[i];
    auto write_sc_status = socket_can.write(cf_to_write);
    if (write_sc_status != scpp::STATUS_OK)
        return false;
    return true;
}

uint32_t TinymovrCAN::make_arbitration_id(uint32_t node_id, uint32_t command_id)
{
    return node_id << 6 | command_id;
}

}