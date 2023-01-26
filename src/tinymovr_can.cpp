
#include <tinymovr_can.hpp>

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

bool TinymovrCAN::read_frame(uint32_t node_id, uint32_t ep_id, uint8_t* data, uint8_t* data_len)
{
    if (scpp::STATUS_OK != write_frame(node_id, ep_id, 0, 0))
    {
        return false;
    }

    scpp::CanFrame fr;

    if (scpp::STATUS_OK != socket_can.read(fr))
    {
        return false;
    }

    memcpy(data, fr.data, 8); // TODO: make safer
    *data_len = fr.len;

    return true;
}

bool TinymovrCAN::write_frame(uint32_t node_id, uint32_t ep_id, const uint8_t *data, uint8_t data_len)
{
    return write_frame(make_arbitration_id(node_id, ep_id));
}

bool TinymovrCAN::write_frame(uint32_t arbitration_id, const uint8_t *data, uint8_t data_len)
{
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