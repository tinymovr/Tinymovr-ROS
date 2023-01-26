/*
* This file was automatically generated using Avlos.
* https://github.com/tinymovr/avlos
*
* Any changes to this file will be overwritten when
* content is regenerated.
*/

#pragma once

#include <helpers.hpp>

class Encoder_ : Node
{
    public:

        Encoder_(uint8_t _can_node_id, send_callback _send_cb, recv_callback _recv_cb):
            Node(_can_node_id, _send_cb, _recv_cb) {};
        float get_position_estimate(void);
        float get_velocity_estimate(void);
        uint8_t get_type(void);
        void set_type(uint8_t value);
        float get_bandwidth(void);
        void set_bandwidth(float value);
        bool get_calibrated(void);
        uint8_t get_errors(void);

};
