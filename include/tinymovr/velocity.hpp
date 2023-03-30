/*
* This file was automatically generated using Avlos.
* https://github.com/tinymovr/avlos
*
* Any changes to this file will be overwritten when
* content is regenerated.
*/

#pragma once

#include <helpers.hpp>

class Velocity_ : Node
{
    public:

        Velocity_(uint8_t _can_node_id, send_callback _send_cb, recv_callback _recv_cb, delay_us_callback _delay_us_cb):
            Node(_can_node_id, _send_cb, _recv_cb, _delay_us_cb) {};
        float get_setpoint(void);
        void set_setpoint(float value);
        float get_limit(void);
        void set_limit(float value);
        float get_p_gain(void);
        void set_p_gain(float value);
        float get_i_gain(void);
        void set_i_gain(float value);
        float get_deadband(void);
        void set_deadband(float value);
        float get_increment(void);
        void set_increment(float value);

};
