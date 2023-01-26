/*
* This file was automatically generated using Avlos.
* https://github.com/tinymovr/avlos
*
* Any changes to this file will be overwritten when
* content is regenerated.
*/

#include <position.hpp>

float Position_::get_setpoint(void)
{
    float value = 0;
    this->send(18, this->_data, 0, true);
    if (this->recv(18, this->_data, &(this->_dlc), RECV_DELAY_US)) 
    {
        read_le(&value, this->_data);
    }
    return value;
}

void Position_::set_setpoint(float value)
{
    write_le(value, this->_data);
    this->send(18, this->_data, sizeof(float), false);
}

float Position_::get_p_gain(void)
{
    float value = 0;
    this->send(19, this->_data, 0, true);
    if (this->recv(19, this->_data, &(this->_dlc), RECV_DELAY_US)) 
    {
        read_le(&value, this->_data);
    }
    return value;
}

void Position_::set_p_gain(float value)
{
    write_le(value, this->_data);
    this->send(19, this->_data, sizeof(float), false);
}



