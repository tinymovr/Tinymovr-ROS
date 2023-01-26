/*
* This file was automatically generated using Avlos.
* https://github.com/tinymovr/avlos
*
* Any changes to this file will be overwritten when
* content is regenerated.
*/

#include <voltage.hpp>

float Voltage_::get_Vq_setpoint(void)
{
    float value = 0;
    this->send(34, this->_data, 0, true);
    if (this->recv(34, this->_data, &(this->_dlc), RECV_DELAY_US)) 
    {
        read_le(&value, this->_data);
    }
    return value;
}



