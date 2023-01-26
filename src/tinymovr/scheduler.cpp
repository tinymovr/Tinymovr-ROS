/*
* This file was automatically generated using Avlos.
* https://github.com/tinymovr/avlos
*
* Any changes to this file will be overwritten when
* content is regenerated.
*/

#include <scheduler.hpp>

uint32_t Scheduler_::get_total(void)
{
    uint32_t value = 0;
    this->send(11, this->_data, 0, true);
    if (this->recv(11, this->_data, &(this->_dlc), RECV_DELAY_US)) 
    {
        read_le(&value, this->_data);
    }
    return value;
}

uint32_t Scheduler_::get_busy(void)
{
    uint32_t value = 0;
    this->send(12, this->_data, 0, true);
    if (this->recv(12, this->_data, &(this->_dlc), RECV_DELAY_US)) 
    {
        read_le(&value, this->_data);
    }
    return value;
}

uint8_t Scheduler_::get_errors(void)
{
    uint8_t value = 0;
    this->send(13, this->_data, 0, true);
    if (this->recv(13, this->_data, &(this->_dlc), RECV_DELAY_US)) 
    {
        read_le(&value, this->_data);
    }
    return value;
}



