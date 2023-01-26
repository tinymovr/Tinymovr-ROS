/*
* This file was automatically generated using Avlos.
* https://github.com/tinymovr/avlos
*
* Any changes to this file will be overwritten when
* content is regenerated.
*/

#pragma once

#include <helpers.hpp>
#include <scheduler.hpp>
#include <controller.hpp>
#include <comms.hpp>
#include <motor.hpp>
#include <encoder.hpp>
#include <traj_planner.hpp>
#include <watchdog.hpp>

static uint32_t avlos_proto_hash = 3273002564;

enum errors_flags
{
    ERRORS_NONE = 0,
    ERRORS_UNDERVOLTAGE = (1 << 0), 
    ERRORS_DRIVER_FAULT = (1 << 1)
};

enum scheduler_errors_flags
{
    SCHEDULER_ERRORS_NONE = 0,
    SCHEDULER_ERRORS_CONTROL_BLOCK_REENTERED = (1 << 0)
};

enum controller_warnings_flags
{
    CONTROLLER_WARNINGS_NONE = 0,
    CONTROLLER_WARNINGS_VELOCITY_LIMITED = (1 << 0), 
    CONTROLLER_WARNINGS_CURRENT_LIMITED = (1 << 1), 
    CONTROLLER_WARNINGS_MODULATION_LIMITED = (1 << 2)
};

enum controller_errors_flags
{
    CONTROLLER_ERRORS_NONE = 0,
    CONTROLLER_ERRORS_CURRENT_LIMIT_EXCEEDED = (1 << 0)
};

enum motor_errors_flags
{
    MOTOR_ERRORS_NONE = 0,
    MOTOR_ERRORS_PHASE_RESISTANCE_OUT_OF_RANGE = (1 << 0), 
    MOTOR_ERRORS_PHASE_INDUCTANCE_OUT_OF_RANGE = (1 << 1), 
    MOTOR_ERRORS_INVALID_POLE_PAIRS = (1 << 2)
};

enum encoder_errors_flags
{
    ENCODER_ERRORS_NONE = 0,
    ENCODER_ERRORS_CALIBRATION_FAILED = (1 << 0), 
    ENCODER_ERRORS_READING_UNSTABLE = (1 << 1)
};

enum traj_planner_errors_flags
{
    TRAJ_PLANNER_ERRORS_NONE = 0,
    TRAJ_PLANNER_ERRORS_INVALID_INPUT = (1 << 0), 
    TRAJ_PLANNER_ERRORS_VCRUISE_OVER_LIMIT = (1 << 1)
};

class Tinymovr : Node
{
    public:

        Tinymovr(uint8_t _can_node_id, send_callback _send_cb, recv_callback _recv_cb):
            Node(_can_node_id, _send_cb, _recv_cb)
            , scheduler(_can_node_id, _send_cb, _recv_cb)
            , controller(_can_node_id, _send_cb, _recv_cb)
            , comms(_can_node_id, _send_cb, _recv_cb)
            , motor(_can_node_id, _send_cb, _recv_cb)
            , encoder(_can_node_id, _send_cb, _recv_cb)
            , traj_planner(_can_node_id, _send_cb, _recv_cb)
            , watchdog(_can_node_id, _send_cb, _recv_cb) {};
        uint32_t get_protocol_hash(void);
        uint32_t get_uid(void);
        float get_Vbus(void);
        float get_Ibus(void);
        float get_power(void);
        float get_temp(void);
        bool get_calibrated(void);
        uint8_t get_errors(void);
        void save_config();
        void erase_config();
        void reset();
        Scheduler_ scheduler;
        Controller_ controller;
        Comms_ comms;
        Motor_ motor;
        Encoder_ encoder;
        Traj_planner_ traj_planner;
        Watchdog_ watchdog;

};
