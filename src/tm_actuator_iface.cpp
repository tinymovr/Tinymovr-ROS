
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <tm_actuator_iface.hpp>

using namespace tinymovr_ros;

TinymovrHW::TinymovrActuator(): tm(1, send_cb, recv_cb)
{ 
    tmcan.init();

    // connect and register the actuator state interface
    hardware_interface::ActuatorStateHandle state_handle_a("Actuator", &hw_positions_[0], &hw_velocities_[0], &hw_efforts_[0]);
    act_state_interface.registerHandle(state_handle_a);
    registerInterface(&act_state_interface);

    // connect and register the actuator position interface
    hardware_interface::ActuatorHandle pos_handle_a(act_state_interface.getHandle("Actuator"), &hw_commands_[0]);
    act_pos_interface.registerHandle(pos_handle_a);
    registerInterface(&act_pos_interface);

    // connect and register the actuator velocity interface
    hardware_interface::ActuatorHandle vel_handle_a(act_state_interface.getHandle("Actuator"), &hw_commands_[1]);
    act_vel_interface.registerHandle(vel_handle_a);
    registerInterface(&act_vel_interface);

    // connect and register the actuator effort interface
    hardware_interface::ActuatorHandle eff_handle_a(act_state_interface.getHandle("Actuator"), &hw_commands_[2]);
    act_eff_interface.registerHandle(eff_handle_a);
    registerInterface(&act_eff_interface);
}

bool TinymovrActuator::read(const ros::Duration& dt)
{
    pos = tm.encoder.get_position_estimate();
    vel = tm.encoder.get_velocity_estimate();
    eff = tm.controller.current.get_Iq_estimate();
    return true;
}

bool TinymovrActuator::write()
{
    tm.controller.position.set_setpoint(cmd_pos);
    tm.controller.velocity.set_setpoint(cmd_vel);
    tm.controller.current.set_Iq_setpoint(cmd_eff);
    return true;
}