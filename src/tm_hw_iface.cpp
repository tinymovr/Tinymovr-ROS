
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <tm_hw_iface.hpp>

using namespace tinymovr_ros;

TinymovrHW::TinymovrHW(): tm(1, send_cb, recv_cb)
{ 
    tmcan.init();

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("Actuator", &hw_positions_[0], &hw_velocities_[0], &hw_efforts_[0]);
    jnt_state_interface.registerHandle(state_handle_a);
    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("Actuator"), &hw_commands_[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);
    registerInterface(&jnt_pos_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("Actuator"), &hw_commands_[1]);
    jnt_vel_interface.registerHandle(vel_handle_a);
    registerInterface(&jnt_vel_interface);

    // connect and register the joint effort interface
    hardware_interface::JointHandle eff_handle_a(jnt_state_interface.getHandle("Actuator"), &hw_commands_[2]);
    jnt_eff_interface.registerHandle(eff_handle_a);
    registerInterface(&jnt_eff_interface);
}

bool TinymovrHW::read(const ros::Duration& dt)
{
    pos = tm.encoder.get_position_estimate();
    vel = tm.encoder.get_velocity_estimate();
    eff = tm.controller.current.get_Iq_estimate();
    return true;
}

bool TinymovrHW::write()
{
    tm.controller.position.set_setpoint(cmd_pos);
    tm.controller.velocity.set_setpoint(cmd_vel);
    tm.controller.current.set_Iq_setpoint(cmd_eff);
    return true;
}