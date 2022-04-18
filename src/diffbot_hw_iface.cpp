
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <tinymovr_ros/diffbot_hw_iface.hpp>

using namespace tinymovr_ros;

#define CMD_GET_ENC_ESTIMATES 0x1A
#define CMD_SET_VEL 0x1B

Diffbot::Diffbot() 
{ 
    tmcan.init();

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("A", &hw_positions_[0], &hw_velocities_[0], &hw_efforts_[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("B", &hw_positions_[1], &hw_velocities_[1], &hw_efforts_[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("A"), &hw_commands_[0]);
    jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("B"), &hw_commands_[1]);
    jnt_vel_interface.registerHandle(vel_handle_b);

    registerInterface(&jnt_vel_interface);
}

bool Diffbot::read(const ros::Duration& dt)
{
    uint8_t data[8] = {0};
    uint8_t data_len = 0;

    for (uint i = 0; i < hw_commands_.size(); i++)
    {
        if (!tmcan.read_frame(hw_node_ids_[i], CMD_GET_ENC_ESTIMATES, data, &data_len))
        {
            return false;
        }
        
        // TODO: assert data length == 8
        
        const float* hw_pos_float = reinterpret_cast<float*>(data);
        const float* hw_vel_float = reinterpret_cast<float*>(data+4);

        hw_positions_[i] = (double)(*hw_pos_float);
        hw_velocities_[i] = (double)(*hw_vel_float);
    }
    // Update the free-flyer, i.e. the base notation using the classical
    // wheel differentiable kinematics
    double base_dx = 0.5 * radius * (hw_velocities_[0] + hw_velocities_[1]) * cos(base_theta_);
    double base_dy = 0.5 * radius * (hw_velocities_[0] + hw_velocities_[1]) * sin(base_theta_);
    double base_dtheta = radius * (hw_velocities_[0] - hw_velocities_[1]) / dist_w;
    base_x_ += base_dx * dt.toSec();
    base_y_ += base_dy * dt.toSec();
    base_theta_ += base_dtheta * dt.toSec();

    return true;
}

bool Diffbot::write()
{
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
        const float command = hw_commands_[i];
        const uint8_t *data = reinterpret_cast<const uint8_t *>(&command);
        if (!tmcan.write_frame(hw_node_ids_[i], CMD_SET_VEL, data, sizeof(float)))
        {
            return false;
        }
    }
    return true;
}
