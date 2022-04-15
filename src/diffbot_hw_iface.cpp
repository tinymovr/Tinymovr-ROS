
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <tinymovr_ros/diffbot_hw_iface.hpp>

using namespace tinymovr_ros;

#define CMD_GET_ENC_ESTIMATES 0x1A
#define CMD_SET_VEL 0x1B

bool Diffbot::read(const ros::Duration& dt)
{
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
        if (write_frame(make_arbitration_id(hw_node_ids_[i], CMD_GET_ENC_ESTIMATES), 0, 0))
        {
            return false;
        }

        if ()
        {
            return false;
        }
        
        const float* hw_pos_float = reinterpret_cast<float*>(fr.data);
        const float* hw_vel_float = reinterpret_cast<float*>(fr.data+4);

        hw_positions_[i] = (double)*hw_pos_float;
        hw_velocities_[i] = (double)*hw_vel_float;
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
        if (write_frame(make_arbitration_id(hw_node_ids_[i], CMD_SET_VEL), data, sizeof(float)) != scpp::STATUS_OK)
        {
            return false;
        }
    }
    return true;
}

bool Diffbot::write_frame(uint32_t arb_id, const uint8_t *data, uint8_t data_len)
{
    
}

uint32_t Diffbot::make_arbitration_id(uint32_t node_id, uint32_t command_id)
{
    return node_id << 6 | command_id;
}