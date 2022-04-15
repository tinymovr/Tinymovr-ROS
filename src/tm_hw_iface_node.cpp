
#include <tinymovr_ros/tm_hw_iface.hpp>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tinymovr_ros_iface_node");

    tinymovr_ros::Tinymovr tm;
    ros::NodeHandle nh;
    controller_manager::ControllerManager cm(&tm, nh);

    ros::Time last, now;
    now = last = ros::Time::now();
    ros::Duration period(0.05);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
        now = ros::Time::now();
        period = now - last;
        last = now;
        tm.read(period);
        cm.update(now, period);
        tm.write();
    }
    spinner.stop();
    return 0;
}
