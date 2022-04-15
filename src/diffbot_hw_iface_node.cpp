
#include <tinymovr_ros/diffbot_hw_iface.hpp>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tinymovr_ros_diffbot_iface_node");

    tinymovr_ros::Diffbot db;
    ros::NodeHandle nh;
    controller_manager::ControllerManager cm(&db, nh);

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
        db.read(period);
        cm.update(now, period);
        db.write();
    }
    spinner.stop();
    return 0;
}
