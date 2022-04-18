
#include <tinymovr_ros/diffbot_hw_iface.hpp>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tinymovr_ros_diffbot_iface_node");

    tinymovr_ros::Diffbot db;
    ros::NodeHandle nh;
    controller_manager::ControllerManager cm(&db, nh);

    ros::Rate rate(20.0);
    ros::Time last;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
        const ros::Time now = ros::Time::now();
        const ros::Duration period = now - last;
        last = now;
        db.read(period);
        cm.update(now, period);
        db.write();
        rate.sleep();
    }
    spinner.stop();
    return 0;
}
