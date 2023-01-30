
#include <tm_joint_iface.hpp>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tinymovr_joint_iface_node");

    tinymovr_ros::TinymovrJoint tm_joint;
    ros::NodeHandle nh;
    controller_manager::ControllerManager cm(&tm, nh);

    ros::Rate rate(20.0);
    ros::Time last;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
        const ros::Time now = ros::Time::now();
        const ros::Duration period = now - last;
        last = now;
        tm_joint.read(period);
        cm.update(now, period);
        tm_joint.write();
        rate.sleep();
    }
    spinner.stop();
    return 0;
}
