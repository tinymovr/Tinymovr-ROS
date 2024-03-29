#include <iostream>
#include <ros/ros.h>
#include <tm_joint_iface.hpp>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tinymovr_joint_iface");
   
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    
    tinymovr_ros::TinymovrJoint tm_joint;

    bool init_success = tm_joint.init(nh, nh);

    controller_manager::ControllerManager cm(&tm_joint, nh);

    ros::Rate rate(100); // 100Hz update rate

    ROS_INFO("tinymovr joint interface started");
    while (ros::ok())
    {
        tm_joint.read(ros::Time::now(), rate.expectedCycleTime());
        cm.update(ros::Time::now(), rate.expectedCycleTime());
        tm_joint.write(ros::Time::now(), rate.expectedCycleTime());
        rate.sleep();
    }

    tm_joint.shutdown();

    spinner.stop();
    return 0;
}
