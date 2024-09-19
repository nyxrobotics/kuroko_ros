#include "roboone_auto.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboone_auto");
    ros::NodeHandle nh;

    RobooneAuto roboone_auto(nh);
    roboone_auto.init();  // Initialize modules and states

    ros::spin();  // Keep the node running

    return 0;
}
