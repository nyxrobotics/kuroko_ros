#include "kuroko_device_manager.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kuroko_device_manager");
  ros::NodeHandle nh;

  KurokoDeviceManager manager(nh);
  manager.initialize();
  manager.start();

  return 0;
}
