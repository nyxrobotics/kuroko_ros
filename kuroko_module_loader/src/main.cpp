#include "kuroko_module_loader.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kuroko_module_loader");
  ros::NodeHandle nh;

  KurokoModuleLoader manager(nh);
  manager.initialize();
  manager.start();

  return 0;
}
