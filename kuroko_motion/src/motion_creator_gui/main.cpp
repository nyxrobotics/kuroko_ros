#include "main.h"

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "motion_creator_gui");
  ros::NodeHandle node_handle("~");
  ros::spin();
  ros::shutdown();
  return 0;
}
