#include <ros/ros.h>
#include "kuroko_optimization/kuroko_optimization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kuroko_optimization");
  ros::NodeHandle nh("~");

  KurokoOptimization optimizer;

  ros::ServiceServer service = nh.advertiseService(
      "/motion_control/online_walking/get_preview_matrix",
      &KurokoOptimization::getPreviewMatrixCallback,
      &optimizer);

  ros::spin();
  return 0;
}
