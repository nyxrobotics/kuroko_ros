#include "kuroko_localization/kuroko_localization.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kuroko_localization");
  kuroko_localization::KurokoLocalization kuroko_localization;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    kuroko_localization.process();
    loop_rate.sleep();
  }
  return 0;
}
