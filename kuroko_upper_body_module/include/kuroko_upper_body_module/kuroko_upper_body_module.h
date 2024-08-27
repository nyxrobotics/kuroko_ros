#ifndef KUROKO_UPPER_BODY_MODULE_H_
#define KUROKO_UPPER_BODY_MODULE_H_

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <map>
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace motion_control
{
class KurokoUpperBodyModule : public robotis_framework::MotionModule,
                              public robotis_framework::Singleton<KurokoUpperBodyModule>
{
public:
  KurokoUpperBodyModule();
  virtual ~KurokoUpperBodyModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, double> sensors) override;

private:
  void updateUpperBodyMotion();

  ros::Time last_msg_time_;
  std::map<std::string, double> previous_result_;
  double previous_volt_;
  double present_volt_;
  int control_cycle_msec_;
  bool DEBUG_PRINT;
};

}  // namespace motion_control

#endif /* KUROKO_UPPER_BODY_MODULE_H_ */
