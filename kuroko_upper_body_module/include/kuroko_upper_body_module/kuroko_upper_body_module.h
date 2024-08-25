#ifndef KUROKO_UPPER_BODY_MODULE_H_
#define KUROKO_UPPER_BODY_MODULE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <map>
#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

namespace motion_control
{
class KurokoUpperBodyModule : public robotis_framework::SensorModule,
                              public robotis_framework::Singleton<KurokoUpperBodyModule>
{
public:
  KurokoUpperBodyModule();
  virtual ~KurokoUpperBodyModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, robotis_framework::Sensor*> sensors) override;

private:
  void queueThread();
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void updateUpperBodyMotion();
  double lowPassFilter(double alpha, double x_new, double& x_old);

  Eigen::Quaterniond getCurrentChestOrientation();  // Function to return chest orientation
  double getCurrentChestAngle();                    // Function to return chest angle

  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher dxl_power_msg_pub_;

  sensor_msgs::Imu imu_msg_;
  double previous_volt_;
  double present_volt_;
  std::map<std::string, double> previous_result_;
  std::map<std::string, double> current_chest_angles_;
  ros::Time last_msg_time_;
  int control_cycle_msec_;
  boost::thread queue_thread_;
  bool DEBUG_PRINT;
};

}  // namespace motion_control

#endif /* KUROKO_UPPER_BODY_MODULE_H_ */
