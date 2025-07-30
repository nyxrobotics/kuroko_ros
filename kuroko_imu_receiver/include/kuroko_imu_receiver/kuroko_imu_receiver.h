#ifndef KUROKO_IMU_RECEIVER_H_
#define KUROKO_IMU_RECEIVER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "ros/subscriber.h"

namespace motion_control
{
class KurokoImuReceiver : public robotis_framework::SensorModule, public robotis_framework::Singleton<KurokoImuReceiver>
{
public:
  KurokoImuReceiver();
  virtual ~KurokoImuReceiver();

  /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot);
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, robotis_framework::Sensor*> sensors);

private:
  void queueThread();
  int control_cycle_msec_;
  boost::thread queue_thread_;
  ros::Time last_msg_time_;
  std::map<std::string, double> previous_result_;
  sensor_msgs::Imu imu_msg_;
  std::string robot_name_;

  /* subscriber & publisher */
  ros::Subscriber imu_sub_;
  void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
  Eigen::Vector3d imuQuaternionToRollPitchYaw(const Eigen::Quaterniond& q);
  double wrapToPi(double angle);
};

}  // namespace motion_control

#endif /* KUROKO_IMU_RECEIVER_H_ */
