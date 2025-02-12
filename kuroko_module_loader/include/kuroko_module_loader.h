#ifndef KUROKO_MODULE_LOADER_H
#define KUROKO_MODULE_LOADER_H

#include "kuroko_initial_pose_module/initial_pose_module.h"
#include "kuroko_joint_controller/kuroko_joint_controller.h"
#include "kuroko_imu_receiver/kuroko_imu_receiver.h"
// #include "kuroko_online_walking_module/online_walking_module.h"
#include "kuroko_tuning_module/tuning_module.h"
#include "kuroko_walking_module/kuroko_walking_module.h"
#include "kuroko_action_module/action_module.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace robotis_framework;
using namespace dynamixel;
using namespace motion_control;

class KurokoModuleLoader
{
public:
  KurokoModuleLoader(ros::NodeHandle& nh);
  ~KurokoModuleLoader();

  void initialize();
  void start();

private:
  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
  void dxlTorqueCheckCallback(const std_msgs::String::ConstPtr& msg);
  void loadParameters(ros::NodeHandle& nh);
  void setupROS(ros::NodeHandle& nh);
  void setupController();

  ros::NodeHandle nh_;

  double protocol_version_;
  int dxl_broadcast_id_;
  int default_dxl_id_;
  int power_ctrl_table_;
  int rgb_led_ctrl_table_;
  int torque_on_ctrl_table_;

  int baudrate_;
  std::string offset_file_;
  std::string robot_file_;
  std::string init_file_;
  std::string device_name_;

  KurokoJointController* controller_;
  PortHandler* port_handler_;

  ros::Publisher init_pose_pub_;
  ros::Publisher demo_command_pub_;
  ros::Subscriber button_sub_;
  ros::Subscriber dxl_torque_sub_;
};

#endif  // KUROKO_MODULE_LOADER_H
