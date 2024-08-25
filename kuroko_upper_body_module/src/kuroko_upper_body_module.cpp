#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <map>

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "kuroko_upper_body_module/kuroko_upper_body_module.h"

namespace motion_control
{
KurokoUpperBodyModule::KurokoUpperBodyModule() : control_cycle_msec_(8), DEBUG_PRINT(false), present_volt_(0.0)
{
  module_name_ = "kuroko_upper_body_module";
}

KurokoUpperBodyModule::~KurokoUpperBodyModule()
{
  queue_thread_.join();
}

void KurokoUpperBodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&KurokoUpperBodyModule::queueThread, this));

  ros::NodeHandle nh;
  imu_sub_ = nh.subscribe("/imu/data", 10, &KurokoUpperBodyModule::imuCallback, this);
}

void KurokoUpperBodyModule::queueThread()
{
  ros::NodeHandle ros_node;

  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/upper_body_module/status", 5);
  imu_pub_ = ros_node.advertise<sensor_msgs::Imu>("/upper_body_module/imu", 5);
  dxl_power_msg_pub_ =
      ros_node.advertise<robotis_controller_msgs::SyncWriteItem>("/upper_body_module/sync_write_item", 5);

  ros::Rate rate(1000.0 / control_cycle_msec_);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void KurokoUpperBodyModule::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  double chest_angle = current_chest_angles_["chest"];
  double chest_angular_velocity_z =
      msg->angular_velocity.z - (chest_angle - previous_result_["chest"]) / (control_cycle_msec_ * 0.001);

  imu_msg_.angular_velocity.x = msg->angular_velocity.x;
  imu_msg_.angular_velocity.y = msg->angular_velocity.y;
  imu_msg_.angular_velocity.z = chest_angular_velocity_z;

  imu_msg_.linear_acceleration.x = msg->linear_acceleration.x;
  imu_msg_.linear_acceleration.y = msg->linear_acceleration.y;
  imu_msg_.linear_acceleration.z = msg->linear_acceleration.z;

  Eigen::Quaterniond chest_orientation = getCurrentChestOrientation();
  Eigen::Vector3d adjusted_acc =
      chest_orientation.conjugate() *
      Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  imu_msg_.linear_acceleration.x = adjusted_acc.x();
  imu_msg_.linear_acceleration.y = adjusted_acc.y();
  imu_msg_.linear_acceleration.z = adjusted_acc.z();

  imu_pub_.publish(imu_msg_);
}

void KurokoUpperBodyModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                                    std::map<std::string, double> sensors)
{
  // Simulate IMU data insertion into sensors map
  sensors["imu_angular_velocity_x"] = imu_msg_.angular_velocity.x;
  sensors["imu_angular_velocity_y"] = imu_msg_.angular_velocity.y;
  sensors["imu_angular_velocity_z"] = imu_msg_.angular_velocity.z;

  sensors["imu_linear_acceleration_x"] = imu_msg_.linear_acceleration.x;
  sensors["imu_linear_acceleration_y"] = imu_msg_.linear_acceleration.y;
  sensors["imu_linear_acceleration_z"] = imu_msg_.linear_acceleration.z;

  // Sample upper body motion code for shoulders and elbows
  double wave_angle = sin(ros::Time::now().toSec()) * DEGREE2RADIAN * 45.0;

  dxls["shoulder_r_pitch"]->dxl_state_->goal_position_ = wave_angle;
  dxls["shoulder_l_pitch"]->dxl_state_->goal_position_ = -wave_angle;
  dxls["elbow_r_front"]->dxl_state_->goal_position_ = wave_angle / 2;
  dxls["elbow_l_front"]->dxl_state_->goal_position_ = -wave_angle / 2;

  updateUpperBodyMotion();
}

void KurokoUpperBodyModule::updateUpperBodyMotion()
{
  // Add logic for more complex motions if needed
}

double KurokoUpperBodyModule::lowPassFilter(double alpha, double x_new, double& x_old)
{
  double filtered_value = alpha * x_new + (1.0 - alpha) * x_old;
  x_old = filtered_value;
  return filtered_value;
}

Eigen::Quaterniond KurokoUpperBodyModule::getCurrentChestOrientation()
{
  return Eigen::Quaterniond::Identity();
}

double KurokoUpperBodyModule::getCurrentChestAngle()
{
  return 0.0;
}

}  // namespace motion_control
