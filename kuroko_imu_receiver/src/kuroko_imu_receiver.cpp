#include <stdio.h>

#include "kuroko_imu_receiver/kuroko_imu_receiver.h"

namespace motion_control
{
KurokoImuReceiver::KurokoImuReceiver() : control_cycle_msec_(8), robot_name_("kuroko")
{
  module_name_ = "kuroko_imu_receiver";  // set unique module name

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;
  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;
  result_["euler_roll"] = 0.0;
  result_["euler_pitch"] = 0.0;
  result_["euler_yaw"] = 0.0;

  previous_result_["gyro_x"] = 0.0;
  previous_result_["gyro_y"] = 0.0;
  previous_result_["gyro_z"] = 0.0;
  previous_result_["acc_x"] = 0.0;
  previous_result_["acc_y"] = 0.0;
  previous_result_["acc_z"] = 0.0;
  previous_result_["euler_roll"] = 0.0;
  previous_result_["euler_pitch"] = 0.0;
  previous_result_["euler_yaw"] = 0.0;

  last_msg_time_ = ros::Time::now();
}

KurokoImuReceiver::~KurokoImuReceiver()
{
  queue_thread_.join();
}

void KurokoImuReceiver::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  last_msg_time_ = ros::Time::now();
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&KurokoImuReceiver::queueThread, this));
}

void KurokoImuReceiver::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;
  ros_node.setCallbackQueue(&callback_queue);
  imu_sub_ = ros_node.subscribe("/" + robot_name_ + "/sensors/imu/data", 5, &KurokoImuReceiver::imuDataCallback, this);
  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void KurokoImuReceiver::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                                std::map<std::string, robotis_framework::Sensor*> sensors)
{
  // if (sensors.count("kuroko_imu_receiver") == 0)
  // {
  //   ROS_ERROR("[KurokoImuReceiver]: 'kuroko_imu_receiver' not found in the sensor list");
  //   ROS_INFO("Sensor list: ");
  //   for (auto& sensor : sensors)
  //     ROS_INFO("%s", sensor.first.c_str());
  //   return;
  // }

  result_["gyro_x"] = imu_msg_.angular_velocity.x;
  result_["gyro_y"] = imu_msg_.angular_velocity.y;
  result_["gyro_z"] = imu_msg_.angular_velocity.z;
  result_["acc_x"] = imu_msg_.linear_acceleration.x;
  result_["acc_y"] = imu_msg_.linear_acceleration.y;
  result_["acc_z"] = imu_msg_.linear_acceleration.z;

  // Get RPY from quaternion
  Eigen::Quaterniond q(imu_msg_.orientation.w, imu_msg_.orientation.x, imu_msg_.orientation.y, imu_msg_.orientation.z);
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  result_["euler_roll"] = euler[0];
  result_["euler_pitch"] = euler[1];
  result_["euler_yaw"] = euler[2];

  ros::Time update_time = imu_msg_.header.stamp;
  ros::Duration update_duration = ros::Time::now() - update_time;

  previous_result_["gyro_x"] = result_["gyro_x"];
  previous_result_["gyro_y"] = result_["gyro_y"];
  previous_result_["gyro_z"] = result_["gyro_z"];
  previous_result_["acc_x"] = result_["acc_x"];
  previous_result_["acc_y"] = result_["acc_y"];
  previous_result_["acc_z"] = result_["acc_z"];

  previous_result_["euler_roll"] = result_["euler_roll"];
  previous_result_["euler_pitch"] = result_["euler_pitch"];
  previous_result_["euler_yaw"] = result_["euler_yaw"];
}

void KurokoImuReceiver::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_msg_ = *msg;
}

}  // namespace motion_control
