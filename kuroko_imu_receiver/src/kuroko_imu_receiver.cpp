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
  Eigen::Vector3d euler = imuQuaternionToRollPitchYaw(q);
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

Eigen::Vector3d KurokoImuReceiver::imuQuaternionToRollPitchYaw(const Eigen::Quaterniond& q)
{
  // ジンバルロックを防ぐため、ベクトルの相対的な変位を使用して角度を求める
  // 解はXYZオイラー角と異なる
  Eigen::Vector3d x_origin = Eigen::Vector3d::UnitX();  // Front
  Eigen::Vector3d y_origin = Eigen::Vector3d::UnitY();  // Left
  Eigen::Vector3d z_origin = Eigen::Vector3d::UnitZ();  // Up

  Eigen::Vector3d x_robot = q * x_origin;
  Eigen::Vector3d y_robot = q * y_origin;
  Eigen::Vector3d z_robot = q * z_origin;

  double roll = 0, pitch = 0, yaw = 0;
  // pitch: 原点のxy平面から見てロボットのx軸がどれだけ傾いているか
  pitch = std::atan2(-x_robot.z(), std::sqrt(x_robot.x() * x_robot.x() + x_robot.y() * x_robot.y()));
  if (z_robot.z() < 0.0)
    pitch = M_PI - pitch;  // 逆立ち補正
  // roll: 原点のxy平面から見てロボットのy軸がどれだけ傾いているか
  roll = std::atan2(y_robot.z(), std::sqrt(y_robot.x() * y_robot.x() + y_robot.y() * y_robot.y()));
  // yaw: 原点のz軸周りにロボットがどれだけ回転しているか
  if (fabs(x_robot.z()) < fabs(y_robot.z()))
  {
    // x軸の傾きがy軸の傾きより小さい場合、x軸を使用してyawを計算
    yaw = std::atan2(x_robot.y(), x_robot.x());
    if (z_robot.z() < 0.0)
      yaw = yaw + M_PI;  // 逆立ち補正
  }
  else
  {
    // y軸の傾きがx軸の傾きより小さい場合、y軸を使用してyawを計算
    yaw = std::atan2(y_robot.y(), y_robot.x()) - M_PI / 2.0;
  }

  // 最後に-M_PIから+M_PIに変換する
  roll = wrapToPi(roll);
  pitch = wrapToPi(pitch);
  yaw = wrapToPi(yaw);
  return Eigen::Vector3d(roll, pitch, yaw);
}

double KurokoImuReceiver::wrapToPi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
    angle += 2.0 * M_PI;
  return angle - M_PI;
}

}  // namespace motion_control
