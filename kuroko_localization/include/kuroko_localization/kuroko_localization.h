#ifndef KUROKO_LOCALIZATION_H_
#define KUROKO_LOCALIZATION_H_

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Eigen>
#include <boost/thread.hpp>
#include "robotis_math/robotis_math.h"

namespace kuroko_localization
{
class KurokoLocalization
{
private:
  // ros node handle
  ros::NodeHandle ros_node_;

  // ROS parameters
  double initial_body_height_;
  bool publish_tf_;
  bool publish_odom_;
  std::string world_frame_id_;
  std::string robot_frame_id_;

  // Publisher
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // subscriber
  ros::Subscriber pelvis_pose_msg_sub_;
  ros::Subscriber pelvis_reset_msg_sub_;

  tf::StampedTransform pelvis_trans_;
  geometry_msgs::PoseStamped pelvis_pose_;
  geometry_msgs::PoseStamped pelvis_pose_old_;
  geometry_msgs::PoseStamped pelvis_pose_base_walking_;
  geometry_msgs::PoseStamped pelvis_pose_offset_;

  geometry_msgs::PoseStamped pelvis_pose_base_walking_new_;
  geometry_msgs::PoseStamped pelvis_pose_offset_new_;

  double err_tol_;
  bool is_moving_walking_;
  boost::mutex mutex_;

public:
  KurokoLocalization(ros::NodeHandle& nh);
  ~KurokoLocalization();

  void initialize();
  void pelvisPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pelvisPoseResetCallback(const std_msgs::String::ConstPtr& msg);
  void update();
  void process();
};

}  // namespace kuroko_localization

#endif  // KUROKO_LOCALIZATION_H_
