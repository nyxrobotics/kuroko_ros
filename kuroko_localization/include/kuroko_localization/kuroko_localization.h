#ifndef KUROKO_LOCALIZATION_H_
#define KUROKO_LOCALIZATION_H_

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
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

  // subscriber
  ros::Subscriber pelvis_pose_msg_sub_;
  //    ros::Subscriber pelvis_base_walking_msg_sub_;
  ros::Subscriber pelvis_reset_msg_sub_;

  tf::TransformBroadcaster broadcaster_;
  tf::StampedTransform pelvis_trans_;

  geometry_msgs::PoseStamped pelvis_pose_;
  geometry_msgs::PoseStamped pelvis_pose_old_;
  geometry_msgs::PoseStamped pelvis_pose_base_walking_;
  geometry_msgs::PoseStamped pelvis_pose_offset_;

  geometry_msgs::PoseStamped pelvis_pose_base_walking_new_;
  geometry_msgs::PoseStamped pelvis_pose_offset_new_;

  double transform_tolerance_;
  double err_tol_;

  bool is_moving_walking_;

  boost::mutex mutex_;

public:
  void initialize();
  void pelvisPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  //    void pelvisPoseBaseWalkingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pelvisPoseResetCallback(const std_msgs::String::ConstPtr& msg);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  // constructor
  KurokoLocalization();
  // destructor
  ~KurokoLocalization();

  void update();
  void process();
};

}  // namespace kuroko_localization

#endif  // KUROKO_LOCALIZATION_H_
