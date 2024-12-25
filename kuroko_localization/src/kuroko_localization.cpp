#include "kuroko_localization/kuroko_localization.h"

namespace kuroko_localization
{
KurokoLocalization::KurokoLocalization(ros::NodeHandle& nh) : ros_node_(nh), err_tol_(0.2), is_moving_walking_(false)
{
  initialize();
  ros_node_.param("initial_body_height", initial_body_height_, 0.3);

  pelvis_pose_base_walking_.pose.position.x = 0.0;
  pelvis_pose_base_walking_.pose.position.y = 0.0;
  pelvis_pose_base_walking_.pose.position.z = 0.0;
  pelvis_pose_base_walking_.pose.orientation.x = 0.0;
  pelvis_pose_base_walking_.pose.orientation.y = 0.0;
  pelvis_pose_base_walking_.pose.orientation.z = 0.0;
  pelvis_pose_base_walking_.pose.orientation.w = 1.0;

  pelvis_pose_offset_.pose.position.x = 0.0;
  pelvis_pose_offset_.pose.position.y = 0.0;
  pelvis_pose_offset_.pose.position.z = initial_body_height_;
  pelvis_pose_offset_.pose.orientation.x = 0.0;
  pelvis_pose_offset_.pose.orientation.y = 0.0;
  pelvis_pose_offset_.pose.orientation.z = 0.0;
  pelvis_pose_offset_.pose.orientation.w = 1.0;

  pelvis_pose_old_.pose.position.x = 0.0;
  pelvis_pose_old_.pose.position.y = 0.0;
  pelvis_pose_old_.pose.position.z = 0.0;
  pelvis_pose_old_.pose.orientation.x = 0.0;
  pelvis_pose_old_.pose.orientation.y = 0.0;
  pelvis_pose_old_.pose.orientation.z = 0.0;
  pelvis_pose_old_.pose.orientation.w = 1.0;

  update();
}

KurokoLocalization::~KurokoLocalization()
{
}

void KurokoLocalization::initialize()
{
  pelvis_pose_msg_sub_ =
      ros_node_.subscribe("/motion_control/pelvis_pose", 5, &KurokoLocalization::pelvisPoseCallback, this);
  pelvis_reset_msg_sub_ =
      ros_node_.subscribe("/motion_control/pelvis_pose_reset", 5, &KurokoLocalization::pelvisPoseResetCallback, this);
}

void KurokoLocalization::pelvisPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  mutex_.lock();

  pelvis_pose_offset_ = *msg;
  pelvis_pose_.header.stamp = pelvis_pose_offset_.header.stamp;

  mutex_.unlock();
}

void KurokoLocalization::pelvisPoseResetCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "reset")
  {
    ROS_INFO("Pelvis Pose Reset");

    pelvis_pose_old_.pose.position.x = 0.0;
    pelvis_pose_old_.pose.position.y = 0.0;
    pelvis_pose_old_.pose.orientation.x = 0.0;
    pelvis_pose_old_.pose.orientation.y = 0.0;
    pelvis_pose_old_.pose.orientation.z = 0.0;
    pelvis_pose_old_.pose.orientation.w = 1.0;
  }
}

void KurokoLocalization::process()
{
  update();

  pelvis_trans_.setOrigin(
      tf::Vector3(pelvis_pose_.pose.position.x, pelvis_pose_.pose.position.y, pelvis_pose_.pose.position.z));

  tf::Quaternion q(pelvis_pose_.pose.orientation.x, pelvis_pose_.pose.orientation.y, pelvis_pose_.pose.orientation.z,
                   pelvis_pose_.pose.orientation.w);

  pelvis_trans_.setRotation(q);
  tf::StampedTransform tmp_tf_stamped(pelvis_trans_, ros::Time::now(), "world", "body_link");

  broadcaster_.sendTransform(tmp_tf_stamped);
}

void KurokoLocalization::update()
{
  mutex_.lock();

  Eigen::Quaterniond pose_old_quaternion(pelvis_pose_old_.pose.orientation.w, pelvis_pose_old_.pose.orientation.x,
                                         pelvis_pose_old_.pose.orientation.y, pelvis_pose_old_.pose.orientation.z);

  Eigen::Quaterniond pose_offset_quaternion(pelvis_pose_offset_.pose.orientation.w,
                                            pelvis_pose_offset_.pose.orientation.x,
                                            pelvis_pose_offset_.pose.orientation.y,
                                            pelvis_pose_offset_.pose.orientation.z);

  Eigen::Quaterniond pose_quaternion = pose_old_quaternion * pose_offset_quaternion;

  Eigen::MatrixXd position_offset = Eigen::MatrixXd::Zero(3, 1);
  position_offset.coeffRef(0, 0) = pelvis_pose_offset_.pose.position.x;
  position_offset.coeffRef(1, 0) = pelvis_pose_offset_.pose.position.y;

  Eigen::MatrixXd orientation = robotis_framework::convertQuaternionToRotation(pose_old_quaternion);
  Eigen::MatrixXd position_offset_new = orientation * position_offset;

  pelvis_pose_offset_new_.pose.position.x = position_offset_new.coeff(0, 0);
  pelvis_pose_offset_new_.pose.position.y = position_offset_new.coeff(1, 0);

  pelvis_pose_.pose.position.x = pelvis_pose_old_.pose.position.x + pelvis_pose_offset_new_.pose.position.x;
  pelvis_pose_.pose.position.y = pelvis_pose_old_.pose.position.y + pelvis_pose_offset_new_.pose.position.y;
  pelvis_pose_.pose.position.z = pelvis_pose_offset_.pose.position.z;

  tf::quaternionEigenToMsg(pose_quaternion, pelvis_pose_.pose.orientation);

  mutex_.unlock();
}

}  // namespace kuroko_localization
