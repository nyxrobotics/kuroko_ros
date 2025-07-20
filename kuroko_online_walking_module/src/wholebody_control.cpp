#include <stdio.h>
#include "kuroko_online_walking_module/wholebody_control.h"
#include <utility>
#include "robotis_math/robotis_linear_algebra.h"

WholebodyControl::WholebodyControl(std::string control_group, double init_time, double fin_time,
                                   geometry_msgs::Pose goal_msg)
{
  control_group_ = control_group;

  init_time_ = init_time;
  fin_time_ = fin_time;

  goal_msg_ = goal_msg;

  // Initialization
  init_body_position_.resize(3, 0.0);
  init_body_velocity_.resize(3, 0.0);
  init_body_accel_.resize(3, 0.0);
  des_body_position_.resize(3, 0.0);
  des_body_velocity_.resize(3, 0.0);
  des_body_accel_.resize(3, 0.0);
  goal_body_position_.resize(3, 0.0);
  goal_body_velocity_.resize(3, 0.0);
  goal_body_accel_.resize(3, 0.0);

  init_l_foot_position_.resize(3, 0.0);
  init_l_foot_velocity_.resize(3, 0.0);
  init_l_foot_accel_.resize(3, 0.0);
  des_l_foot_position_.resize(3, 0.0);
  des_l_foot_velocity_.resize(3, 0.0);
  des_l_foot_accel_.resize(3, 0.0);
  goal_l_foot_position_.resize(3, 0.0);
  goal_l_foot_velocity_.resize(3, 0.0);
  goal_l_foot_accel_.resize(3, 0.0);

  init_r_foot_position_.resize(3, 0.0);
  init_r_foot_velocity_.resize(3, 0.0);
  init_r_foot_accel_.resize(3, 0.0);
  des_r_foot_position_.resize(3, 0.0);
  des_r_foot_velocity_.resize(3, 0.0);
  des_r_foot_accel_.resize(3, 0.0);
  goal_r_foot_position_.resize(3, 0.0);
  goal_r_foot_velocity_.resize(3, 0.0);
  goal_r_foot_accel_.resize(3, 0.0);

  goal_task_position_.resize(3, 0.0);
  goal_task_velocity_.resize(3, 0.0);
  goal_task_accel_.resize(3, 0.0);

  goal_task_position_[0] = goal_msg_.position.x;
  goal_task_position_[1] = goal_msg_.position.y;
  goal_task_position_[2] = goal_msg_.position.z;

  Eigen::Quaterniond goal_task_quaternion(goal_msg_.orientation.w, goal_msg_.orientation.x, goal_msg_.orientation.y,
                                          goal_msg_.orientation.z);
  goal_task_quaternion_ = goal_task_quaternion;
}

WholebodyControl::~WholebodyControl()
{
}

void WholebodyControl::initialize(const std::vector<double_t>& init_body_pos, std::vector<double_t> init_body_rpy,
                                  std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_rpy,
                                  std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_rpy)
{
  init_body_position_ = init_body_pos;
  des_body_position_ = init_body_pos;

  Eigen::Quaterniond body_quaternion =
      robotis_framework::convertRPYToQuaternion(init_body_rpy[0], init_body_rpy[1], init_body_rpy[2]);
  init_body_quaternion_ = body_quaternion;
  des_body_quaternion_ = body_quaternion;

  init_r_foot_position_ = init_r_foot_pos;
  init_l_foot_position_ = init_l_foot_pos;

  des_l_foot_position_ = init_l_foot_position_;
  des_r_foot_position_ = init_r_foot_position_;

  Eigen::Quaterniond l_foot_quaternion =
      robotis_framework::convertRPYToQuaternion(init_l_foot_rpy[0], init_l_foot_rpy[1], init_l_foot_rpy[2]);
  init_l_foot_quaternion_ = l_foot_quaternion;
  des_l_foot_quaternion_ = l_foot_quaternion;

  Eigen::Quaterniond r_foot_quaternion =
      robotis_framework::convertRPYToQuaternion(init_r_foot_rpy[0], init_r_foot_rpy[1], init_r_foot_rpy[2]);
  init_r_foot_quaternion_ = r_foot_quaternion;
  des_r_foot_quaternion_ = r_foot_quaternion;

  if (control_group_ == "body")
  {
    task_trajectory_ = new robotis_framework::MinimumJerk(init_time_, fin_time_, init_body_position_,
                                                          init_body_velocity_, init_body_accel_, goal_task_position_,
                                                          goal_task_velocity_, goal_task_accel_);
    init_task_quaternion_ = body_quaternion;
  }
  else if (control_group_ == "right_leg")
  {
    task_trajectory_ = new robotis_framework::MinimumJerk(init_time_, fin_time_, init_r_foot_position_,
                                                          init_r_foot_velocity_, init_r_foot_accel_,
                                                          goal_task_position_, goal_task_velocity_, goal_task_accel_);
    init_task_quaternion_ = r_foot_quaternion;
  }
  else if (control_group_ == "left_leg")
  {
    task_trajectory_ = new robotis_framework::MinimumJerk(init_time_, fin_time_, init_l_foot_position_,
                                                          init_l_foot_velocity_, init_l_foot_accel_,
                                                          goal_task_position_, goal_task_velocity_, goal_task_accel_);
    init_task_quaternion_ = l_foot_quaternion;
  }
}

void WholebodyControl::update()
{
}

void WholebodyControl::finalize()
{
  if (task_trajectory_ != nullptr)
  {
    delete task_trajectory_;
    task_trajectory_ = nullptr;
  }
}

void WholebodyControl::set(double time)
{
  if (task_trajectory_ == nullptr)
    return;
  std::vector<double_t> des_task_pos = task_trajectory_->getPosition(time);

  double count = std::min(std::max((time - init_time_) / fin_time_, 0.0), 1.0);
  des_task_quaternion_ = init_task_quaternion_.slerp(count, goal_task_quaternion_);

  if (control_group_ == "left_leg")
  {
    des_body_position_ = init_body_position_;
    des_body_quaternion_ = init_body_quaternion_;

    des_l_foot_position_ = des_task_pos;
    des_l_foot_quaternion_ = des_task_quaternion_;

    des_r_foot_position_ = init_r_foot_position_;
    des_r_foot_quaternion_ = init_r_foot_quaternion_;
  }
  else if (control_group_ == "right_leg")
  {
    des_body_position_ = init_body_position_;
    des_body_quaternion_ = init_body_quaternion_;

    des_l_foot_position_ = init_l_foot_position_;
    des_l_foot_quaternion_ = init_l_foot_quaternion_;

    des_r_foot_position_ = des_task_pos;
    des_r_foot_quaternion_ = des_task_quaternion_;
  }
  else if (control_group_ == "body")
  {
    des_body_position_ = des_task_pos;
    des_body_quaternion_ = des_task_quaternion_;

    des_l_foot_position_ = init_l_foot_position_;
    des_l_foot_quaternion_ = init_l_foot_quaternion_;

    des_r_foot_position_ = init_r_foot_position_;
    des_r_foot_quaternion_ = init_r_foot_quaternion_;
  }
}

std::vector<double_t> WholebodyControl::getJointPosition(double /*time*/)
{
  return std::vector<double_t>();
}

std::vector<double_t> WholebodyControl::getJointVelocity(double /*time*/)
{
  return std::vector<double_t>();
}

std::vector<double_t> WholebodyControl::getJointAcceleration(double /*time*/)
{
  return std::vector<double_t>();
}

void WholebodyControl::getTaskPosition(std::vector<double_t>& l_foot_pos, std::vector<double_t>& r_foot_pos,
                                       std::vector<double_t>& body_pos)
{
  l_foot_pos = des_l_foot_position_;
  r_foot_pos = des_r_foot_position_;
  body_pos = des_body_position_;
}

std::vector<double_t> WholebodyControl::getTaskVelocity(double /*time*/)
{
  return std::vector<double_t>();
}

std::vector<double_t> WholebodyControl::getTaskAcceleration(double /*time*/)
{
  return std::vector<double_t>();
}

void WholebodyControl::getTaskOrientation(std::vector<double_t>& l_foot_rpy, std::vector<double_t>& r_foot_rpy,
                                          std::vector<double_t>& body_rpy)
{
  l_foot_rpy.resize(3);
  r_foot_rpy.resize(3);
  body_rpy.resize(3);
  Eigen::Vector3d l_foot_rpy_euler = robotis_framework::convertQuaternionToRPY(des_l_foot_quaternion_);
  l_foot_rpy[0] = l_foot_rpy_euler.x();
  l_foot_rpy[1] = l_foot_rpy_euler.y();
  l_foot_rpy[2] = l_foot_rpy_euler.z();

  Eigen::Vector3d r_foot_rpy_euler = robotis_framework::convertQuaternionToRPY(des_r_foot_quaternion_);
  r_foot_rpy[0] = r_foot_rpy_euler.x();
  r_foot_rpy[1] = r_foot_rpy_euler.y();
  r_foot_rpy[2] = r_foot_rpy_euler.z();

  Eigen::Vector3d body_rpy_euler = robotis_framework::convertQuaternionToRPY(des_body_quaternion_);
  body_rpy[0] = body_rpy_euler.x();
  body_rpy[1] = body_rpy_euler.y();
  body_rpy[2] = body_rpy_euler.z();
}
void WholebodyControl::getGroupPose(const std::string& /*name*/, geometry_msgs::Pose* /*msg*/)
{
}
