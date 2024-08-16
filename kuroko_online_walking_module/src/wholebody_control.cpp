#include <stdio.h>
#include "kuroko_online_walking_module/wholebody_control.h"
#include <utility>

WholebodyControl::WholebodyControl(std::string control_group, double init_time, double fin_time,
                                   geometry_msgs::Pose goal_msg)
{
  control_group_ = std::move(control_group);

  init_time_ = init_time;
  fin_time_ = fin_time;

  goal_msg_ = goal_msg;

  // Initialization
  init_body_pos_.resize(3, 0.0);
  init_body_vel_.resize(3, 0.0);
  init_body_accel_.resize(3, 0.0);
  des_body_pos_.resize(3, 0.0);
  des_body_vel_.resize(3, 0.0);
  des_body_accel_.resize(3, 0.0);
  goal_body_pos_.resize(3, 0.0);
  goal_body_vel_.resize(3, 0.0);
  goal_body_accel_.resize(3, 0.0);

  init_l_foot_pos_.resize(3, 0.0);
  init_l_foot_vel_.resize(3, 0.0);
  init_l_foot_accel_.resize(3, 0.0);
  des_l_foot_pos_.resize(3, 0.0);
  des_l_foot_vel_.resize(3, 0.0);
  des_l_foot_accel_.resize(3, 0.0);
  goal_l_foot_pos_.resize(3, 0.0);
  goal_l_foot_vel_.resize(3, 0.0);
  goal_l_foot_accel_.resize(3, 0.0);

  init_r_foot_pos_.resize(3, 0.0);
  init_r_foot_vel_.resize(3, 0.0);
  init_r_foot_accel_.resize(3, 0.0);
  des_r_foot_pos_.resize(3, 0.0);
  des_r_foot_vel_.resize(3, 0.0);
  des_r_foot_accel_.resize(3, 0.0);
  goal_r_foot_pos_.resize(3, 0.0);
  goal_r_foot_vel_.resize(3, 0.0);
  goal_r_foot_accel_.resize(3, 0.0);

  goal_task_pos_.resize(3, 0.0);
  goal_task_vel_.resize(3, 0.0);
  goal_task_accel_.resize(3, 0.0);

  goal_task_pos_[0] = goal_msg_.position.x;
  goal_task_pos_[1] = goal_msg_.position.y;
  goal_task_pos_[2] = goal_msg_.position.z;

  Eigen::Quaterniond goal_task_q(goal_msg_.orientation.w, goal_msg_.orientation.x, goal_msg_.orientation.y,
                                 goal_msg_.orientation.z);
  goal_task_q_ = goal_task_q;
}

WholebodyControl::~WholebodyControl()
{
}

void WholebodyControl::initialize(const std::vector<double_t>& init_body_pos, std::vector<double_t> init_body_rot,
                                  std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                                  std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q)
{
  init_body_pos_ = init_body_pos;
  des_body_pos_ = init_body_pos;

  Eigen::Quaterniond body_q(init_body_rot[3], init_body_rot[0], init_body_rot[1], init_body_rot[2]);
  init_body_q_ = body_q;
  des_body_q_ = body_q;

  init_r_foot_pos_ = std::move(init_r_foot_pos);
  init_l_foot_pos_ = std::move(init_l_foot_pos);

  des_l_foot_pos_ = init_l_foot_pos_;
  des_r_foot_pos_ = init_r_foot_pos_;

  Eigen::Quaterniond l_foot_q(init_l_foot_Q[3], init_l_foot_Q[0], init_l_foot_Q[1], init_l_foot_Q[2]);
  init_l_foot_q_ = l_foot_q;
  des_l_foot_q_ = l_foot_q;

  Eigen::Quaterniond r_foot_q(init_r_foot_Q[3], init_r_foot_Q[0], init_r_foot_Q[1], init_r_foot_Q[2]);
  init_r_foot_q_ = r_foot_q;
  des_r_foot_q_ = r_foot_q;

  if (control_group_ == "body")
  {
    task_trajectory_ =
        new robotis_framework::MinimumJerk(init_time_, fin_time_, init_body_pos_, init_body_vel_, init_body_accel_,
                                           goal_task_pos_, goal_task_vel_, goal_task_accel_);
    init_task_q_ = body_q;
  }
  else if (control_group_ == "right_leg")
  {
    task_trajectory_ =
        new robotis_framework::MinimumJerk(init_time_, fin_time_, init_r_foot_pos_, init_r_foot_vel_,
                                           init_r_foot_accel_, goal_task_pos_, goal_task_vel_, goal_task_accel_);
    init_task_q_ = r_foot_q;
  }
  else if (control_group_ == "left_leg")
  {
    task_trajectory_ =
        new robotis_framework::MinimumJerk(init_time_, fin_time_, init_l_foot_pos_, init_l_foot_vel_,
                                           init_l_foot_accel_, goal_task_pos_, goal_task_vel_, goal_task_accel_);
    init_task_q_ = l_foot_q;
  }
}

void WholebodyControl::update()
{
}

void WholebodyControl::finalize()
{
  delete task_trajectory_;
}

void WholebodyControl::set(double time)
{
  std::vector<double_t> des_task_pos = task_trajectory_->getPosition(time);

  double count = time / fin_time_;
  des_task_q_ = init_task_q_.slerp(count, goal_task_q_);

  if (control_group_ == "left_leg")
  {
    des_body_pos_ = init_body_pos_;
    des_body_q_ = init_body_q_;

    des_l_foot_pos_ = des_task_pos;
    des_l_foot_q_ = des_task_q_;

    des_r_foot_pos_ = init_r_foot_pos_;
    des_r_foot_q_ = init_r_foot_q_;
  }
  else if (control_group_ == "right_leg")
  {
    des_body_pos_ = init_body_pos_;
    des_body_q_ = init_body_q_;

    des_l_foot_pos_ = init_l_foot_pos_;
    des_l_foot_q_ = init_l_foot_q_;

    des_r_foot_pos_ = des_task_pos;
    des_r_foot_q_ = des_task_q_;
  }
  else if (control_group_ == "body")
  {
    des_body_pos_ = des_task_pos;
    des_body_q_ = des_task_q_;

    des_l_foot_pos_ = init_l_foot_pos_;
    des_l_foot_q_ = init_l_foot_q_;

    des_r_foot_pos_ = init_r_foot_pos_;
    des_r_foot_q_ = init_r_foot_q_;
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
  l_foot_pos = des_l_foot_pos_;
  r_foot_pos = des_r_foot_pos_;
  body_pos = des_body_pos_;
}

std::vector<double_t> WholebodyControl::getTaskVelocity(double /*time*/)
{
  return std::vector<double_t>();
}

std::vector<double_t> WholebodyControl::getTaskAcceleration(double /*time*/)
{
  return std::vector<double_t>();
}

void WholebodyControl::getTaskOrientation(std::vector<double_t>& l_foot_Q, std::vector<double_t>& r_foot_Q,
                                          std::vector<double_t>& body_Q)
{
  l_foot_Q[0] = des_l_foot_q_.x();
  l_foot_Q[1] = des_l_foot_q_.y();
  l_foot_Q[2] = des_l_foot_q_.z();
  l_foot_Q[3] = des_l_foot_q_.w();

  r_foot_Q[0] = des_r_foot_q_.x();
  r_foot_Q[1] = des_r_foot_q_.y();
  r_foot_Q[2] = des_r_foot_q_.z();
  r_foot_Q[3] = des_r_foot_q_.w();

  body_Q[0] = des_body_q_.x();
  body_Q[1] = des_body_q_.y();
  body_Q[2] = des_body_q_.z();
  body_Q[3] = des_body_q_.w();
}
void WholebodyControl::getGroupPose(const std::string& /*name*/, geometry_msgs::Pose* /*msg*/)
{
}
