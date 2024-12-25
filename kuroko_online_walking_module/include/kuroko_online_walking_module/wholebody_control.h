#ifndef KUROKO_ONLINE_WALKING_MODULE_WHOLEBODY_CONTROL_
#define KUROKO_ONLINE_WALKING_MODULE_WHOLEBODY_CONTROL_

#pragma once

#include "robotis_math/robotis_math.h"
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <map>
#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>

class WholebodyControl
{
public:
  WholebodyControl(std::string control_group, double init_time, double fin_time, geometry_msgs::Pose goal_msg);
  virtual ~WholebodyControl();

  void initialize(const std::vector<double_t>& init_body_pos, std::vector<double_t> init_body_rot,
                  std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                  std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q);
  void update();
  void finalize();

  void set(double time);

  std::vector<double_t> getJointPosition(double time);
  std::vector<double_t> getJointVelocity(double time);
  std::vector<double_t> getJointAcceleration(double time);

  void getTaskPosition(std::vector<double_t>& l_foot_pos, std::vector<double_t>& r_foot_pos,
                       std::vector<double_t>& body_pos);
  std::vector<double_t> getTaskVelocity(double time);
  std::vector<double_t> getTaskAcceleration(double time);
  void getTaskOrientation(std::vector<double_t>& l_foot_Q, std::vector<double_t>& r_foot_Q,
                          std::vector<double_t>& body_Q);

  void getGroupPose(const std::string& name, geometry_msgs::Pose* msg);

private:
  robotis_framework::MinimumJerk* task_trajectory_;

  std::string control_group_;
  int end_link_;
  double init_time_, fin_time_;
  geometry_msgs::Pose goal_msg_;

  std::vector<double_t> init_body_pos_, init_body_vel_, init_body_accel_;
  std::vector<double_t> des_body_pos_, des_body_vel_, des_body_accel_;
  std::vector<double_t> goal_body_pos_, goal_body_vel_, goal_body_accel_;
  Eigen::Quaterniond init_body_q_, des_body_q_, goal_body_q_;

  std::vector<double_t> init_l_foot_pos_, init_l_foot_vel_, init_l_foot_accel_;
  std::vector<double_t> des_l_foot_pos_, des_l_foot_vel_, des_l_foot_accel_;
  std::vector<double_t> goal_l_foot_pos_, goal_l_foot_vel_, goal_l_foot_accel_;
  Eigen::Quaterniond init_l_foot_q_, des_l_foot_q_, goal_l_foot_q_;

  std::vector<double_t> init_r_foot_pos_, init_r_foot_vel_, init_r_foot_accel_;
  std::vector<double_t> des_r_foot_pos_, des_r_foot_vel_, des_r_foot_accel_;
  std::vector<double_t> goal_r_foot_pos_, goal_r_foot_vel_, goal_r_foot_accel_;
  Eigen::Quaterniond init_r_foot_q_, des_r_foot_q_, goal_r_foot_q_;

  std::vector<double_t> goal_task_pos_, goal_task_vel_, goal_task_accel_;
  Eigen::Quaterniond init_task_q_, des_task_q_, goal_task_q_;
};

#endif
