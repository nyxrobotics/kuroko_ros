#ifndef KUROKO_ONLINE_WALKING_MODULE_KUROKO_KDL_
#define KUROKO_ONLINE_WALKING_MODULE_KUROKO_KDL_

#pragma once

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/joint.hpp>
#include <map>
#include <math.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <stdint.h>
#include <string>
#include <vector>

#define LEG_JOINT_NUM (6)
#define D2R (M_PI / 180.0)

class KurokoKinematics
{
public:
  KurokoKinematics();
  virtual ~KurokoKinematics();

  //  void initialize(std::vector<double_t> pelvis_position,
  //  std::vector<double_t> pelvis_orientation);
  void initialize(const Eigen::MatrixXd& pelvis_position, const Eigen::MatrixXd& pelvis_orientation);
  void setJointPosition(Eigen::VectorXd rleg_joint_position, Eigen::VectorXd lleg_joint_position);
  void solveForwardKinematics(std::vector<double_t>& rleg_position, std::vector<double_t>& rleg_orientation,
                              std::vector<double_t>& lleg_position, std::vector<double_t>& lleg_orientation);
  bool solveInverseKinematics(std::vector<double_t>& rleg_output, const Eigen::MatrixXd& rleg_target_position,
                              Eigen::Quaterniond rleg_target_orientation, std::vector<double_t>& lleg_output,
                              const Eigen::MatrixXd& lleg_target_position, Eigen::Quaterniond lleg_target_orientation);
  void finalize();

protected:
  KDL::Chain rleg_chain_;
  KDL::ChainDynParam* rleg_dyn_param_ = nullptr;
  KDL::ChainJntToJacSolver* rleg_jacobian_solver_;
  KDL::ChainFkSolverPos_recursive* rleg_fk_solver_;
  KDL::ChainIkSolverVel_pinv* rleg_ik_vel_solver_;
  KDL::ChainIkSolverPos_NR_JL* rleg_ik_pos_solver_;

  KDL::Chain lleg_chain_;
  KDL::ChainDynParam* lleg_dyn_param_ = nullptr;
  KDL::ChainJntToJacSolver* lleg_jacobian_solver_;
  KDL::ChainFkSolverPos_recursive* lleg_fk_solver_;
  KDL::ChainIkSolverVel_pinv* lleg_ik_vel_solver_;
  KDL::ChainIkSolverPos_NR_JL* lleg_ik_pos_solver_;

  KDL::ChainFkSolverPos_recursive* rleg_ft_fk_solver_;
  KDL::ChainFkSolverPos_recursive* lleg_ft_fk_solver_;

  Eigen::VectorXd rleg_joint_position_, lleg_joint_position_;
  geometry_msgs::Pose rleg_pose_, lleg_pose_;
  geometry_msgs::Pose rleg_ft_pose_, lleg_ft_pose_;
};

#endif
