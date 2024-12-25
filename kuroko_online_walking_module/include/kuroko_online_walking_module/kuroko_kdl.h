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

#define LEG_JOINT_NUM (7)  // Adjusted the number of joints for the Kuroko robot
#define D2R (M_PI / 180.0)

class KurokoKinematics
{
public:
  KurokoKinematics();
  virtual ~KurokoKinematics();

  // Initialize the kinematic chains with pelvis position and orientation
  void initialize(const Eigen::MatrixXd& pelvis_position, const Eigen::MatrixXd& pelvis_orientation);

  // Set the joint positions for both legs
  void setJointPosition(Eigen::VectorXd rleg_joint_position, Eigen::VectorXd lleg_joint_position);

  // Solve forward kinematics for both legs and return the positions and orientations
  void solveForwardKinematics(std::vector<double_t>& rleg_position, std::vector<double_t>& rleg_orientation,
                              std::vector<double_t>& lleg_position, std::vector<double_t>& lleg_orientation);

  // Solve inverse kinematics for both legs and return the joint positions
  bool solveInverseKinematics(std::vector<double_t>& rleg_output, const Eigen::MatrixXd& rleg_target_position,
                              Eigen::Quaterniond rleg_target_orientation, std::vector<double_t>& lleg_output,
                              const Eigen::MatrixXd& lleg_target_position, Eigen::Quaterniond lleg_target_orientation);

  // Finalize the kinematic solvers
  void finalize();

protected:
  KDL::Chain rleg_chain_;                            // Right leg kinematic chain
  KDL::ChainDynParam* rleg_dyn_param_ = nullptr;     // Dynamics parameter solver for right leg
  KDL::ChainJntToJacSolver* rleg_jacobian_solver_;   // Jacobian solver for right leg
  KDL::ChainFkSolverPos_recursive* rleg_fk_solver_;  // Forward kinematics solver for right leg
  KDL::ChainIkSolverVel_pinv* rleg_ik_vel_solver_;   // Inverse kinematics velocity solver for right leg
  KDL::ChainIkSolverPos_NR_JL* rleg_ik_pos_solver_;  // Inverse kinematics position solver for right leg

  KDL::Chain lleg_chain_;                            // Left leg kinematic chain
  KDL::ChainDynParam* lleg_dyn_param_ = nullptr;     // Dynamics parameter solver for left leg
  KDL::ChainJntToJacSolver* lleg_jacobian_solver_;   // Jacobian solver for left leg
  KDL::ChainFkSolverPos_recursive* lleg_fk_solver_;  // Forward kinematics solver for left leg
  KDL::ChainIkSolverVel_pinv* lleg_ik_vel_solver_;   // Inverse kinematics velocity solver for left leg
  KDL::ChainIkSolverPos_NR_JL* lleg_ik_pos_solver_;  // Inverse kinematics position solver for left leg

  Eigen::VectorXd rleg_joint_position_, lleg_joint_position_;  // Joint positions for right and left legs
  geometry_msgs::Pose rleg_pose_, lleg_pose_;                  // Pose of right and left legs
};

#endif  // KUROKO_ONLINE_WALKING_MODULE_KUROKO_KDL_
