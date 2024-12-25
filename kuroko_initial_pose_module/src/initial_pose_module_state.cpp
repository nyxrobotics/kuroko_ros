#include "kuroko_initial_pose_module/initial_pose_module_state.h"

namespace motion_control
{
InitialPoseModuleState::InitialPoseModuleState()
{
  is_moving_ = false;

  cnt_ = 0;

  mov_time_ = 1.0;
  smp_time_ = 0.008;
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);

  joint_ini_pose_ = Eigen::MatrixXd::Zero(MAX_JOINT_ID + 1, 1);
  joint_pose_ = Eigen::MatrixXd::Zero(MAX_JOINT_ID + 1, 1);

  via_num_ = 1;

  joint_via_pose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_via_dpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_via_ddpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);

  via_time_ = Eigen::MatrixXd::Zero(via_num_, 1);
}

InitialPoseModuleState::~InitialPoseModuleState()
{
}

}  // namespace motion_control
