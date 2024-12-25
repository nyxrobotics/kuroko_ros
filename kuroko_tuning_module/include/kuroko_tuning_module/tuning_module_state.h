#ifndef TUNE_MODULE_STATE_H_
#define TUNE_MODULE_STATE_H_

#include <eigen3/Eigen/Eigen>

#include "kuroko_kinematics/kuroko_kinematics.h"
#include "robotis_math/robotis_math.h"

namespace motion_control
{
class TuningModuleState
{
public:
  TuningModuleState(int via_num = 1);
  ~TuningModuleState();

  bool is_moving_;
  bool is_generating_;

  int cnt_;  // counter number

  double mov_time_;  // movement time
  double smp_time_;  // sampling time

  int all_time_steps_;  // all time steps of movement time

  Eigen::MatrixXd calc_joint_tra_;  // calculated joint trajectory

  Eigen::MatrixXd joint_ini_pose_;
  Eigen::MatrixXd joint_pose_;

  int via_num_;

  Eigen::MatrixXd joint_via_pose_;
  Eigen::MatrixXd joint_via_dpose_;
  Eigen::MatrixXd joint_via_ddpose_;

  Eigen::MatrixXd via_time_;
};

}  // namespace motion_control

#endif /* INITIAL_POSE_MODULE_STATE_H_ */
