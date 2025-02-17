#ifndef KUROKO_KINEMATICS_H_
#define KUROKO_KINEMATICS_H_

#include <eigen3/Eigen/Eigen>
#include <vector>

#include "kuroko_kinematics_define.h"
#include "link_data.h"

namespace motion_control
{
enum TreeSelect
{
  MANIPULATION,
  WALKING,
  WHOLE_BODY
};

class KurokoKinematics
{
public:
  KurokoKinematics();
  ~KurokoKinematics();
  KurokoKinematics(TreeSelect tree);

  bool solveInverseKinematicsForRightLeg(std::vector<double>& joints_out, std::vector<double> target_pose_in);
  bool solveInverseKinematicsForLeftLeg(std::vector<double>& joints_out, std::vector<double> target_pose_in);

  bool solveForwardKinematicsForRightLeg(const std::vector<double> joints_in, std::vector<double>& target_pose_out);
  bool solveForwardKinematicsForLeftLeg(const std::vector<double> joints_in, std::vector<double>& target_pose_out);

  Eigen::MatrixXd getJointAxis(const std::string& link_name);
  double getJointDirection(const std::string& link_name);
  double getJointDirection(const int link_id);
  double leg_max_height_;
  double leg_side_offset_;
  double gripper_length_;

private:
  LinkData* joint_link_tree_[ALL_JOINT_ID + 1];
  LinkData* getLinkData(const std::string& link_name);
  LinkData* getLinkData(const int link_id);
  int getLinkIndex(const std::string& link_name);
};

}  // namespace motion_control

#endif /* KUROKO_KINEMATICS_H_ */
