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

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  Eigen::MatrixXd calcMC(int joint_id);
  Eigen::MatrixXd calcCOM(const Eigen::MatrixXd& mc);

  void calcForwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(const Eigen::MatrixXd& tar_position, const Eigen::MatrixXd& curr_position,
                            const Eigen::MatrixXd& tar_orientation, Eigen::MatrixXd curr_orientation);

  bool calcInverseKinematicsForRightLeg(double* out, double x, double y, double z, double roll, double pitch,
                                        double yaw);
  bool calcInverseKinematicsForLeftLeg(double* out, double x, double y, double z, double roll, double pitch, double yaw);

  bool calcInverseKinematicsForRightArm(double* out, double x, double y, double z, double shoulder_pitch,
                                        double gripoper_open_angle);
  bool calcInverseKinematicsForLeftArm(double* out, double x, double y, double z, double shoulder_pitch,
                                       double gripoper_open_angle);

  LinkData* joint_link_tree_[ALL_JOINT_ID + 1];

  LinkData* getLinkData(const std::string& link_name);
  LinkData* getLinkData(const int link_id);
  int getLinkIndex(const std::string& link_name);
  Eigen::MatrixXd getJointAxis(const std::string& link_name);
  double getJointDirection(const std::string& link_name);
  double getJointDirection(const int link_id);

  Eigen::MatrixXd calcPreviewParam(double preview_time, double control_cycle, double lipm_height,
                                   const Eigen::MatrixXd& K, const Eigen::MatrixXd& P);

  double leg_max_height_;
  double leg_side_offset_;
  double gripper_length_;
};

}  // namespace motion_control

#endif /* KUROKO_KINEMATICS_H_ */
