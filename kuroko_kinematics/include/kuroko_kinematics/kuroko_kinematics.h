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

  double getTotalMass(int joint_id);
  Eigen::MatrixXd getMomentOfInertia(int joint_id);
  Eigen::MatrixXd getCenterOfMass(const Eigen::MatrixXd& mc);

  void solveForwardKinematics(int joint_ID);

  Eigen::MatrixXd getJacobian(std::vector<int> idx);
  Eigen::MatrixXd getJacobianCenterOfMass(std::vector<int> idx);
  Eigen::MatrixXd computeStateError(const Eigen::MatrixXd& target_position, const Eigen::MatrixXd& current_position,
                                    const Eigen::MatrixXd& target_orientation, Eigen::MatrixXd current_orientation);

  bool solveInverseKinematicsForRightLeg(double* out, double x, double y, double z, double roll, double pitch,
                                         double yaw);
  bool solveInverseKinematicsForLeftLeg(double* out, double x, double y, double z, double roll, double pitch,
                                        double yaw);

  bool solveInverseKinematicsForRightArm(double* out, double x, double y, double z, double shoulder_pitch,
                                         double gripoper_open_angle);
  bool solveInverseKinematicsForLeftArm(double* out, double x, double y, double z, double shoulder_pitch,
                                        double gripoper_open_angle);

  LinkData* joint_link_tree_[ALL_JOINT_ID + 1];

  LinkData* getLinkData(const std::string& link_name);
  LinkData* getLinkData(const int link_id);
  int getLinkIndex(const std::string& link_name);
  Eigen::MatrixXd getJointAxis(const std::string& link_name);
  double getJointDirection(const std::string& link_name);
  double getJointDirection(const int link_id);

  double leg_max_height_;
  double leg_side_offset_;
  double gripper_length_;
};

}  // namespace motion_control

#endif /* KUROKO_KINEMATICS_H_ */
