#ifndef LINK_DATA_H_
#define LINK_DATA_H_

#include <eigen3/Eigen/Eigen>

#include "robotis_math/robotis_math.h"

namespace motion_control
{
class LinkData
{
public:
  LinkData();
  ~LinkData();

  // Tree Params
  std::string name_;
  int parent_;
  int sibling_;
  int child_;

  // Joint params
  Eigen::MatrixXd joint_position_;
  Eigen::MatrixXd joint_axis_;
  double joint_limit_upper_;
  double joint_limit_lower_;

  // Link params
  double link_mass_;
  Eigen::MatrixXd link_center_of_mass_;
  Eigen::MatrixXd link_inertia_;

  // Internal variables
  double internal_joint_angle_;
  double internal_joint_velocity_;
  double internal_joint_acceleration_;
  Eigen::MatrixXd internal_position_;
  Eigen::MatrixXd internal_orientation_;
  Eigen::MatrixXd internal_transformation_;
};

}  // namespace motion_control

#endif /* LINK_DATA_H_ */
