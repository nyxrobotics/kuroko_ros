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

  std::string name_;

  int parent_;
  int sibling_;
  int child_;

  double mass_;

  Eigen::MatrixXd relative_position_;
  Eigen::MatrixXd joint_axis_;
  Eigen::MatrixXd center_of_mass_;
  Eigen::MatrixXd inertia_;

  double joint_limit_max_;
  double joint_limit_min_;

  double joint_angle_;
  double joint_velocity_;
  double joint_acceleration_;

  Eigen::MatrixXd position_;
  Eigen::MatrixXd orientation_;
  Eigen::MatrixXd transformation_;
};

}  // namespace motion_control

#endif /* LINK_DATA_H_ */
