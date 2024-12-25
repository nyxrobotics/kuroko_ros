#include "kuroko_kinematics/link_data.h"

namespace motion_control
{
LinkData::LinkData()
{
  // Tree Params
  name_ = "";
  parent_ = -1;
  sibling_ = -1;
  child_ = -1;

  // Joint params
  joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
  joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  joint_limit_lower_ = -100.0;
  joint_limit_upper_ = 100.0;
  joint_mimic_ = "";
  joint_mimic_multiplier_ = 1.0;

  // Link params
  link_mass_ = 0.0;
  link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Internal variables
  internal_joint_angle_ = 0.0;
  internal_joint_velocity_ = 0.0;
  internal_joint_acceleration_ = 0.0;
  internal_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
  internal_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
  internal_transformation_ = robotis_framework::getTransformationXYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

LinkData::~LinkData()
{
}

}  // namespace motion_control
