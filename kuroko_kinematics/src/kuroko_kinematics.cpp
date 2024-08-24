#include "kuroko_kinematics/kuroko_kinematics.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "robotis_math/robotis_linear_algebra.h"

namespace motion_control
{
KurokoKinematics::KurokoKinematics()
{
}
KurokoKinematics::~KurokoKinematics()
{
}

KurokoKinematics::KurokoKinematics(TreeSelect tree)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    joint_link_tree_[id] = new LinkData();

  if (tree == WHOLE_BODY)
  {
    // Base link
    joint_link_tree_[0]->name_ = "base";
    joint_link_tree_[0]->parent_ = -1;
    joint_link_tree_[0]->sibling_ = -1;
    joint_link_tree_[0]->child_ = 1;
    joint_link_tree_[0]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[0]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[0]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[0]->joint_limit_lower_ = -100.0;
    joint_link_tree_[0]->joint_limit_upper_ = 100.0;
    joint_link_tree_[0]->link_mass_ = 0.0;
    joint_link_tree_[0]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[0]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Waist link
    joint_link_tree_[1]->name_ = "waist";
    joint_link_tree_[1]->parent_ = 0;
    joint_link_tree_[1]->sibling_ = -1;
    joint_link_tree_[1]->child_ = 2;
    joint_link_tree_[1]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[1]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[1]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[1]->joint_limit_lower_ = -100.0;
    joint_link_tree_[1]->joint_limit_upper_ = 100.0;
    joint_link_tree_[1]->link_mass_ = 0.337;
    joint_link_tree_[1]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.00125, 0.0, -0.04875);
    joint_link_tree_[1]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Chest link
    joint_link_tree_[2]->name_ = "chest";
    joint_link_tree_[2]->parent_ = 1;
    joint_link_tree_[2]->sibling_ = 11;  // hip_r_roll
    joint_link_tree_[2]->child_ = 3;
    joint_link_tree_[2]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[2]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[2]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    joint_link_tree_[2]->joint_limit_lower_ = -3.1;
    joint_link_tree_[2]->joint_limit_upper_ = 3.1;
    joint_link_tree_[2]->link_mass_ = 0.337;
    joint_link_tree_[2]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.057);
    joint_link_tree_[2]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right Arm
    joint_link_tree_[3]->name_ = "shoulder_r_pitch";
    joint_link_tree_[3]->parent_ = 2;
    joint_link_tree_[3]->sibling_ = 7;  // (shoulder_l_pitch)
    joint_link_tree_[3]->child_ = 4;
    joint_link_tree_[3]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, -0.1275, 0.01275);
    joint_link_tree_[3]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[3]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[3]->joint_limit_lower_ = -3.1;
    joint_link_tree_[3]->joint_limit_upper_ = 3.1;
    joint_link_tree_[3]->link_mass_ = 0.022;
    joint_link_tree_[3]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, -0.024, 0.0);
    joint_link_tree_[3]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[4]->name_ = "shoulder_r_roll";
    joint_link_tree_[4]->parent_ = 3;
    joint_link_tree_[4]->sibling_ = -1;
    joint_link_tree_[4]->child_ = 5;
    joint_link_tree_[4]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, -0.1275, 0.01275);
    joint_link_tree_[4]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[4]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    joint_link_tree_[4]->joint_limit_lower_ = -1.0472;
    joint_link_tree_[4]->joint_limit_upper_ = 2.0071;
    joint_link_tree_[4]->link_mass_ = 0.342;
    joint_link_tree_[4]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, -0.016, -0.02575);
    joint_link_tree_[4]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[5]->name_ = "elbow_r_front_and_rear";
    joint_link_tree_[5]->parent_ = 4;
    joint_link_tree_[5]->sibling_ = -1;
    joint_link_tree_[5]->child_ = 6;
    joint_link_tree_[5]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.008, -0.052);
    joint_link_tree_[5]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[5]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[5]->joint_limit_lower_ = -2.6180;
    joint_link_tree_[5]->joint_limit_upper_ = 2.6180;
    joint_link_tree_[5]->link_mass_ = 0.063;
    joint_link_tree_[5]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0043, 0.0, -0.1);
    joint_link_tree_[5]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[6]->name_ = "arm_r_end";
    joint_link_tree_[6]->parent_ = 5;
    joint_link_tree_[6]->sibling_ = -1;
    joint_link_tree_[6]->child_ = -1;
    joint_link_tree_[6]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.208);
    joint_link_tree_[6]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[6]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[6]->joint_limit_lower_ = -100;
    joint_link_tree_[6]->joint_limit_upper_ = 100;
    joint_link_tree_[6]->link_mass_ = 0;
    joint_link_tree_[6]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_tree_[6]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Left Arm
    joint_link_tree_[7]->name_ = "shoulder_l_pitch";
    joint_link_tree_[7]->parent_ = 2;
    joint_link_tree_[7]->sibling_ = -1;
    joint_link_tree_[7]->child_ = 8;
    joint_link_tree_[7]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.1275, 0.01275);
    joint_link_tree_[7]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[7]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[7]->joint_limit_lower_ = -3.1;
    joint_link_tree_[7]->joint_limit_upper_ = 3.1;
    joint_link_tree_[7]->link_mass_ = 0.022;
    joint_link_tree_[7]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.024, 0.0);
    joint_link_tree_[7]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[8]->name_ = "shoulder_l_roll";
    joint_link_tree_[8]->parent_ = 7;
    joint_link_tree_[8]->sibling_ = -1;
    joint_link_tree_[8]->child_ = 9;
    joint_link_tree_[8]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.038, 0.0);
    joint_link_tree_[8]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[8]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    joint_link_tree_[8]->joint_limit_lower_ = -2.0071;
    joint_link_tree_[8]->joint_limit_upper_ = 1.0472;
    joint_link_tree_[8]->link_mass_ = 0.342;
    joint_link_tree_[8]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.016, -0.02575);
    joint_link_tree_[8]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[9]->name_ = "elbow_l_front_and_rear";
    joint_link_tree_[9]->parent_ = 8;
    joint_link_tree_[9]->sibling_ = -1;
    joint_link_tree_[9]->child_ = 10;
    joint_link_tree_[9]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, -0.008, -0.052);
    joint_link_tree_[9]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[9]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[9]->joint_limit_lower_ = -2.6180;
    joint_link_tree_[9]->joint_limit_upper_ = 2.6180;
    joint_link_tree_[9]->link_mass_ = 0.063;
    joint_link_tree_[9]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0043, 0.0, -0.104);
    joint_link_tree_[9]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[10]->name_ = "arm_l_end";
    joint_link_tree_[10]->parent_ = 9;
    joint_link_tree_[10]->sibling_ = -1;
    joint_link_tree_[10]->child_ = -1;
    joint_link_tree_[10]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.208);
    joint_link_tree_[10]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[10]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[10]->joint_limit_lower_ = -100;
    joint_link_tree_[10]->joint_limit_upper_ = 100;
    joint_link_tree_[10]->link_mass_ = 0;
    joint_link_tree_[10]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_tree_[10]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right Leg
    joint_link_tree_[11]->name_ = "hip_r_roll";
    joint_link_tree_[11]->parent_ = 1;    // waist
    joint_link_tree_[11]->sibling_ = 25;  // hip_l_roll
    joint_link_tree_[11]->child_ = 12;    // hip_r_pitch
    joint_link_tree_[11]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.08425);
    joint_link_tree_[11]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[11]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    joint_link_tree_[11]->joint_limit_lower_ = -0.5550;
    joint_link_tree_[11]->joint_limit_upper_ = 1.8640;
    joint_link_tree_[11]->link_mass_ = 0.026;
    joint_link_tree_[11]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.00125, -0.0065, -0.004125);
    joint_link_tree_[11]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right hip pitch
    joint_link_tree_[12]->name_ = "hip_r_pitch";
    joint_link_tree_[12]->parent_ = 11;  // hip_r_roll
    joint_link_tree_[12]->sibling_ = -1;
    joint_link_tree_[12]->child_ = 13;  // thigh_r_front_active
    joint_link_tree_[12]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, -0.027, 0.0);
    joint_link_tree_[12]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[12]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[12]->joint_limit_lower_ = -3.1;
    joint_link_tree_[12]->joint_limit_upper_ = 3.1;
    joint_link_tree_[12]->link_mass_ = 0.478;
    joint_link_tree_[12]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.00175, -0.049, -0.015);
    joint_link_tree_[12]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right thigh front active
    joint_link_tree_[13]->name_ = "thigh_r_front_active";
    joint_link_tree_[13]->parent_ = 12;   // hip_r_pitch
    joint_link_tree_[13]->sibling_ = 20;  // shin_r_active
    joint_link_tree_[13]->child_ = 14;    // knee_r_passive
    joint_link_tree_[13]->joint_position_ = robotis_framework::getTransitionXYZ(0.015, -0.02925, -0.0305);
    joint_link_tree_[13]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, -0.785398163, 0.0);
    joint_link_tree_[13]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[13]->joint_limit_lower_ = -0.4974;
    joint_link_tree_[13]->joint_limit_upper_ = 1.8588;
    joint_link_tree_[13]->link_mass_ = 0.012;
    joint_link_tree_[13]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.05);
    joint_link_tree_[13]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right knee passive
    joint_link_tree_[14]->name_ = "knee_r_passive";
    joint_link_tree_[14]->parent_ = 13;  // thigh_r_front_active
    joint_link_tree_[14]->sibling_ = -1;
    joint_link_tree_[14]->child_ = 15;  // shin_r_front_passive
    joint_link_tree_[14]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.1);
    joint_link_tree_[14]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.785398163, 0.0);
    joint_link_tree_[14]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[14]->joint_limit_lower_ = -1.8588;
    joint_link_tree_[14]->joint_limit_upper_ = 0.4974;
    joint_link_tree_[14]->joint_mimic_ = "thigh_r_front_active";
    joint_link_tree_[14]->joint_mimic_multiplier_ = -1.0;
    joint_link_tree_[14]->link_mass_ = 0.007;
    joint_link_tree_[14]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.01675, 0.0, 0.0);
    joint_link_tree_[14]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right shin front passive
    joint_link_tree_[15]->name_ = "shin_r_front_passive";
    joint_link_tree_[15]->parent_ = 14;   // knee_r_passive
    joint_link_tree_[15]->sibling_ = 24;  // shin_r_rear_passive
    joint_link_tree_[15]->child_ = 16;    // ankle_r_roll
    joint_link_tree_[15]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[15]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.785398163, 0.0);
    joint_link_tree_[15]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[15]->joint_limit_lower_ = -1.8588;
    joint_link_tree_[15]->joint_limit_upper_ = 0.4974;
    joint_link_tree_[15]->joint_mimic_ = "shin_r_active";
    joint_link_tree_[15]->joint_mimic_multiplier_ = 1.0;
    joint_link_tree_[15]->link_mass_ = 0.013;
    joint_link_tree_[15]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.05);
    joint_link_tree_[15]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right ankle pitch passive
    joint_link_tree_[16]->name_ = "ankle_r_pitch_passive";
    joint_link_tree_[16]->parent_ = 15;  // shin_r_front_passive
    joint_link_tree_[16]->sibling_ = -1;
    joint_link_tree_[16]->child_ = 17;  // ankle_r_roll
    joint_link_tree_[16]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.1);
    joint_link_tree_[16]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, -0.785398163, 0.0);
    joint_link_tree_[16]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[16]->joint_limit_lower_ = -0.4974;
    joint_link_tree_[16]->joint_limit_upper_ = 1.8588;
    joint_link_tree_[16]->joint_mimic_ = "shin_r_active";
    joint_link_tree_[16]->joint_mimic_multiplier_ = -1.0;
    joint_link_tree_[16]->link_mass_ = 0.104;
    joint_link_tree_[16]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.0285, 0, -0.013);
    joint_link_tree_[16]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right ankle roll
    joint_link_tree_[17]->name_ = "ankle_r_roll";
    joint_link_tree_[17]->parent_ = 16;  // ankle_r_pitch_passive
    joint_link_tree_[17]->sibling_ = -1;
    joint_link_tree_[17]->child_ = 18;  // ankle_r_yaw
    joint_link_tree_[17]->joint_position_ = robotis_framework::getTransitionXYZ(-0.015, 0, -0.027);
    joint_link_tree_[17]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[17]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    joint_link_tree_[17]->joint_limit_lower_ = -0.3704;
    joint_link_tree_[17]->joint_limit_upper_ = 1.5708;
    joint_link_tree_[17]->link_mass_ = 0.0136;
    joint_link_tree_[17]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, -0.009375, -0.01175);
    joint_link_tree_[17]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right ankle yaw
    joint_link_tree_[18]->name_ = "ankle_r_yaw";
    joint_link_tree_[18]->parent_ = 17;  // ankle_r_roll
    joint_link_tree_[18]->sibling_ = -1;
    joint_link_tree_[18]->child_ = 19;  // leg_r_end
    joint_link_tree_[18]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.03825);
    joint_link_tree_[18]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[18]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    joint_link_tree_[18]->joint_limit_lower_ = -3.1;
    joint_link_tree_[18]->joint_limit_upper_ = 3.1;
    joint_link_tree_[18]->link_mass_ = 0.043;
    joint_link_tree_[18]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.004);
    joint_link_tree_[18]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right leg end
    joint_link_tree_[19]->name_ = "leg_r_end";
    joint_link_tree_[19]->parent_ = 18;  // ankle_r_yaw
    joint_link_tree_[19]->sibling_ = -1;
    joint_link_tree_[19]->child_ = -1;
    joint_link_tree_[19]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.004);
    joint_link_tree_[19]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[19]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[19]->joint_limit_lower_ = -100;
    joint_link_tree_[19]->joint_limit_upper_ = 100;
    joint_link_tree_[19]->link_mass_ = 0;
    joint_link_tree_[19]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_tree_[19]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[20]->name_ = "shin_r_active";
    joint_link_tree_[20]->parent_ = 12;   // hip_r_pitch
    joint_link_tree_[20]->sibling_ = 23;  // thigh_r_middle_passive_link
    joint_link_tree_[20]->child_ = 21;    // thigh_r_rear_passive_mimic
    joint_link_tree_[20]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0185, -0.02925, -0.0305);
    joint_link_tree_[20]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 1.047197551, 0);
    joint_link_tree_[20]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[20]->joint_limit_lower_ = -1.8588;
    joint_link_tree_[20]->joint_limit_upper_ = 0.4974;
    joint_link_tree_[20]->link_mass_ = 0.008;
    joint_link_tree_[20]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.004745, 0, -0.015);
    joint_link_tree_[20]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[21]->name_ = "thigh_r_rear_passive_mimic";
    joint_link_tree_[21]->parent_ = 20;  // shin_r_active
    joint_link_tree_[21]->sibling_ = -1;
    joint_link_tree_[21]->child_ = 22;  // thigh_r_rear_passive
    joint_link_tree_[21]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.03);
    joint_link_tree_[21]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, -1.832597551, 0);
    joint_link_tree_[21]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[21]->joint_limit_lower_ = -3.1;
    joint_link_tree_[21]->joint_limit_upper_ = 3.1;
    joint_link_tree_[21]->joint_mimic_ = "shin_r_active";
    joint_link_tree_[21]->joint_mimic_multiplier_ = -1.0;
    joint_link_tree_[21]->link_mass_ = 0.0;
    joint_link_tree_[21]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_tree_[21]->link_inertia_ = robotis_framework::getInertiaXYZ(0, 0, 0, 0, 0, 0);

    joint_link_tree_[22]->name_ = "thigh_r_rear_passive";
    joint_link_tree_[22]->parent_ = 21;  // thigh_r_rear_passive_mimic
    joint_link_tree_[22]->sibling_ = -1;
    joint_link_tree_[22]->child_ = -1;
    joint_link_tree_[22]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_tree_[22]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 0, 0);
    joint_link_tree_[22]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[22]->joint_limit_lower_ = -3.1;
    joint_link_tree_[22]->joint_limit_upper_ = 3.1;
    joint_link_tree_[22]->joint_mimic_ = "thigh_r_front_active";
    joint_link_tree_[22]->joint_mimic_multiplier_ = 1.0;
    joint_link_tree_[22]->link_mass_ = 0.011;
    joint_link_tree_[22]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_tree_[22]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[23]->name_ = "thigh_r_middle_passive";
    joint_link_tree_[23]->parent_ = 12;  // hip_r_pitch
    joint_link_tree_[23]->sibling_ = -1;
    joint_link_tree_[23]->child_ = -1;
    joint_link_tree_[23]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0185, -0.02925, -0.0305);
    joint_link_tree_[23]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, -0.785398163, 0);
    joint_link_tree_[23]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[23]->joint_limit_lower_ = -3.1;
    joint_link_tree_[23]->joint_limit_upper_ = 3.1;
    joint_link_tree_[23]->joint_mimic_ = "thigh_r_front_active";
    joint_link_tree_[23]->joint_mimic_multiplier_ = 1.0;
    joint_link_tree_[23]->link_mass_ = 0.012;
    joint_link_tree_[23]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_tree_[23]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[24]->name_ = "shin_r_rear_passive";
    joint_link_tree_[24]->parent_ = 14;  // knee_r_passive
    joint_link_tree_[24]->sibling_ = -1;
    joint_link_tree_[24]->child_ = -1;
    joint_link_tree_[24]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0335, 0, 0);
    joint_link_tree_[24]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 0.785398163, 0);
    joint_link_tree_[24]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[24]->joint_limit_lower_ = -1.8588;
    joint_link_tree_[24]->joint_limit_upper_ = 0.4974;
    joint_link_tree_[24]->joint_mimic_ = "shin_r_active";
    joint_link_tree_[24]->joint_mimic_multiplier_ = 1.0;
    joint_link_tree_[24]->link_mass_ = 0.016;
    joint_link_tree_[24]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_tree_[24]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Left Leg
    joint_link_tree_[25]->name_ = "hip_l_roll";
    joint_link_tree_[25]->parent_ = 1;  // waist
    joint_link_tree_[25]->sibling_ = -1;
    joint_link_tree_[25]->child_ = 26;  // hip_l_pitch
    joint_link_tree_[25]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.08425);
    joint_link_tree_[25]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[25]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    joint_link_tree_[25]->joint_limit_lower_ = -0.5550;
    joint_link_tree_[25]->joint_limit_upper_ = 1.8640;
    joint_link_tree_[25]->link_mass_ = 0.026;
    joint_link_tree_[25]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.00125, 0.0065, -0.004125);
    joint_link_tree_[25]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[26]->name_ = "hip_l_pitch";
    joint_link_tree_[26]->parent_ = 25;  // hip_l_roll
    joint_link_tree_[26]->sibling_ = -1;
    joint_link_tree_[26]->child_ = 27;  // thigh_l_front_active
    joint_link_tree_[26]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.027, 0.0);
    joint_link_tree_[26]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[26]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_tree_[26]->joint_limit_lower_ = -3.1;
    joint_link_tree_[26]->joint_limit_upper_ = 3.1;
    joint_link_tree_[26]->link_mass_ = 0.478;
    joint_link_tree_[26]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.00175, 0.049, -0.015);
    joint_link_tree_[26]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[27]->name_ = "thigh_l_front_active";
    joint_link_tree_[27]->parent_ = 26;   // hip_l_pitch
    joint_link_tree_[27]->sibling_ = 34;  // shin_l_active
    joint_link_tree_[27]->child_ = 28;    // knee_l_passive
    joint_link_tree_[27]->joint_position_ = robotis_framework::getTransitionXYZ(0.015, 0.02925, -0.0305);
    joint_link_tree_[27]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, -0.785398163, 0.0);
    joint_link_tree_[27]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[27]->joint_limit_lower_ = -1.8588;
    joint_link_tree_[27]->joint_limit_upper_ = 0.4974;
    joint_link_tree_[27]->link_mass_ = 0.012;
    joint_link_tree_[27]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.05);
    joint_link_tree_[27]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[28]->name_ = "knee_l_passive";
    joint_link_tree_[28]->parent_ = 27;  // thigh_l_front_active
    joint_link_tree_[28]->sibling_ = -1;
    joint_link_tree_[28]->child_ = 29;  // shin_l_front_passive
    joint_link_tree_[28]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.1);
    joint_link_tree_[28]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.785398163, 0.0);
    joint_link_tree_[28]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[28]->joint_limit_lower_ = -0.4974;
    joint_link_tree_[28]->joint_limit_upper_ = 1.8588;
    joint_link_tree_[28]->joint_mimic_ = "thigh_l_front_active";
    joint_link_tree_[28]->joint_mimic_multiplier_ = -1.0;
    joint_link_tree_[28]->link_mass_ = 0.007;
    joint_link_tree_[28]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.01675, 0.0, 0.0);
    joint_link_tree_[28]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[29]->name_ = "shin_l_front_passive";
    joint_link_tree_[29]->parent_ = 28;   // knee_l_passive
    joint_link_tree_[29]->sibling_ = 33;  // shin_l_rear_passive
    joint_link_tree_[29]->child_ = 30;    // ankle_l_pitch_passive
    joint_link_tree_[29]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[29]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.785398163, 0.0);
    joint_link_tree_[29]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[29]->joint_limit_lower_ = -0.4974;
    joint_link_tree_[29]->joint_limit_upper_ = 1.8588;
    joint_link_tree_[29]->joint_mimic_ = "shin_l_active";
    joint_link_tree_[29]->joint_mimic_multiplier_ = 1.0;
    joint_link_tree_[29]->link_mass_ = 0.013;
    joint_link_tree_[29]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.05);
    joint_link_tree_[29]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[30]->name_ = "ankle_l_pitch_passive";
    joint_link_tree_[30]->parent_ = 29;  // shin_l_front_passive
    joint_link_tree_[30]->sibling_ = -1;
    joint_link_tree_[30]->child_ = 31;  // ankle_l_roll
    joint_link_tree_[30]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.1);
    joint_link_tree_[30]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, -0.785398163, 0.0);
    joint_link_tree_[30]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[30]->joint_limit_lower_ = -1.8588;
    joint_link_tree_[30]->joint_limit_upper_ = 0.4974;
    joint_link_tree_[30]->joint_mimic_ = "shin_l_active";
    joint_link_tree_[30]->joint_mimic_multiplier_ = -1.0;
    joint_link_tree_[30]->link_mass_ = 0.104;
    joint_link_tree_[30]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.0285, 0, -0.013);
    joint_link_tree_[30]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[31]->name_ = "ankle_l_roll";
    joint_link_tree_[31]->parent_ = 30;  // ankle_l_pitch_passive
    joint_link_tree_[31]->sibling_ = -1;
    joint_link_tree_[31]->child_ = 32;  // ankle_l_yaw
    joint_link_tree_[31]->joint_position_ = robotis_framework::getTransitionXYZ(-0.015, 0, -0.027);
    joint_link_tree_[31]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[31]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    joint_link_tree_[31]->joint_limit_lower_ = -1.5708;
    joint_link_tree_[31]->joint_limit_upper_ = 0.3704;
    joint_link_tree_[31]->link_mass_ = 0.0136;
    joint_link_tree_[31]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0.009375, -0.01175);
    joint_link_tree_[31]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[32]->name_ = "ankle_l_yaw";
    joint_link_tree_[32]->parent_ = 31;  // ankle_l_roll
    joint_link_tree_[32]->sibling_ = -1;
    joint_link_tree_[32]->child_ = 33;  // leg_l_end
    joint_link_tree_[32]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.03825);
    joint_link_tree_[32]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[32]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    joint_link_tree_[32]->joint_limit_lower_ = -3.1;
    joint_link_tree_[32]->joint_limit_upper_ = 3.1;
    joint_link_tree_[32]->link_mass_ = 0.043;
    joint_link_tree_[32]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.004);
    joint_link_tree_[32]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[33]->name_ = "leg_l_end";
    joint_link_tree_[33]->parent_ = 32;  // ankle_l_yaw
    joint_link_tree_[33]->sibling_ = -1;
    joint_link_tree_[33]->child_ = -1;
    joint_link_tree_[33]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.004);
    joint_link_tree_[33]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_tree_[33]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_tree_[33]->joint_limit_lower_ = -100;
    joint_link_tree_[33]->joint_limit_upper_ = 100;
    joint_link_tree_[33]->link_mass_ = 0;
    joint_link_tree_[33]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_tree_[33]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[34]->name_ = "shin_l_active";
    joint_link_tree_[34]->parent_ = 26;   // hip_l_pitch
    joint_link_tree_[34]->sibling_ = 37;  // thigh_l_middle_passive_link
    joint_link_tree_[34]->child_ = 35;    // thigh_l_rear_passive_mimic
    joint_link_tree_[34]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0185, 0.02925, -0.0305);
    joint_link_tree_[34]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 1.047197551, 0);
    joint_link_tree_[34]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[34]->joint_limit_lower_ = -0.4974;
    joint_link_tree_[34]->joint_limit_upper_ = 1.8588;
    joint_link_tree_[34]->link_mass_ = 0.008;
    joint_link_tree_[34]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.004745, 0, -0.015);
    joint_link_tree_[34]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[35]->name_ = "thigh_l_rear_passive_mimic";
    joint_link_tree_[35]->parent_ = 34;  // shin_l_active
    joint_link_tree_[35]->sibling_ = -1;
    joint_link_tree_[35]->child_ = 36;  // thigh_l_rear_passive
    joint_link_tree_[35]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.03);
    joint_link_tree_[35]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, -1.832597551, 0);
    joint_link_tree_[35]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[35]->joint_limit_lower_ = -3.1;
    joint_link_tree_[35]->joint_limit_upper_ = 3.1;
    joint_link_tree_[35]->joint_mimic_ = "shin_l_active";
    joint_link_tree_[35]->joint_mimic_multiplier_ = -1.0;
    joint_link_tree_[35]->link_mass_ = 0.0;
    joint_link_tree_[35]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_tree_[35]->link_inertia_ = robotis_framework::getInertiaXYZ(0, 0, 0, 0, 0, 0);

    joint_link_tree_[36]->name_ = "thigh_l_rear_passive";
    joint_link_tree_[36]->parent_ = 35;  // thigh_l_rear_passive_mimic
    joint_link_tree_[36]->sibling_ = -1;
    joint_link_tree_[36]->child_ = -1;
    joint_link_tree_[36]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_tree_[36]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 0, 0);
    joint_link_tree_[36]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[36]->joint_limit_lower_ = -3.1;
    joint_link_tree_[36]->joint_limit_upper_ = 3.1;
    joint_link_tree_[36]->joint_mimic_ = "thigh_l_front_active";
    joint_link_tree_[36]->joint_mimic_multiplier_ = 1.0;
    joint_link_tree_[36]->link_mass_ = 0.011;
    joint_link_tree_[36]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_tree_[36]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[37]->name_ = "thigh_l_middle_passive";
    joint_link_tree_[37]->parent_ = 26;  // hip_l_pitch
    joint_link_tree_[37]->sibling_ = -1;
    joint_link_tree_[37]->child_ = -1;
    joint_link_tree_[37]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0185, 0.02925, -0.0305);
    joint_link_tree_[37]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, -0.785398163, 0);
    joint_link_tree_[37]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[37]->joint_limit_lower_ = -3.1;
    joint_link_tree_[37]->joint_limit_upper_ = 3.1;
    joint_link_tree_[37]->joint_mimic_ = "thigh_l_front_active";
    joint_link_tree_[37]->joint_mimic_multiplier_ = 1.0;
    joint_link_tree_[37]->link_mass_ = 0.012;
    joint_link_tree_[37]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_tree_[37]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_tree_[38]->name_ = "shin_l_rear_passive";
    joint_link_tree_[38]->parent_ = 29;  // knee_l_passive
    joint_link_tree_[38]->sibling_ = -1;
    joint_link_tree_[38]->child_ = -1;
    joint_link_tree_[38]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0335, 0, 0);
    joint_link_tree_[38]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 0.785398163, 0);
    joint_link_tree_[38]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_tree_[38]->joint_limit_lower_ = -0.4974;
    joint_link_tree_[38]->joint_limit_upper_ = 1.8588;
    joint_link_tree_[38]->joint_mimic_ = "shin_l_active";
    joint_link_tree_[38]->joint_mimic_multiplier_ = 1.0;
    joint_link_tree_[38]->link_mass_ = 0.016;
    joint_link_tree_[38]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_tree_[38]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  leg_side_offset_ =
      2.0 * (std::fabs(joint_link_tree_[getLinkIndex("hip_r_roll")]->joint_position_.coeff(1, 0) +
                       joint_link_tree_[getLinkIndex("hip_r_pitch")]->joint_position_.coeff(1, 0) +
                       joint_link_tree_[getLinkIndex("thigh_r_front_active")]->joint_position_.coeff(1, 0)));
  leg_max_height_ = std::fabs(joint_link_tree_[getLinkIndex("thigh_r_front_active")]->joint_position_.coeff(2, 0) +
                              joint_link_tree_[getLinkIndex("knee_r_passive")]->joint_position_.coeff(2, 0) +
                              joint_link_tree_[getLinkIndex("shin_r_front_passive")]->joint_position_.coeff(2, 0) +
                              joint_link_tree_[getLinkIndex("ankle_r_pitch_passive")]->joint_position_.coeff(2, 0) +
                              joint_link_tree_[getLinkIndex("ankle_r_roll")]->joint_position_.coeff(2, 0) +
                              joint_link_tree_[getLinkIndex("ankle_r_yaw")]->joint_position_.coeff(2, 0) +
                              joint_link_tree_[getLinkIndex("leg_r_end")]->joint_position_.coeff(2, 0));
}

std::vector<int> KurokoKinematics::findRoute(int to)
{
  int id = joint_link_tree_[to]->parent_;

  std::vector<int> idx;

  if (id == 0)
  {
    idx.push_back(0);
    idx.push_back(to);
  }
  else
  {
    idx = findRoute(id);
    idx.push_back(to);
  }

  return idx;
}

std::vector<int> KurokoKinematics::findRoute(int from, int to)
{
  int id = joint_link_tree_[to]->parent_;

  std::vector<int> idx;

  if (id == from)
  {
    idx.push_back(from);
    idx.push_back(to);
  }
  else if (id != 0)
  {
    idx = findRoute(from, id);
    idx.push_back(to);
  }

  return idx;
}

// Calculate the mass of the robot
double KurokoKinematics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = joint_link_tree_[joint_id]->link_mass_ + calcTotalMass(joint_link_tree_[joint_id]->sibling_) +
           calcTotalMass(joint_link_tree_[joint_id]->child_);

  return mass;
}

// Moment of inertia created by all links around the origin of the world coordinate system
Eigen::MatrixXd KurokoKinematics::calcMC(int joint_id)
{
  Eigen::MatrixXd mc(3, 1);

  if (joint_id == -1)
    mc = Eigen::MatrixXd::Zero(3, 1);
  else
  {
    mc = joint_link_tree_[joint_id]->link_mass_ *
         (joint_link_tree_[joint_id]->internal_orientation_ * joint_link_tree_[joint_id]->link_center_of_mass_ +
          joint_link_tree_[joint_id]->internal_position_);
    mc = mc + calcMC(joint_link_tree_[joint_id]->sibling_) + calcMC(joint_link_tree_[joint_id]->child_);
  }

  return mc;
}

// Calculate the robot's center of gravity
Eigen::MatrixXd KurokoKinematics::calcCOM(const Eigen::MatrixXd& mc)
{
  double mass;
  Eigen::MatrixXd com(3, 1);

  mass = calcTotalMass(0);
  com = mc / mass;

  return com;
}

void KurokoKinematics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
  {
    joint_link_tree_[0]->internal_position_ = Eigen::MatrixXd::Zero(3, 1);
    joint_link_tree_[0]->internal_orientation_ = joint_link_tree_[0]->joint_orientation_;
  }

  if (joint_id != 0)
  {
    int parent = joint_link_tree_[joint_id]->parent_;
    double joint_angle = joint_link_tree_[joint_id]->internal_joint_angle_;

    if (!joint_link_tree_[joint_id]->joint_mimic_.empty())
    {
      int mimic_id = getLinkIndex(joint_link_tree_[joint_id]->joint_mimic_);
      if (mimic_id != -1)
      {
        joint_angle =
            joint_link_tree_[mimic_id]->internal_joint_angle_ * joint_link_tree_[joint_id]->joint_mimic_multiplier_;
      }
    }

    joint_link_tree_[joint_id]->internal_position_ =
        joint_link_tree_[parent]->internal_orientation_ * joint_link_tree_[joint_id]->joint_position_ +
        joint_link_tree_[parent]->internal_position_;

    joint_link_tree_[joint_id]->internal_orientation_ =
        joint_link_tree_[parent]->internal_orientation_ * joint_link_tree_[joint_id]->joint_orientation_ *
        robotis_framework::calcRodrigues(robotis_framework::calcHatto(joint_link_tree_[joint_id]->joint_axis_),
                                         joint_angle);

    joint_link_tree_[joint_id]->internal_transformation_.block<3, 1>(0, 3) =
        joint_link_tree_[joint_id]->internal_position_;
    joint_link_tree_[joint_id]->internal_transformation_.block<3, 3>(0, 0) =
        joint_link_tree_[joint_id]->internal_orientation_;
  }
  calcForwardKinematics(joint_link_tree_[joint_id]->sibling_);
  calcForwardKinematics(joint_link_tree_[joint_id]->child_);
}

Eigen::MatrixXd KurokoKinematics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = joint_link_tree_[idx[end]]->internal_position_;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, idx_size);

  for (int id = 0; id < idx_size; id++)
  {
    int curr_id = idx[id];

    Eigen::MatrixXd tar_orientation =
        joint_link_tree_[curr_id]->internal_orientation_ * joint_link_tree_[curr_id]->joint_axis_;

    jacobian.block(0, id, 3, 1) =
        robotis_framework::calcCross(tar_orientation, tar_position - joint_link_tree_[curr_id]->internal_position_);
    jacobian.block(3, id, 3, 1) = tar_orientation;
  }

  return jacobian;
}

Eigen::MatrixXd KurokoKinematics::calcJacobianCOM(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = joint_link_tree_[idx[end]]->internal_position_;
  Eigen::MatrixXd jacobian_com = Eigen::MatrixXd::Zero(6, idx_size);

  for (int id = 0; id < idx_size; id++)
  {
    int curr_id = idx[id];
    double mass = calcTotalMass(curr_id);

    Eigen::MatrixXd og = calcMC(curr_id) / mass - joint_link_tree_[curr_id]->internal_position_;
    Eigen::MatrixXd tar_orientation =
        joint_link_tree_[curr_id]->internal_orientation_ * joint_link_tree_[curr_id]->joint_axis_;

    jacobian_com.block(0, id, 3, 1) = robotis_framework::calcCross(tar_orientation, og);
    jacobian_com.block(3, id, 3, 1) = tar_orientation;
  }

  return jacobian_com;
}

Eigen::MatrixXd KurokoKinematics::calcVWerr(const Eigen::MatrixXd& tar_position, const Eigen::MatrixXd& curr_position,
                                            const Eigen::MatrixXd& tar_orientation, Eigen::MatrixXd curr_orientation)
{
  Eigen::MatrixXd pos_err = tar_position - curr_position;
  Eigen::MatrixXd ori_err = curr_orientation.transpose() * tar_orientation;
  Eigen::MatrixXd ori_err_dash = curr_orientation * robotis_framework::convertRotToOmega(ori_err);

  Eigen::MatrixXd err = Eigen::MatrixXd::Zero(6, 1);
  err.block<3, 1>(0, 0) = pos_err;
  err.block<3, 1>(3, 0) = ori_err_dash;

  return err;
}

bool KurokoKinematics::calcInverseKinematicsForRightLeg(double* out, double x, double y, double z, double roll,
                                                        double pitch, double yaw)
{
  // std::cout << "Calculating Inverse Kinematics for Right Leg..." << std::endl;
  // std::cout << "Input Pose:" << std::endl;
  // std::cout << "X: " << x << std::endl;
  // std::cout << "Y: " << y << std::endl;
  // std::cout << "Z: " << z << std::endl;
  // std::cout << "Roll: " << roll << std::endl;
  // std::cout << "Pitch: " << pitch << std::endl;
  // std::cout << "Yaw: " << yaw << std::endl;
  // Define link lengths based on the robot's dimensions
  double hip_roll_to_pitch_offset_y = -leg_side_offset_ / 2.0;
  double hip_pitch_to_thigh_upper_z =
      joint_link_tree_[getLinkIndex("thigh_r_front_active")]->joint_position_.coeff(2, 0);
  double thigh_length = fabs(joint_link_tree_[getLinkIndex("knee_r_passive")]->joint_position_.coeff(2, 0));
  double shin_length = fabs(joint_link_tree_[getLinkIndex("ankle_r_pitch_passive")]->joint_position_.coeff(2, 0));
  double shin_lower_to_ankle_roll_offset_z =
      joint_link_tree_[getLinkIndex("ankle_r_roll")]->joint_position_.coeff(2, 0);
  double ankle_roll_to_yaw_offset_z = joint_link_tree_[getLinkIndex("ankle_r_yaw")]->joint_position_.coeff(2, 0) +
                                      joint_link_tree_[getLinkIndex("leg_r_end")]->joint_position_.coeff(2, 0);

  // Ankle yaw
  double ankle_yaw = yaw;

  // Hip pitch
  double hip_pitch = pitch;

  // Hip roll
  double waist_to_ankle_roll_x = x;
  double waist_to_ankle_roll_y = y + ankle_roll_to_yaw_offset_z * sin(roll);
  double waist_to_ankle_roll_z = z - ankle_roll_to_yaw_offset_z * cos(roll);
  double waist_to_ankle_roll_yz_plane_distance =
      sqrt(waist_to_ankle_roll_z * waist_to_ankle_roll_z + waist_to_ankle_roll_y * waist_to_ankle_roll_y);
  double hip_roll = acos(hip_roll_to_pitch_offset_y / waist_to_ankle_roll_yz_plane_distance) +
                    atan2(waist_to_ankle_roll_y, -waist_to_ankle_roll_z) - 0.5 * M_PI;
  // std::cout << "Hip Roll: " << acos(hip_roll_to_pitch_offset_y / waist_to_ankle_roll_yz_plane_distance) << "+"
  //           << atan2(waist_to_ankle_roll_y, waist_to_ankle_roll_z) << "-" << 0.5 * M_PI << "=" << hip_roll << std::endl;
  // std::cout << "waist_to_ankle_roll_y: " << waist_to_ankle_roll_y << std::endl;
  // std::cout << "waist_to_ankle_roll_z: " << waist_to_ankle_roll_z << std::endl;
  // Ankle roll
  double ankle_roll = roll - hip_roll;

  // Thigh upper and Shin lower joints
  double hip_roll_to_target_x = x;
  double hip_roll_to_target_z = z * cos(hip_roll) - y * sin(hip_roll);
  double hip_roll_to_target_y = z * sin(hip_roll) + y * cos(hip_roll);
  std::cout << "hip_roll: " << hip_roll << std::endl;
  std::cout << "hip_roll_to_target_x: " << hip_roll_to_target_x << std::endl;
  std::cout << "hip_roll_to_target_y: " << hip_roll_to_target_y << std::endl;
  std::cout << "hip_roll_to_target_z: " << hip_roll_to_target_z << std::endl;
  double hip_pitch_to_target_y = hip_roll_to_target_y - hip_roll_to_pitch_offset_y;
  double hip_pitch_to_target_z = hip_roll_to_target_z * cos(-hip_pitch) - hip_roll_to_target_x * sin(-hip_pitch);
  double hip_pitch_to_target_x = hip_roll_to_target_z * sin(-hip_pitch) + hip_roll_to_target_x * cos(-hip_pitch);
  std::cout << "hip_pitch_to_target_x: " << hip_pitch_to_target_x << std::endl;
  std::cout << "hip_pitch_to_target_y: " << hip_pitch_to_target_y << std::endl;
  std::cout << "hip_pitch_to_target_z: " << hip_pitch_to_target_z << std::endl;
  double thigh_upper_to_shin_lower_x = hip_pitch_to_target_x;
  double thigh_upper_to_shin_lower_z = hip_pitch_to_target_z - hip_pitch_to_thigh_upper_z -
                                       shin_lower_to_ankle_roll_offset_z - ankle_roll_to_yaw_offset_z * cos(ankle_roll);
  std::cout << "thigh_upper_to_shin_lower_z: " << thigh_upper_to_shin_lower_z << std::endl;
  if (thigh_upper_to_shin_lower_z > -0.001)
  {
    std::cout << "Target position is out of reach (too high)" << std::endl;
    thigh_upper_to_shin_lower_z = -0.001;
  }
  double triangle_knee_angle, triangle_thigh_angle, triangle_shin_angle;
  double triangle_knee_line_length = sqrt(pow(thigh_upper_to_shin_lower_x, 2) + pow(thigh_upper_to_shin_lower_z, 2));
  double tiangle_thigh_line_length = shin_length;
  double triangle_shin_line_length = thigh_length;
  // std::cout << "Triangle Knee Line legth: " << triangle_knee_line_length << std::endl;
  // std::cout << "Triangle Thigh Line legth: " << tiangle_thigh_line_length << std::endl;
  // std::cout << "Triangle Shin Line legth: " << triangle_shin_line_length << std::endl;
  if (triangle_knee_line_length > tiangle_thigh_line_length + triangle_shin_line_length ||
      triangle_knee_line_length > triangle_knee_line_length + tiangle_thigh_line_length ||
      tiangle_thigh_line_length > triangle_knee_line_length + triangle_shin_line_length)
  {
    std::cout << "Target position is out of reach (too low)" << std::endl;
    triangle_knee_angle = M_PI;
    triangle_thigh_angle = 0;
    triangle_shin_angle = 0;
  }
  else
  {
    triangle_knee_angle = acos(
        (pow(triangle_shin_line_length, 2) + pow(tiangle_thigh_line_length, 2) - pow(triangle_knee_line_length, 2)) /
        (2 * triangle_shin_line_length * tiangle_thigh_line_length));
    triangle_thigh_angle = acos(
        (pow(triangle_shin_line_length, 2) + pow(triangle_knee_line_length, 2) - pow(tiangle_thigh_line_length, 2)) /
        (2 * triangle_shin_line_length * triangle_knee_line_length));
    triangle_shin_angle = M_PI - triangle_knee_angle - triangle_thigh_angle;
  }
  std::cout << "Triangle Knee Angle: " << triangle_knee_angle << std::endl;
  std::cout << "Triangle Thigh Angle: " << triangle_thigh_angle << std::endl;
  std::cout << "Triangle Shin Angle: " << triangle_shin_angle << std::endl;
  double triangle_knee_line_angle = atan2(-thigh_upper_to_shin_lower_x, -thigh_upper_to_shin_lower_z);
  std::cout << "Triangle Knee Line Angle: " << triangle_knee_line_angle << std::endl;
  double shin_pitch = triangle_knee_line_angle + triangle_shin_angle;
  double thigh_pitch = -((M_PI - triangle_knee_angle) - shin_pitch);

  // Set the output joint angles
  double ankle_yaw_joint = joint_link_tree_[getLinkIndex("ankle_r_yaw")]->joint_axis_.coeff(2, 0) *
                           (ankle_yaw - robotis_framework::convertRotationToRPY(
                                            joint_link_tree_[getLinkIndex("ankle_r_yaw")]->joint_orientation_)
                                            .coeff(2, 0));
  double ankle_roll_joint = joint_link_tree_[getLinkIndex("ankle_r_roll")]->joint_axis_.coeff(0, 0) *
                            (ankle_roll - robotis_framework::convertRotationToRPY(
                                              joint_link_tree_[getLinkIndex("ankle_r_roll")]->joint_orientation_)
                                              .coeff(0, 0));
  double shin_pitch_joint =
      joint_link_tree_[getLinkIndex("shin_r_front_passive")]->joint_axis_.coeff(1, 0) *
      (shin_pitch - robotis_framework::convertRotationToRPY(
                        joint_link_tree_[getLinkIndex("shin_r_front_passive")]->joint_orientation_)
                        .coeff(1, 0));
  double thigh_pitch_joint =
      joint_link_tree_[getLinkIndex("thigh_r_front_active")]->joint_axis_.coeff(1, 0) *
      (thigh_pitch - robotis_framework::convertRotationToRPY(
                         joint_link_tree_[getLinkIndex("thigh_r_front_active")]->joint_orientation_)
                         .coeff(1, 0));
  double hip_pitch_joint = joint_link_tree_[getLinkIndex("hip_r_pitch")]->joint_axis_.coeff(1, 0) *
                           (hip_pitch - robotis_framework::convertRotationToRPY(
                                            joint_link_tree_[getLinkIndex("hip_r_pitch")]->joint_orientation_)
                                            .coeff(1, 0));
  double hip_roll_joint = joint_link_tree_[getLinkIndex("hip_r_roll")]->joint_axis_.coeff(0, 0) *
                          (hip_roll - robotis_framework::convertRotationToRPY(
                                          joint_link_tree_[getLinkIndex("hip_r_roll")]->joint_orientation_)
                                          .coeff(0, 0));

  hip_roll_joint = std::max(std::min(hip_roll_joint, joint_link_tree_[getLinkIndex("hip_r_roll")]->joint_limit_upper_),
                            joint_link_tree_[getLinkIndex("hip_r_roll")]->joint_limit_lower_);
  hip_pitch_joint =
      std::max(std::min(hip_pitch_joint, joint_link_tree_[getLinkIndex("hip_r_pitch")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("hip_r_pitch")]->joint_limit_lower_);
  thigh_pitch_joint =
      std::max(std::min(thigh_pitch_joint, joint_link_tree_[getLinkIndex("thigh_r_front_active")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("thigh_r_front_active")]->joint_limit_lower_);
  shin_pitch_joint =
      std::max(std::min(shin_pitch_joint, joint_link_tree_[getLinkIndex("shin_r_front_passive")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("shin_r_front_passive")]->joint_limit_lower_);
  ankle_roll_joint =
      std::max(std::min(ankle_roll_joint, joint_link_tree_[getLinkIndex("ankle_r_roll")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("ankle_r_roll")]->joint_limit_lower_);
  ankle_yaw_joint =
      std::max(std::min(ankle_yaw_joint, joint_link_tree_[getLinkIndex("ankle_r_yaw")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("ankle_r_yaw")]->joint_limit_lower_);
  double shin_active_joint =
      shin_pitch_joint / joint_link_tree_[getLinkIndex("shin_r_front_passive")]->joint_mimic_multiplier_;
  shin_active_joint =
      std::max(std::min(shin_active_joint, joint_link_tree_[getLinkIndex("shin_r_active")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("shin_r_active")]->joint_limit_lower_);

  out[0] = hip_roll_joint;
  out[1] = hip_pitch_joint;
  out[2] = thigh_pitch_joint;
  out[3] = shin_active_joint;
  out[4] = ankle_roll_joint;
  out[5] = ankle_yaw_joint;
  // std::cout << "Inverse Kinematics result (Right Leg):" << std::endl;
  // std::cout << "Joint ID: 11 (hip_r_roll), Angle: " << out[0] << std::endl;
  // std::cout << "Joint ID: 12 (hip_r_pitch), Angle: " << out[1] << std::endl;
  // std::cout << "Joint ID: 13 (thigh_r_front_active), Angle: " << out[2] << std::endl;
  // std::cout << "Joint ID: 20 (shin_r_active), Angle: " << out[3] << std::endl;
  // std::cout << "Joint ID: 17 (ankle_r_roll), Angle: " << out[4] << std::endl;
  // std::cout << "Joint ID: 18 (ankle_r_yaw), Angle: " << out[5] << std::endl;

  return true;
}

bool KurokoKinematics::calcInverseKinematicsForLeftLeg(double* out, double x, double y, double z, double roll,
                                                       double pitch, double yaw)
{
  std::cout << "Calculating Inverse Kinematics for Left Leg..." << std::endl;
  std::cout << "Input Pose:" << std::endl;
  std::cout << "X: " << x << std::endl;
  std::cout << "Y: " << y << std::endl;
  std::cout << "Z: " << z << std::endl;
  std::cout << "Roll: " << roll << std::endl;
  std::cout << "Pitch: " << pitch << std::endl;
  std::cout << "Yaw: " << yaw << std::endl;
  // Define link lengths based on the robot's dimensions
  double hip_roll_to_pitch_offset_y = leg_side_offset_ / 2.0;
  double hip_pitch_to_thigh_upper_z =
      joint_link_tree_[getLinkIndex("thigh_l_front_active")]->joint_position_.coeff(2, 0);
  double thigh_length = fabs(joint_link_tree_[getLinkIndex("knee_l_passive")]->joint_position_.coeff(2, 0));
  double shin_length = fabs(joint_link_tree_[getLinkIndex("ankle_l_pitch_passive")]->joint_position_.coeff(2, 0));
  double shin_lower_to_ankle_roll_offset_z =
      joint_link_tree_[getLinkIndex("ankle_l_roll")]->joint_position_.coeff(2, 0);
  double ankle_roll_to_yaw_offset_z = joint_link_tree_[getLinkIndex("ankle_l_yaw")]->joint_position_.coeff(2, 0) +
                                      joint_link_tree_[getLinkIndex("leg_l_end")]->joint_position_.coeff(2, 0);

  // Ankle yaw
  double ankle_yaw = yaw;

  // Hip pitch
  double hip_pitch = pitch;

  // Hip roll
  double waist_to_ankle_roll_x = x;
  double waist_to_ankle_roll_y = y + ankle_roll_to_yaw_offset_z * sin(roll);
  double waist_to_ankle_roll_z = z - ankle_roll_to_yaw_offset_z * cos(roll);
  double waist_to_ankle_roll_yz_plane_distance =
      sqrt(waist_to_ankle_roll_z * waist_to_ankle_roll_z + waist_to_ankle_roll_y * waist_to_ankle_roll_y);
  double hip_roll = acos(hip_roll_to_pitch_offset_y / waist_to_ankle_roll_yz_plane_distance) +
                    atan2(waist_to_ankle_roll_y, -waist_to_ankle_roll_z) - 0.5 * M_PI;
  // std::cout << "Hip Roll: " << acos(hip_roll_to_pitch_offset_y / waist_to_ankle_roll_yz_plane_distance) << "+"
  //           << atan2(waist_to_ankle_roll_y, waist_to_ankle_roll_z) << "-" << 0.5 * M_PI << "=" << hip_roll << std::endl;
  // std::cout << "waist_to_ankle_roll_y: " << waist_to_ankle_roll_y << std::endl;
  // std::cout << "waist_to_ankle_roll_z: " << waist_to_ankle_roll_z << std::endl;
  // Ankle roll
  double ankle_roll = roll - hip_roll;

  // Thigh upper and Shin lower joints
  double hip_roll_to_target_x = x;
  double hip_roll_to_target_z = z * cos(hip_roll) - y * sin(hip_roll);
  double hip_roll_to_target_y = z * sin(hip_roll) + y * cos(hip_roll);
  std::cout << "hip_roll: " << hip_roll << std::endl;
  std::cout << "hip_roll_to_target_x: " << hip_roll_to_target_x << std::endl;
  std::cout << "hip_roll_to_target_y: " << hip_roll_to_target_y << std::endl;
  std::cout << "hip_roll_to_target_z: " << hip_roll_to_target_z << std::endl;
  double hip_pitch_to_target_y = hip_roll_to_target_y - hip_roll_to_pitch_offset_y;
  double hip_pitch_to_target_z = hip_roll_to_target_z * cos(-hip_pitch) - hip_roll_to_target_x * sin(-hip_pitch);
  double hip_pitch_to_target_x = hip_roll_to_target_z * sin(-hip_pitch) + hip_roll_to_target_x * cos(-hip_pitch);
  std::cout << "hip_pitch_to_target_x: " << hip_pitch_to_target_x << std::endl;
  std::cout << "hip_pitch_to_target_y: " << hip_pitch_to_target_y << std::endl;
  std::cout << "hip_pitch_to_target_z: " << hip_pitch_to_target_z << std::endl;
  double thigh_upper_to_shin_lower_x = hip_pitch_to_target_x;
  double thigh_upper_to_shin_lower_z = hip_pitch_to_target_z - hip_pitch_to_thigh_upper_z -
                                       shin_lower_to_ankle_roll_offset_z - ankle_roll_to_yaw_offset_z * cos(ankle_roll);
  std::cout << "thigh_upper_to_shin_lower_z: " << thigh_upper_to_shin_lower_z << std::endl;
  if (thigh_upper_to_shin_lower_z > -0.001)
  {
    std::cout << "Target position is out of reach (too high)" << std::endl;
    thigh_upper_to_shin_lower_z = -0.001;
  }
  double triangle_knee_angle, triangle_thigh_angle, triangle_shin_angle;
  double triangle_knee_line_length = sqrt(pow(thigh_upper_to_shin_lower_x, 2) + pow(thigh_upper_to_shin_lower_z, 2));
  double tiangle_thigh_line_length = shin_length;
  double triangle_shin_line_length = thigh_length;
  // std::cout << "Triangle Knee Line legth: " << triangle_knee_line_length << std::endl;
  // std::cout << "Triangle Thigh Line legth: " << tiangle_thigh_line_length << std::endl;
  // std::cout << "Triangle Shin Line legth: " << triangle_shin_line_length << std::endl;
  if (triangle_knee_line_length > tiangle_thigh_line_length + triangle_shin_line_length ||
      triangle_knee_line_length > triangle_knee_line_length + tiangle_thigh_line_length ||
      tiangle_thigh_line_length > triangle_knee_line_length + triangle_shin_line_length)
  {
    std::cout << "Target position is out of reach (too low)" << std::endl;
    triangle_knee_angle = M_PI;
    triangle_thigh_angle = 0;
    triangle_shin_angle = 0;
  }
  else
  {
    triangle_knee_angle = acos(
        (pow(triangle_shin_line_length, 2) + pow(tiangle_thigh_line_length, 2) - pow(triangle_knee_line_length, 2)) /
        (2 * triangle_shin_line_length * tiangle_thigh_line_length));
    triangle_thigh_angle = acos(
        (pow(triangle_shin_line_length, 2) + pow(triangle_knee_line_length, 2) - pow(tiangle_thigh_line_length, 2)) /
        (2 * triangle_shin_line_length * triangle_knee_line_length));
    triangle_shin_angle = M_PI - triangle_knee_angle - triangle_thigh_angle;
  }
  std::cout << "Triangle Knee Angle: " << triangle_knee_angle << std::endl;
  std::cout << "Triangle Thigh Angle: " << triangle_thigh_angle << std::endl;
  std::cout << "Triangle Shin Angle: " << triangle_shin_angle << std::endl;
  double triangle_knee_line_angle = atan2(-thigh_upper_to_shin_lower_x, -thigh_upper_to_shin_lower_z);
  std::cout << "Triangle Knee Line Angle: " << triangle_knee_line_angle << std::endl;
  double shin_pitch = triangle_knee_line_angle + triangle_shin_angle;
  double thigh_pitch = -((M_PI - triangle_knee_angle) - shin_pitch);

  // Set the output joint angles
  double ankle_yaw_joint = joint_link_tree_[getLinkIndex("ankle_l_yaw")]->joint_axis_.coeff(2, 0) *
                           (ankle_yaw - robotis_framework::convertRotationToRPY(
                                            joint_link_tree_[getLinkIndex("ankle_l_yaw")]->joint_orientation_)
                                            .coeff(2, 0));
  double ankle_roll_joint = joint_link_tree_[getLinkIndex("ankle_l_roll")]->joint_axis_.coeff(0, 0) *
                            (ankle_roll - robotis_framework::convertRotationToRPY(
                                              joint_link_tree_[getLinkIndex("ankle_l_roll")]->joint_orientation_)
                                              .coeff(0, 0));
  double shin_pitch_joint =
      joint_link_tree_[getLinkIndex("shin_l_front_passive")]->joint_axis_.coeff(1, 0) *
      (shin_pitch - robotis_framework::convertRotationToRPY(
                        joint_link_tree_[getLinkIndex("shin_l_front_passive")]->joint_orientation_)
                        .coeff(1, 0));
  double thigh_pitch_joint =
      joint_link_tree_[getLinkIndex("thigh_l_front_active")]->joint_axis_.coeff(1, 0) *
      (thigh_pitch - robotis_framework::convertRotationToRPY(
                         joint_link_tree_[getLinkIndex("thigh_l_front_active")]->joint_orientation_)
                         .coeff(1, 0));
  double hip_pitch_joint = joint_link_tree_[getLinkIndex("hip_l_pitch")]->joint_axis_.coeff(1, 0) *
                           (hip_pitch - robotis_framework::convertRotationToRPY(
                                            joint_link_tree_[getLinkIndex("hip_l_pitch")]->joint_orientation_)
                                            .coeff(1, 0));
  double hip_roll_joint = joint_link_tree_[getLinkIndex("hip_l_roll")]->joint_axis_.coeff(0, 0) *
                          (hip_roll - robotis_framework::convertRotationToRPY(
                                          joint_link_tree_[getLinkIndex("hip_l_roll")]->joint_orientation_)
                                          .coeff(0, 0));

  hip_roll_joint = std::max(std::min(hip_roll_joint, joint_link_tree_[getLinkIndex("hip_l_roll")]->joint_limit_upper_),
                            joint_link_tree_[getLinkIndex("hip_l_roll")]->joint_limit_lower_);
  hip_pitch_joint =
      std::max(std::min(hip_pitch_joint, joint_link_tree_[getLinkIndex("hip_l_pitch")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("hip_l_pitch")]->joint_limit_lower_);
  thigh_pitch_joint =
      std::max(std::min(thigh_pitch_joint, joint_link_tree_[getLinkIndex("thigh_l_front_active")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("thigh_l_front_active")]->joint_limit_lower_);
  shin_pitch_joint =
      std::max(std::min(shin_pitch_joint, joint_link_tree_[getLinkIndex("shin_l_front_passive")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("shin_l_front_passive")]->joint_limit_lower_);
  ankle_roll_joint =
      std::max(std::min(ankle_roll_joint, joint_link_tree_[getLinkIndex("ankle_l_roll")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("ankle_l_roll")]->joint_limit_lower_);
  ankle_yaw_joint =
      std::max(std::min(ankle_yaw_joint, joint_link_tree_[getLinkIndex("ankle_l_yaw")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("ankle_l_yaw")]->joint_limit_lower_);
  double shin_active_joint =
      shin_pitch_joint / joint_link_tree_[getLinkIndex("shin_l_front_passive")]->joint_mimic_multiplier_;
  shin_active_joint =
      std::max(std::min(shin_active_joint, joint_link_tree_[getLinkIndex("shin_l_active")]->joint_limit_upper_),
               joint_link_tree_[getLinkIndex("shin_l_active")]->joint_limit_lower_);
  out[0] = hip_roll_joint;
  out[1] = hip_pitch_joint;
  out[2] = thigh_pitch_joint;
  out[3] = shin_active_joint;
  out[4] = ankle_roll_joint;
  out[5] = ankle_yaw_joint;

  std::cout << "Inverse Kinematics result (Left Leg):" << std::endl;
  std::cout << "Joint ID: 20 (hip_l_roll), Angle: " << out[0] << std::endl;
  std::cout << "Joint ID: 21 (hip_l_pitch), Angle: " << out[1] << std::endl;
  std::cout << "Joint ID: 22 (thigh_l_front_active), Angle: " << out[2] << std::endl;
  std::cout << "Joint ID: 34 (shin_l_active), Angle: " << out[3] << std::endl;
  std::cout << "Joint ID: 26 (ankle_l_roll), Angle: " << out[4] << std::endl;
  std::cout << "Joint ID: 27 (ankle_l_yaw), Angle: " << out[5] << std::endl;

  return true;
}

LinkData* KurokoKinematics::getLinkData(const std::string& link_name)
{
  for (int ix = 0; ix <= ALL_JOINT_ID; ix++)
  {
    if (joint_link_tree_[ix]->name_ == link_name)
    {
      return joint_link_tree_[ix];
    }
  }

  return nullptr;
}

int KurokoKinematics::getLinkIndex(const std::string& link_name)
{
  for (int i = 0; i <= ALL_JOINT_ID; i++)
  {
    if (joint_link_tree_[i]->name_ == link_name)
    {
      return i;
    }
  }
  return -1;  // Not found
}

LinkData* KurokoKinematics::getLinkData(const int link_id)
{
  if (joint_link_tree_[link_id] != nullptr)
  {
    return joint_link_tree_[link_id];
  }

  return nullptr;
}

Eigen::MatrixXd KurokoKinematics::getJointAxis(const std::string& link_name)
{
  Eigen::MatrixXd joint_axis;

  LinkData* link_data = getLinkData(link_name);

  if (link_data != nullptr)
  {
    joint_axis = link_data->joint_axis_;
  }

  return joint_axis;
}

double KurokoKinematics::getJointDirection(const std::string& link_name)
{
  double joint_direction = 0.0;
  LinkData* link_data = getLinkData(link_name);

  if (link_data != nullptr)
  {
    joint_direction =
        link_data->joint_axis_.coeff(0, 0) + link_data->joint_axis_.coeff(1, 0) + link_data->joint_axis_.coeff(2, 0);
  }

  return joint_direction;
}

double KurokoKinematics::getJointDirection(const int link_id)
{
  double joint_direction = 0.0;
  LinkData* link_data = getLinkData(link_id);

  if (link_data != nullptr)
  {
    joint_direction =
        link_data->joint_axis_.coeff(0, 0) + link_data->joint_axis_.coeff(1, 0) + link_data->joint_axis_.coeff(2, 0);
  }

  return joint_direction;
}

Eigen::MatrixXd KurokoKinematics::calcPreviewParam(double preview_time, double control_cycle, double lipm_height,
                                                   const Eigen::MatrixXd& K, const Eigen::MatrixXd& P)
{
  double t = control_cycle;
  double preview_size = round(preview_time / control_cycle) + 1;

  Eigen::MatrixXd a;
  a.resize(3, 3);
  a << 1, t, t * t / 2.0, 0, 1, t, 0, 0, 1;

  Eigen::MatrixXd b;
  b.resize(3, 1);
  b << t * t * t / 6.0, t * t / 2.0, t;

  Eigen::MatrixXd c;
  c.resize(1, 3);
  c << 1, 0, -lipm_height / 9.81;

  Eigen::MatrixXd temp_a = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd tempb = Eigen::MatrixXd::Zero(4, 1);
  Eigen::MatrixXd tempc = Eigen::MatrixXd::Zero(1, 4);

  temp_a.coeffRef(0, 0) = 1;
  temp_a.block<1, 3>(0, 1) = c * a;
  temp_a.block<3, 3>(1, 1) = a;

  tempb.coeffRef(0, 0) = (c * b).coeff(0, 0);
  tempb.block<3, 1>(1, 0) = b;

  tempc.coeffRef(0, 0) = 1;

  double r = 1e-6;
  double q_e = 1;
  double q_x = 0;

  Eigen::MatrixXd q = Eigen::MatrixXd::Zero(4, 4);
  q.coeffRef(0, 0) = q_e;
  q.coeffRef(1, 1) = q_e;
  q.coeffRef(2, 2) = q_e;
  q.coeffRef(3, 3) = q_x;

  Eigen::MatrixXd f;
  f.resize(1, preview_size);

  Eigen::MatrixXd mat_r = Eigen::MatrixXd::Zero(1, 1);
  mat_r.coeffRef(0, 0) = r;

  Eigen::MatrixXd temp_coeff1 = mat_r + ((tempb.transpose() * P) * tempb);
  Eigen::MatrixXd temp_coeff1_inv = temp_coeff1.inverse();
  Eigen::MatrixXd temp_coeff2 = tempb.transpose();
  Eigen::MatrixXd temp_coeff3 = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd temp_coeff4 = P * tempc.transpose();

  f.block<1, 1>(0, 0) = ((temp_coeff1_inv * temp_coeff2) * temp_coeff3) * temp_coeff4;

  for (int i = 1; i < preview_size; i++)
  {
    temp_coeff3 = temp_coeff3 * ((temp_a - tempb * K).transpose());
    f.block<1, 1>(0, i) = ((temp_coeff1_inv * temp_coeff2) * temp_coeff3) * temp_coeff4;
  }

  return f;
}

}  // namespace motion_control
