#include "kuroko_kinematics/kuroko_kinematics.h"
#include <iostream>

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
    joint_link_pairs_[id] = new LinkData();

  if (tree == WHOLE_BODY)
  {
    joint_link_pairs_[0]->name_ = "base";
    joint_link_pairs_[0]->parent_ = -1;
    joint_link_pairs_[0]->sibling_ = -1;
    joint_link_pairs_[0]->child_ = 1;
    joint_link_pairs_[0]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[0]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[0]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[0]->joint_limit_lower_ = -100.0;
    joint_link_pairs_[0]->joint_limit_upper_ = 100.0;
    joint_link_pairs_[0]->link_mass_ = 0.0;
    joint_link_pairs_[0]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[0]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Body
    joint_link_pairs_[1]->name_ = "waist";
    joint_link_pairs_[1]->parent_ = 0;
    joint_link_pairs_[1]->sibling_ = -1;
    joint_link_pairs_[1]->child_ = 2;
    joint_link_pairs_[1]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[1]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[1]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[1]->joint_limit_lower_ = -100.0;
    joint_link_pairs_[1]->joint_limit_upper_ = 100.0;
    joint_link_pairs_[1]->link_mass_ = 0.337;
    joint_link_pairs_[1]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.00125, 0.0, -0.04875);
    joint_link_pairs_[1]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[2]->name_ = "chest";
    joint_link_pairs_[2]->parent_ = 1;
    joint_link_pairs_[2]->sibling_ = 11;  // TODO (hip_r_roll)
    joint_link_pairs_[2]->child_ = 3;
    joint_link_pairs_[2]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[2]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[2]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    joint_link_pairs_[2]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[2]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[2]->link_mass_ = 0.337;
    joint_link_pairs_[2]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.057);
    joint_link_pairs_[2]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    // Right Arm
    joint_link_pairs_[3]->name_ = "shoulder_r_pitch";
    joint_link_pairs_[3]->parent_ = 2;
    joint_link_pairs_[3]->sibling_ = 7;  // (shoulder_l_pitch)
    joint_link_pairs_[3]->child_ = 4;
    joint_link_pairs_[3]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, -0.1275, 0.01275);
    joint_link_pairs_[3]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[3]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[3]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[3]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[3]->link_mass_ = 0.022;
    joint_link_pairs_[3]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, -0.024, 0.0);
    joint_link_pairs_[3]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[4]->name_ = "shoulder_r_roll";
    joint_link_pairs_[4]->parent_ = 3;
    joint_link_pairs_[4]->sibling_ = -1;
    joint_link_pairs_[4]->child_ = 5;
    joint_link_pairs_[4]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, -0.1275, 0.01275);
    joint_link_pairs_[4]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[4]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    joint_link_pairs_[4]->joint_limit_lower_ = -1.0472;
    joint_link_pairs_[4]->joint_limit_upper_ = 2.0071;
    joint_link_pairs_[4]->link_mass_ = 0.342;
    joint_link_pairs_[4]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, -0.016, -0.02575);
    joint_link_pairs_[4]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[5]->name_ = "elbow_r_front_and_rear";
    joint_link_pairs_[5]->parent_ = 4;
    joint_link_pairs_[5]->sibling_ = -1;
    joint_link_pairs_[5]->child_ = 6;
    joint_link_pairs_[5]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.008, -0.052);
    joint_link_pairs_[5]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[5]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[5]->joint_limit_lower_ = -2.6180;
    joint_link_pairs_[5]->joint_limit_upper_ = 2.6180;
    joint_link_pairs_[5]->link_mass_ = 0.063;
    joint_link_pairs_[5]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0043, 0.0, -0.1);
    joint_link_pairs_[5]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[6]->name_ = "arm_r_end";
    joint_link_pairs_[6]->parent_ = 5;
    joint_link_pairs_[6]->sibling_ = -1;
    joint_link_pairs_[6]->child_ = -1;
    joint_link_pairs_[6]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.208);
    joint_link_pairs_[6]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[6]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[6]->joint_limit_lower_ = -100;
    joint_link_pairs_[6]->joint_limit_upper_ = 100;
    joint_link_pairs_[6]->link_mass_ = 0;
    joint_link_pairs_[6]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_pairs_[6]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Left Arm
    joint_link_pairs_[7]->name_ = "shoulder_l_pitch";
    joint_link_pairs_[7]->parent_ = 2;
    joint_link_pairs_[7]->sibling_ = -1;
    joint_link_pairs_[7]->child_ = 8;
    joint_link_pairs_[7]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.1275, 0.01275);
    joint_link_pairs_[7]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[7]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[7]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[7]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[7]->link_mass_ = 0.022;
    joint_link_pairs_[7]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.024, 0.0);
    joint_link_pairs_[7]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[8]->name_ = "shoulder_l_roll";
    joint_link_pairs_[8]->parent_ = 7;
    joint_link_pairs_[8]->sibling_ = -1;
    joint_link_pairs_[8]->child_ = 9;
    joint_link_pairs_[8]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.038, 0.0);
    joint_link_pairs_[8]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[8]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    joint_link_pairs_[8]->joint_limit_lower_ = -2.0071;
    joint_link_pairs_[8]->joint_limit_upper_ = 1.0472;
    joint_link_pairs_[8]->link_mass_ = 0.342;
    joint_link_pairs_[8]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.016, -0.02575);
    joint_link_pairs_[8]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[9]->name_ = "elbow_l_front_and_rear";
    joint_link_pairs_[9]->parent_ = 8;
    joint_link_pairs_[9]->sibling_ = -1;
    joint_link_pairs_[9]->child_ = 10;
    joint_link_pairs_[9]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, -0.008, -0.052);
    joint_link_pairs_[9]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[9]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[9]->joint_limit_lower_ = -2.6180;
    joint_link_pairs_[9]->joint_limit_upper_ = 2.6180;
    joint_link_pairs_[9]->link_mass_ = 0.063;
    joint_link_pairs_[9]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0043, 0.0, -0.104);
    joint_link_pairs_[9]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[10]->name_ = "arm_l_end";
    joint_link_pairs_[10]->parent_ = 9;
    joint_link_pairs_[10]->sibling_ = -1;
    joint_link_pairs_[10]->child_ = -1;
    joint_link_pairs_[10]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.208);
    joint_link_pairs_[10]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[10]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[10]->joint_limit_lower_ = -100;
    joint_link_pairs_[10]->joint_limit_upper_ = 100;
    joint_link_pairs_[10]->link_mass_ = 0;
    joint_link_pairs_[10]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_pairs_[10]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Right Leg
    joint_link_pairs_[11]->name_ = "hip_r_roll";
    joint_link_pairs_[11]->parent_ = 1;    // waist
    joint_link_pairs_[11]->sibling_ = 25;  // hip_l_roll
    joint_link_pairs_[11]->child_ = 12;
    joint_link_pairs_[11]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.08425);
    joint_link_pairs_[11]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[11]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    joint_link_pairs_[11]->joint_limit_lower_ = -0.5550;
    joint_link_pairs_[11]->joint_limit_upper_ = 1.8640;
    joint_link_pairs_[11]->link_mass_ = 0.026;
    joint_link_pairs_[11]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.00125, -0.0065, -0.004125);
    joint_link_pairs_[11]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[12]->name_ = "hip_r_pitch";
    joint_link_pairs_[12]->parent_ = 11;
    joint_link_pairs_[12]->sibling_ = -1;
    joint_link_pairs_[12]->child_ = 13;
    joint_link_pairs_[12]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, -0.027, 0.0);
    joint_link_pairs_[12]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[12]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[12]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[12]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[12]->link_mass_ = 0.478;
    joint_link_pairs_[12]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.00175, -0.049, -0.015);
    joint_link_pairs_[12]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[13]->name_ = "thigh_r_front_active";
    joint_link_pairs_[13]->parent_ = 12;
    joint_link_pairs_[13]->sibling_ = 20;  // shin_r_active
    joint_link_pairs_[13]->child_ = 14;
    joint_link_pairs_[13]->joint_position_ = robotis_framework::getTransitionXYZ(0.015, -0.02925, -0.0305);
    joint_link_pairs_[13]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, -0.785398163, 0.0);
    joint_link_pairs_[13]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[13]->joint_limit_lower_ = -0.4974;
    joint_link_pairs_[13]->joint_limit_upper_ = 1.8588;
    joint_link_pairs_[13]->link_mass_ = 0.012;
    joint_link_pairs_[13]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.05);
    joint_link_pairs_[13]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[14]->name_ = "knee_r_passive";
    joint_link_pairs_[14]->parent_ = 13;
    joint_link_pairs_[14]->sibling_ = -1;
    joint_link_pairs_[14]->child_ = 15;
    joint_link_pairs_[14]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.1);
    joint_link_pairs_[14]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.785398163, 0.0);
    joint_link_pairs_[14]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[14]->joint_limit_lower_ = -1.8588;
    joint_link_pairs_[14]->joint_limit_upper_ = 0.4974;
    joint_link_pairs_[14]->joint_mimic_ = "thigh_r_front_active";
    joint_link_pairs_[14]->joint_mimic_multiplier_ = -1.0;
    joint_link_pairs_[14]->link_mass_ = 0.007;
    joint_link_pairs_[14]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.01675, 0.0, 0.0);
    joint_link_pairs_[14]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[15]->name_ = "shin_r_front_passive";
    joint_link_pairs_[15]->parent_ = 14;
    joint_link_pairs_[15]->sibling_ = 24;
    joint_link_pairs_[15]->child_ = 16;
    joint_link_pairs_[15]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[15]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.785398163, 0.0);
    joint_link_pairs_[15]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[15]->joint_limit_lower_ = -1.8588;
    joint_link_pairs_[15]->joint_limit_upper_ = 0.4974;
    joint_link_pairs_[15]->joint_mimic_ = "shin_r_active";
    joint_link_pairs_[15]->joint_mimic_multiplier_ = 1.0;
    joint_link_pairs_[15]->link_mass_ = 0.013;
    joint_link_pairs_[15]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.05);
    joint_link_pairs_[15]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[16]->name_ = "ankle_r_pitch_passive";
    joint_link_pairs_[16]->parent_ = 15;
    joint_link_pairs_[16]->sibling_ = -1;
    joint_link_pairs_[16]->child_ = 17;
    joint_link_pairs_[16]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.1);
    joint_link_pairs_[16]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, -0.785398163, 0.0);
    joint_link_pairs_[16]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[16]->joint_limit_lower_ = -0.4974;
    joint_link_pairs_[16]->joint_limit_upper_ = 1.8588;
    joint_link_pairs_[16]->joint_mimic_ = "shin_r_active";
    joint_link_pairs_[16]->joint_mimic_multiplier_ = -1.0;
    joint_link_pairs_[16]->link_mass_ = 0.104;
    joint_link_pairs_[16]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.0285, 0, -0.013);
    joint_link_pairs_[16]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[17]->name_ = "ankle_r_roll";
    joint_link_pairs_[17]->parent_ = 16;
    joint_link_pairs_[17]->sibling_ = -1;
    joint_link_pairs_[17]->child_ = 18;
    joint_link_pairs_[17]->joint_position_ = robotis_framework::getTransitionXYZ(-0.015, 0, -0.027);
    joint_link_pairs_[17]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[17]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    joint_link_pairs_[17]->joint_limit_lower_ = -0.3704;
    joint_link_pairs_[17]->joint_limit_upper_ = 1.5708;
    joint_link_pairs_[17]->link_mass_ = 0.0136;
    joint_link_pairs_[17]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, -0.009375, -0.01175);
    joint_link_pairs_[17]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[18]->name_ = "ankle_r_yaw";
    joint_link_pairs_[18]->parent_ = 17;
    joint_link_pairs_[18]->sibling_ = -1;
    joint_link_pairs_[18]->child_ = 19;
    joint_link_pairs_[18]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.03825);
    joint_link_pairs_[18]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[18]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    joint_link_pairs_[18]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[18]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[18]->link_mass_ = 0.043;
    joint_link_pairs_[18]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.004);
    joint_link_pairs_[18]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[19]->name_ = "leg_r_end";
    joint_link_pairs_[19]->parent_ = 18;
    joint_link_pairs_[19]->sibling_ = -1;
    joint_link_pairs_[19]->child_ = -1;
    joint_link_pairs_[19]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.004);
    joint_link_pairs_[19]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[19]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[19]->joint_limit_lower_ = -100;
    joint_link_pairs_[19]->joint_limit_upper_ = 100;
    joint_link_pairs_[19]->link_mass_ = 0;
    joint_link_pairs_[19]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_pairs_[19]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_pairs_[20]->name_ = "shin_r_active";
    joint_link_pairs_[20]->parent_ = 12;   // hip_r_pitch
    joint_link_pairs_[20]->sibling_ = 23;  // thigh_r_middle_passive_link
    joint_link_pairs_[20]->child_ = 21;
    joint_link_pairs_[20]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0185, -0.02925, -0.0305);
    joint_link_pairs_[20]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 1.047197551, 0);
    joint_link_pairs_[20]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[20]->joint_limit_lower_ = -1.8588;
    joint_link_pairs_[20]->joint_limit_upper_ = 0.4974;
    joint_link_pairs_[20]->link_mass_ = 0.008;
    joint_link_pairs_[20]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.004745, 0, -0.015);
    joint_link_pairs_[20]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[21]->name_ = "thigh_r_rear_passive_mimic";
    joint_link_pairs_[21]->parent_ = 20;
    joint_link_pairs_[21]->sibling_ = -1;
    joint_link_pairs_[21]->child_ = 22;
    joint_link_pairs_[21]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.03);
    joint_link_pairs_[21]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, -1.832597551, 0);
    joint_link_pairs_[21]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[21]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[21]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[21]->joint_mimic_ = "shin_r_active";
    joint_link_pairs_[21]->joint_mimic_multiplier_ = -1.0;
    joint_link_pairs_[21]->link_mass_ = 0.0;
    joint_link_pairs_[21]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_pairs_[21]->link_inertia_ = robotis_framework::getInertiaXYZ(0, 0, 0, 0, 0, 0);

    joint_link_pairs_[22]->name_ = "thigh_r_rear_passive";
    joint_link_pairs_[22]->parent_ = 21;
    joint_link_pairs_[22]->sibling_ = -1;
    joint_link_pairs_[22]->child_ = 23;
    joint_link_pairs_[22]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_pairs_[22]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 0, 0);
    joint_link_pairs_[22]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[22]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[22]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[21]->joint_mimic_ = "thigh_r_front_active";
    joint_link_pairs_[21]->joint_mimic_multiplier_ = 1.0;
    joint_link_pairs_[22]->link_mass_ = 0.011;
    joint_link_pairs_[22]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_pairs_[22]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[23]->name_ = "thigh_r_middle_passive";
    joint_link_pairs_[23]->parent_ = 12;  // hip_r_pitch
    joint_link_pairs_[23]->sibling_ = -1;
    joint_link_pairs_[23]->child_ = 24;
    joint_link_pairs_[23]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0185, -0.02925, -0.0305);
    joint_link_pairs_[23]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, -0.785398163, 0);
    joint_link_pairs_[23]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[23]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[23]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[21]->joint_mimic_ = "thigh_r_front_active";
    joint_link_pairs_[21]->joint_mimic_multiplier_ = 1.0;
    joint_link_pairs_[23]->link_mass_ = 0.012;
    joint_link_pairs_[23]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_pairs_[23]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[24]->name_ = "shin_r_rear_passive";
    joint_link_pairs_[24]->parent_ = 14;  // knee_r_passive
    joint_link_pairs_[24]->sibling_ = -1;
    joint_link_pairs_[24]->child_ = 25;
    joint_link_pairs_[24]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0335, 0, 0);
    joint_link_pairs_[24]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 0.785398163, 0);
    joint_link_pairs_[24]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[24]->joint_limit_lower_ = -1.8588;
    joint_link_pairs_[24]->joint_limit_upper_ = 0.4974;
    joint_link_pairs_[24]->joint_mimic_ = "shin_r_active";
    joint_link_pairs_[24]->joint_mimic_multiplier_ = 1.0;
    joint_link_pairs_[24]->link_mass_ = 0.016;
    joint_link_pairs_[24]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_pairs_[24]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    // Left Leg
    joint_link_pairs_[25]->name_ = "hip_l_roll";
    joint_link_pairs_[25]->parent_ = 1;  // waist
    joint_link_pairs_[25]->sibling_ = -1;
    joint_link_pairs_[25]->child_ = 26;
    joint_link_pairs_[25]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.08425);
    joint_link_pairs_[25]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[25]->joint_axis_ = robotis_framework::getTransitionXYZ(-1.0, 0.0, 0.0);
    joint_link_pairs_[25]->joint_limit_lower_ = -0.5550;
    joint_link_pairs_[25]->joint_limit_upper_ = 1.8640;
    joint_link_pairs_[25]->link_mass_ = 0.026;
    joint_link_pairs_[25]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.00125, 0.0065, -0.004125);
    joint_link_pairs_[25]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[26]->name_ = "hip_l_pitch";
    joint_link_pairs_[26]->parent_ = 25;
    joint_link_pairs_[26]->sibling_ = -1;
    joint_link_pairs_[26]->child_ = 27;
    joint_link_pairs_[26]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.027, 0.0);
    joint_link_pairs_[26]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[26]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    joint_link_pairs_[26]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[26]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[26]->link_mass_ = 0.478;
    joint_link_pairs_[26]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.00175, 0.049, -0.015);
    joint_link_pairs_[26]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[27]->name_ = "thigh_l_front_active";
    joint_link_pairs_[27]->parent_ = 26;
    joint_link_pairs_[27]->sibling_ = 34;  // shin_l_active
    joint_link_pairs_[27]->child_ = 28;
    joint_link_pairs_[27]->joint_position_ = robotis_framework::getTransitionXYZ(0.015, 0.02925, -0.0305);
    joint_link_pairs_[27]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, -0.785398163, 0.0);
    joint_link_pairs_[27]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[27]->joint_limit_lower_ = -1.8588;
    joint_link_pairs_[27]->joint_limit_upper_ = 0.4974;
    joint_link_pairs_[27]->link_mass_ = 0.012;
    joint_link_pairs_[27]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.05);
    joint_link_pairs_[27]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[28]->name_ = "knee_l_passive";
    joint_link_pairs_[28]->parent_ = 27;
    joint_link_pairs_[28]->sibling_ = -1;
    joint_link_pairs_[28]->child_ = 29;
    joint_link_pairs_[28]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.1);
    joint_link_pairs_[28]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.785398163, 0.0);
    joint_link_pairs_[28]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[28]->joint_limit_lower_ = -0.4974;
    joint_link_pairs_[28]->joint_limit_upper_ = 1.8588;
    joint_link_pairs_[28]->joint_mimic_ = "thigh_l_front_active";
    joint_link_pairs_[28]->joint_mimic_multiplier_ = -1.0;
    joint_link_pairs_[28]->link_mass_ = 0.007;
    joint_link_pairs_[28]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.01675, 0.0, 0.0);
    joint_link_pairs_[28]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[29]->name_ = "shin_l_front_passive";
    joint_link_pairs_[29]->parent_ = 28;
    joint_link_pairs_[29]->sibling_ = 33;
    joint_link_pairs_[29]->child_ = 30;
    joint_link_pairs_[29]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[29]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.785398163, 0.0);
    joint_link_pairs_[29]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[29]->joint_limit_lower_ = -0.4974;
    joint_link_pairs_[29]->joint_limit_upper_ = 1.8588;
    joint_link_pairs_[29]->joint_mimic_ = "shin_l_active";
    joint_link_pairs_[29]->joint_mimic_multiplier_ = 1.0;
    joint_link_pairs_[29]->link_mass_ = 0.013;
    joint_link_pairs_[29]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.05);
    joint_link_pairs_[29]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[30]->name_ = "ankle_l_pitch_passive";
    joint_link_pairs_[30]->parent_ = 29;
    joint_link_pairs_[30]->sibling_ = -1;
    joint_link_pairs_[30]->child_ = 31;
    joint_link_pairs_[30]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.1);
    joint_link_pairs_[30]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, -0.785398163, 0.0);
    joint_link_pairs_[30]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[30]->joint_limit_lower_ = -1.8588;
    joint_link_pairs_[30]->joint_limit_upper_ = 0.4974;
    joint_link_pairs_[30]->joint_mimic_ = "shin_l_active";
    joint_link_pairs_[30]->joint_mimic_multiplier_ = -1.0;
    joint_link_pairs_[30]->link_mass_ = 0.104;
    joint_link_pairs_[30]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.0285, 0, -0.013);
    joint_link_pairs_[30]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[31]->name_ = "ankle_l_roll";
    joint_link_pairs_[31]->parent_ = 30;
    joint_link_pairs_[31]->sibling_ = -1;
    joint_link_pairs_[31]->child_ = 32;
    joint_link_pairs_[31]->joint_position_ = robotis_framework::getTransitionXYZ(-0.015, 0, -0.027);
    joint_link_pairs_[31]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[31]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    joint_link_pairs_[31]->joint_limit_lower_ = -1.5708;
    joint_link_pairs_[31]->joint_limit_upper_ = 0.3704;
    joint_link_pairs_[31]->link_mass_ = 0.0136;
    joint_link_pairs_[31]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0.009375, -0.01175);
    joint_link_pairs_[31]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[32]->name_ = "ankle_l_yaw";
    joint_link_pairs_[32]->parent_ = 31;
    joint_link_pairs_[32]->sibling_ = -1;
    joint_link_pairs_[32]->child_ = 33;
    joint_link_pairs_[32]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.03825);
    joint_link_pairs_[32]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[32]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    joint_link_pairs_[32]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[32]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[32]->link_mass_ = 0.043;
    joint_link_pairs_[32]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.004);
    joint_link_pairs_[32]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[33]->name_ = "leg_l_end";
    joint_link_pairs_[33]->parent_ = 32;
    joint_link_pairs_[33]->sibling_ = -1;
    joint_link_pairs_[33]->child_ = -1;
    joint_link_pairs_[33]->joint_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, -0.004);
    joint_link_pairs_[33]->joint_orientation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    joint_link_pairs_[33]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    joint_link_pairs_[33]->joint_limit_lower_ = -100;
    joint_link_pairs_[33]->joint_limit_upper_ = 100;
    joint_link_pairs_[33]->link_mass_ = 0;
    joint_link_pairs_[33]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_pairs_[33]->link_inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    joint_link_pairs_[34]->name_ = "shin_l_active";
    joint_link_pairs_[34]->parent_ = 27;  // thigh_l_front_active
    joint_link_pairs_[34]->sibling_ = -1;
    joint_link_pairs_[34]->child_ = 35;
    joint_link_pairs_[34]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0185, 0.02925, -0.0305);
    joint_link_pairs_[34]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 1.047197551, 0);
    joint_link_pairs_[34]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[34]->joint_limit_lower_ = -0.4974;
    joint_link_pairs_[34]->joint_limit_upper_ = 1.8588;
    joint_link_pairs_[34]->link_mass_ = 0.008;
    joint_link_pairs_[34]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(-0.004745, 0, -0.015);
    joint_link_pairs_[34]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[35]->name_ = "thigh_l_rear_passive_mimic";
    joint_link_pairs_[35]->parent_ = 34;
    joint_link_pairs_[35]->sibling_ = -1;
    joint_link_pairs_[35]->child_ = 36;
    joint_link_pairs_[35]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, -0.03);
    joint_link_pairs_[35]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, -1.832597551, 0);
    joint_link_pairs_[35]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[35]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[35]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[35]->joint_mimic_ = "shin_l_active";
    joint_link_pairs_[35]->joint_mimic_multiplier_ = -1.0;
    joint_link_pairs_[35]->link_mass_ = 0.0;
    joint_link_pairs_[35]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_pairs_[35]->link_inertia_ = robotis_framework::getInertiaXYZ(0, 0, 0, 0, 0, 0);

    joint_link_pairs_[36]->name_ = "thigh_l_rear_passive";
    joint_link_pairs_[36]->parent_ = 35;
    joint_link_pairs_[36]->sibling_ = -1;
    joint_link_pairs_[36]->child_ = 37;
    joint_link_pairs_[36]->joint_position_ = robotis_framework::getTransitionXYZ(0, 0, 0);
    joint_link_pairs_[36]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 0, 0);
    joint_link_pairs_[36]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[36]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[36]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[36]->joint_mimic_ = "thigh_l_front_active";
    joint_link_pairs_[36]->joint_mimic_multiplier_ = 1.0;
    joint_link_pairs_[36]->link_mass_ = 0.011;
    joint_link_pairs_[36]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_pairs_[36]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[37]->name_ = "thigh_l_middle_passive";
    joint_link_pairs_[37]->parent_ = 27;  // thigh_l_front_active
    joint_link_pairs_[37]->sibling_ = -1;
    joint_link_pairs_[37]->child_ = 38;
    joint_link_pairs_[37]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0185, 0.02925, -0.0305);
    joint_link_pairs_[37]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, -0.785398163, 0);
    joint_link_pairs_[37]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[37]->joint_limit_lower_ = -3.1;
    joint_link_pairs_[37]->joint_limit_upper_ = 3.1;
    joint_link_pairs_[37]->joint_mimic_ = "thigh_l_front_active";
    joint_link_pairs_[37]->joint_mimic_multiplier_ = 1.0;
    joint_link_pairs_[37]->link_mass_ = 0.012;
    joint_link_pairs_[37]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_pairs_[37]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);

    joint_link_pairs_[38]->name_ = "shin_l_rear_passive";
    joint_link_pairs_[38]->parent_ = 29;  // knee_l_passive
    joint_link_pairs_[38]->sibling_ = -1;
    joint_link_pairs_[38]->child_ = 39;
    joint_link_pairs_[38]->joint_position_ = robotis_framework::getTransitionXYZ(-0.0335, 0, 0);
    joint_link_pairs_[38]->joint_orientation_ = robotis_framework::convertRPYToRotation(0, 0.785398163, 0);
    joint_link_pairs_[38]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, -1.0, 0.0);
    joint_link_pairs_[38]->joint_limit_lower_ = -0.4974;
    joint_link_pairs_[38]->joint_limit_upper_ = 1.8588;
    joint_link_pairs_[38]->joint_mimic_ = "shin_l_active";
    joint_link_pairs_[38]->joint_mimic_multiplier_ = 1.0;
    joint_link_pairs_[38]->link_mass_ = 0.016;
    joint_link_pairs_[38]->link_center_of_mass_ = robotis_framework::getTransitionXYZ(0, 0, -0.05);
    joint_link_pairs_[38]->link_inertia_ = robotis_framework::getInertiaXYZ(0.01, 0.0, 0.0, 0.01, 0.0, 0.01);
  }

  leg_side_offset_m_ =
      2.0 * (std::fabs(joint_link_pairs_[getLinkIndex("hip_r_roll")]->joint_position_.coeff(1, 0) +
                       joint_link_pairs_[getLinkIndex("hip_r_pitch")]->joint_position_.coeff(1, 0) +
                       joint_link_pairs_[getLinkIndex("thigh_r_front_active")]->joint_position_.coeff(1, 0)));
  thigh_length_m_ = std::fabs(joint_link_pairs_[getLinkIndex("knee_r_passive")]->joint_position_.coeff(2, 0));
  calf_length_m_ = std::fabs(joint_link_pairs_[getLinkIndex("ankle_r_pitch_passive")]->joint_position_.coeff(2, 0));
  ankle_length_m_ = std::fabs(joint_link_pairs_[getLinkIndex("ankle_r_roll")]->joint_position_.coeff(2, 0) +
                              joint_link_pairs_[getLinkIndex("ankle_r_yaw")]->joint_position_.coeff(2, 0) +
                              joint_link_pairs_[getLinkIndex("leg_r_end")]->joint_position_.coeff(2, 0));
}

std::vector<int> KurokoKinematics::findRoute(int to)
{
  int id = joint_link_pairs_[to]->parent_;

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
  int id = joint_link_pairs_[to]->parent_;

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

double KurokoKinematics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = joint_link_pairs_[joint_id]->link_mass_ + calcTotalMass(joint_link_pairs_[joint_id]->sibling_) +
           calcTotalMass(joint_link_pairs_[joint_id]->child_);

  return mass;
}

Eigen::MatrixXd KurokoKinematics::calcMC(int joint_id)
{
  Eigen::MatrixXd mc(3, 1);

  if (joint_id == -1)
    mc = Eigen::MatrixXd::Zero(3, 1);
  else
  {
    mc = joint_link_pairs_[joint_id]->link_mass_ *
         (joint_link_pairs_[joint_id]->internal_orientation_ * joint_link_pairs_[joint_id]->link_center_of_mass_ +
          joint_link_pairs_[joint_id]->internal_position_);
    mc = mc + calcMC(joint_link_pairs_[joint_id]->sibling_) + calcMC(joint_link_pairs_[joint_id]->child_);
  }

  return mc;
}

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
    joint_link_pairs_[0]->internal_position_ = Eigen::MatrixXd::Zero(3, 1);
    joint_link_pairs_[0]->internal_orientation_ = joint_link_pairs_[0]->joint_orientation_;
  }

  if (joint_id != 0)
  {
    int parent = joint_link_pairs_[joint_id]->parent_;
    double joint_angle = joint_link_pairs_[joint_id]->internal_joint_angle_;

    // Apply mimic joint angle calculation if this joint mimics another joint
    if (!joint_link_pairs_[joint_id]->joint_mimic_.empty())
    {
      int mimic_id = getLinkIndex(joint_link_pairs_[joint_id]->joint_mimic_);
      if (mimic_id != -1)
      {
        joint_angle =
            joint_link_pairs_[mimic_id]->internal_joint_angle_ * joint_link_pairs_[joint_id]->joint_mimic_multiplier_;
      }
    }

    // Calculate the current joint's position and orientation based on its parent
    joint_link_pairs_[joint_id]->internal_position_ =
        joint_link_pairs_[parent]->internal_orientation_ * joint_link_pairs_[joint_id]->joint_position_ +
        joint_link_pairs_[parent]->internal_position_;

    joint_link_pairs_[joint_id]->internal_orientation_ =
        joint_link_pairs_[parent]->internal_orientation_ *
        joint_link_pairs_[joint_id]->joint_orientation_ *  // Apply joint_orientation_ directly
        robotis_framework::calcRodrigues(robotis_framework::calcHatto(joint_link_pairs_[joint_id]->joint_axis_),
                                         joint_angle);

    joint_link_pairs_[joint_id]->internal_transformation_.block<3, 1>(0, 3) =
        joint_link_pairs_[joint_id]->internal_position_;
    joint_link_pairs_[joint_id]->internal_transformation_.block<3, 3>(0, 0) =
        joint_link_pairs_[joint_id]->internal_orientation_;
  }

  // Recursively process sibling and child joints
  calcForwardKinematics(joint_link_pairs_[joint_id]->sibling_);
  calcForwardKinematics(joint_link_pairs_[joint_id]->child_);
}

Eigen::MatrixXd KurokoKinematics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = joint_link_pairs_[idx[end]]->internal_position_;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, idx_size);

  for (int id = 0; id < idx_size; id++)
  {
    int curr_id = idx[id];

    Eigen::MatrixXd tar_orientation =
        joint_link_pairs_[curr_id]->internal_orientation_ * joint_link_pairs_[curr_id]->joint_axis_;

    jacobian.block(0, id, 3, 1) =
        robotis_framework::calcCross(tar_orientation, tar_position - joint_link_pairs_[curr_id]->internal_position_);
    jacobian.block(3, id, 3, 1) = tar_orientation;
  }

  return jacobian;
}

Eigen::MatrixXd KurokoKinematics::calcJacobianCOM(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = joint_link_pairs_[idx[end]]->internal_position_;
  Eigen::MatrixXd jacobian_com = Eigen::MatrixXd::Zero(6, idx_size);

  for (int id = 0; id < idx_size; id++)
  {
    int curr_id = idx[id];
    double mass = calcTotalMass(curr_id);

    Eigen::MatrixXd og = calcMC(curr_id) / mass - joint_link_pairs_[curr_id]->internal_position_;
    Eigen::MatrixXd tar_orientation =
        joint_link_pairs_[curr_id]->internal_orientation_ * joint_link_pairs_[curr_id]->joint_axis_;

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

bool KurokoKinematics::calcInverseKinematics(int to, const Eigen::MatrixXd& tar_position,
                                             const Eigen::MatrixXd& tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  std::vector<int> idx = findRoute(to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = joint_link_pairs_[to]->internal_position_;
    Eigen::MatrixXd curr_orientation = joint_link_pairs_[to]->internal_orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inverse = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inverse * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_num = idx[id];
      joint_link_pairs_[joint_num]->internal_joint_angle_ += delta_angle.coeff(id);
    }

    // Recalculate forward kinematics with updated joint angles
    calcForwardKinematics(0);
  }

  // Check joint limits to ensure calculated angles are within allowable range
  for (int joint_num : idx)
  {
    if (joint_link_pairs_[joint_num]->internal_joint_angle_ >= joint_link_pairs_[joint_num]->joint_limit_upper_)
    {
      limit_success = false;
      break;
    }
    else if (joint_link_pairs_[joint_num]->internal_joint_angle_ <= joint_link_pairs_[joint_num]->joint_limit_lower_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  return ik_success && limit_success;
}

bool KurokoKinematics::calcInverseKinematics(int from, int to, const Eigen::MatrixXd& tar_position,
                                             const Eigen::MatrixXd& tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  std::vector<int> idx = findRoute(from, to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = joint_link_pairs_[to]->internal_position_;
    Eigen::MatrixXd curr_orientation = joint_link_pairs_[to]->internal_orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inv = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_num = idx[id];
      joint_link_pairs_[joint_num]->internal_joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  for (int joint_num : idx)
  {
    if (joint_link_pairs_[joint_num]->internal_joint_angle_ >= joint_link_pairs_[joint_num]->joint_limit_upper_)
    {
      limit_success = false;
      break;
    }
    else if (joint_link_pairs_[joint_num]->internal_joint_angle_ <= joint_link_pairs_[joint_num]->joint_limit_lower_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  return ik_success && limit_success;
}

bool KurokoKinematics::calcInverseKinematics(int to, const Eigen::MatrixXd& tar_position,
                                             const Eigen::MatrixXd& tar_orientation, int max_iter, double ik_err,
                                             const Eigen::MatrixXd& weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(), idx.size());

  for (int ix = 0; ix < idx.size(); ix++)
    weight_matrix.coeffRef(ix, ix) = weight.coeff(idx[ix], 0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double r_damping = 1e-5;

  for (int ix = 0; ix < 3; ix++)
  {
    eval.coeffRef(ix, ix) = p_damping;
    eval.coeffRef(ix + 3, ix + 3) = r_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = joint_link_pairs_[to]->internal_position_;
    Eigen::MatrixXd curr_orientation = joint_link_pairs_[to]->internal_orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_id = idx[id];
      joint_link_pairs_[joint_id]->internal_joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  /* check joint limit */
  for (int joint_num : idx)
  {
    if (joint_link_pairs_[joint_num]->internal_joint_angle_ >= joint_link_pairs_[joint_num]->joint_limit_upper_)
    {
      limit_success = false;
      break;
    }
    else if (joint_link_pairs_[joint_num]->internal_joint_angle_ <= joint_link_pairs_[joint_num]->joint_limit_lower_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  return ik_success && limit_success;
}

bool KurokoKinematics::calcInverseKinematics(int from, int to, const Eigen::MatrixXd& tar_position,
                                             const Eigen::MatrixXd& tar_orientation, int max_iter, double ik_err,
                                             const Eigen::MatrixXd& weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(), idx.size());

  for (int ix = 0; ix < idx.size(); ix++)
    weight_matrix.coeffRef(ix, ix) = weight.coeff(idx[ix], 0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double r_damping = 1e-5;

  for (int ix = 0; ix < 3; ix++)
  {
    eval.coeffRef(ix, ix) = p_damping;
    eval.coeffRef(ix + 3, ix + 3) = r_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);
    Eigen::MatrixXd curr_position = joint_link_pairs_[to]->internal_position_;
    Eigen::MatrixXd curr_orientation = joint_link_pairs_[to]->internal_orientation_;
    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();
    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_id = idx[id];
      joint_link_pairs_[joint_id]->internal_joint_angle_ += delta_angle.coeff(id);
    }
    calcForwardKinematics(0);
  }

  /* check joint limit */
  for (int joint_num : idx)
  {
    if (joint_link_pairs_[joint_num]->internal_joint_angle_ >= joint_link_pairs_[joint_num]->joint_limit_upper_)
    {
      limit_success = false;
      break;
    }
    else if (joint_link_pairs_[joint_num]->internal_joint_angle_ <= joint_link_pairs_[joint_num]->joint_limit_lower_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  return ik_success && limit_success;
}

bool KurokoKinematics::calcInverseKinematicsForLeg(double* out, double x, double y, double z, double roll, double pitch,
                                                   double yaw)
{
  Eigen::Matrix4d trans_ad, trans_da, trans_cd, trans_dc, trans_ac;
  Eigen::Vector3d vec;

  bool invertible;
  double rac, arc_cos, arc_tan, k, l, m, n, s, c, theta;
  double thigh_length = thigh_length_m_;
  double calf_length = calf_length_m_;
  double ankle_length = ankle_length_m_;

  // Transformation from the base to the desired position
  trans_ad = robotis_framework::getTransformationXYZRPY(x, y, z, roll, pitch, yaw);

  // Adjusting the target position considering the ankle length
  vec.coeffRef(0) = trans_ad.coeff(0, 3) + trans_ad.coeff(0, 2) * ankle_length;
  vec.coeffRef(1) = trans_ad.coeff(1, 3) + trans_ad.coeff(1, 2) * ankle_length;
  vec.coeffRef(2) = trans_ad.coeff(2, 3) + trans_ad.coeff(2, 2) * ankle_length;

  // Step 1: Calculate the knee pitch angle
  rac = vec.norm();
  arc_cos =
      acos((rac * rac - thigh_length * thigh_length - calf_length * calf_length) / (2.0 * thigh_length * calf_length));
  if (std::isnan(arc_cos))
    return false;
  *(out + 3) = arc_cos;  // Knee pitch angle

  // Step 2: Calculate the ankle roll angle
  trans_ad.computeInverseWithCheck(trans_da, invertible);
  if (!invertible)
    return false;

  k = sqrt(trans_da.coeff(1, 3) * trans_da.coeff(1, 3) + trans_da.coeff(2, 3) * trans_da.coeff(2, 3));
  l = sqrt(trans_da.coeff(1, 3) * trans_da.coeff(1, 3) +
           (trans_da.coeff(2, 3) - ankle_length) * (trans_da.coeff(2, 3) - ankle_length));
  m = (k * k - l * l - ankle_length * ankle_length) / (2.0 * l * ankle_length);

  if (m > 1.0)
    m = 1.0;
  else if (m < -1.0)
    m = -1.0;
  arc_cos = acos(m);

  if (std::isnan(arc_cos))
    return false;

  if (trans_da.coeff(1, 3) < 0.0)
    *(out + 5) = -arc_cos;
  else
    *(out + 5) = arc_cos;  // Ankle roll angle

  // Step 3: Calculate the hip yaw angle
  trans_cd = robotis_framework::getTransformationXYZRPY(0, 0, -ankle_length, *(out + 5), 0, 0);
  trans_cd.computeInverseWithCheck(trans_dc, invertible);
  if (!invertible)
    return false;

  trans_ac = trans_ad * trans_dc;
  arc_tan = atan2(-trans_ac.coeff(0, 1), trans_ac.coeff(1, 1));
  if (std::isinf(arc_tan))
    return false;
  *(out) = arc_tan;  // Hip yaw angle

  // Step 4: Calculate the hip roll angle
  arc_tan = atan2(trans_ac.coeff(2, 1), -trans_ac.coeff(0, 1) * sin(*(out)) + trans_ac.coeff(1, 1) * cos(*(out)));
  if (std::isinf(arc_tan))
    return false;
  *(out + 1) = arc_tan;  // Hip roll angle

  // Step 5: Calculate the hip pitch and ankle pitch angles
  arc_tan = atan2(trans_ac.coeff(0, 2) * cos(*(out)) + trans_ac.coeff(1, 2) * sin(*(out)),
                  trans_ac.coeff(0, 0) * cos(*(out)) + trans_ac.coeff(1, 0) * sin(*(out)));
  if (std::isinf(arc_tan))
    return false;
  theta = arc_tan;
  k = sin(*(out + 3)) * calf_length;
  l = -thigh_length - cos(*(out + 3)) * calf_length;
  m = cos(*(out)) * vec.coeff(0) + sin(*(out)) * vec.coeff(1);
  n = cos(*(out + 1)) * vec.coeff(2) + sin(*(out)) * sin(*(out + 1)) * vec.coeff(0) -
      cos(*(out)) * sin(*(out + 1)) * vec.coeff(1);
  s = (k * n + l * m) / (k * k + l * l);
  c = (n - k * s) / l;
  arc_tan = atan2(s, c);
  if (std::isinf(arc_tan))
    return false;
  *(out + 2) = arc_tan;                          // Hip pitch angle
  *(out + 4) = theta - *(out + 3) - *(out + 2);  // Ankle pitch angle

  return true;
}

bool KurokoKinematics::calcInverseKinematicsForRightLeg(double* out, double x, double y, double z, double roll,
                                                        double pitch, double yaw)
{
  if (calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw))
  {
    for (int ix = 0; ix < 6; ix++)
      out[ix] *= getJointDirection(ID_R_LEG_START + 2 * ix);

    return true;
  }
  else
    return false;
}

bool KurokoKinematics::calcInverseKinematicsForLeftLeg(double* out, double x, double y, double z, double roll,
                                                       double pitch, double yaw)
{
  if (calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw))
  {
    for (int ix = 0; ix < 6; ix++)
      out[ix] *= getJointDirection(ID_L_LEG_START + 2 * ix);

    return true;
  }
  else
    return false;
}

LinkData* KurokoKinematics::getLinkData(const std::string& link_name)
{
  for (int ix = 0; ix <= ALL_JOINT_ID; ix++)
  {
    if (joint_link_pairs_[ix]->name_ == link_name)
    {
      return joint_link_pairs_[ix];
    }
  }

  return nullptr;
}

int KurokoKinematics::getLinkIndex(const std::string& link_name)
{
  for (int i = 0; i <= ALL_JOINT_ID; i++)
  {
    if (joint_link_pairs_[i]->name_ == link_name)
    {
      return i;
    }
  }
  return -1;  // Not found
}

LinkData* KurokoKinematics::getLinkData(const int link_id)
{
  if (joint_link_pairs_[link_id] != nullptr)
  {
    return joint_link_pairs_[link_id];
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
