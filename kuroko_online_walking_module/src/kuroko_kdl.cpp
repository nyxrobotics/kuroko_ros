#include <stdio.h>
#include "kuroko_online_walking_module/kuroko_kdl.h"
#include <utility>

KurokoKinematics::KurokoKinematics()
{
  rleg_joint_position_.resize(LEG_JOINT_NUM);

  for (int i = 0; i < LEG_JOINT_NUM; i++)
    rleg_joint_position_(i) = 0.0;
}

KurokoKinematics::~KurokoKinematics()
{
}

void KurokoKinematics::initialize(const Eigen::MatrixXd& pelvis_position, const Eigen::MatrixXd& pelvis_orientation)
{
  KDL::Chain rleg_chain, lleg_chain;

  double pelvis_x = pelvis_position.coeff(0, 0);
  double pelvis_y = pelvis_position.coeff(1, 0);
  double pelvis_z = pelvis_position.coeff(2, 0);

  double pelvis_xx = pelvis_orientation.coeff(0, 0);
  double pelvis_yx = pelvis_orientation.coeff(0, 1);
  double pelvis_zx = pelvis_orientation.coeff(0, 2);

  double pelvis_xy = pelvis_orientation.coeff(1, 0);
  double pelvis_yy = pelvis_orientation.coeff(1, 1);
  double pelvis_zy = pelvis_orientation.coeff(1, 2);

  double pelvis_xz = pelvis_orientation.coeff(2, 0);
  double pelvis_yz = pelvis_orientation.coeff(2, 1);
  double pelvis_zz = pelvis_orientation.coeff(2, 2);

  // Set Kinematics Tree

  // Initialize Chain
  rleg_chain_ = KDL::Chain();
  std::vector<double> rleg_limit_min;
  std::vector<double> rleg_limit_max;
  std::vector<bool> rleg_joint_direction;
  std::vector<double> rleg_joint_offset;

  lleg_chain_ = KDL::Chain();
  std::vector<double> lleg_limit_min;
  std::vector<double> lleg_limit_max;
  std::vector<double> min_position_limit, max_position_limit;

  // Right Leg Chain
  rleg_chain_.addSegment(KDL::Segment(
      "base", KDL::Joint(KDL::Joint::None),
      KDL::Frame(KDL::Rotation(pelvis_xx, pelvis_yx, pelvis_zx, pelvis_xy, pelvis_yy, pelvis_zy, pelvis_xz, pelvis_yz,
                               pelvis_zz),
                 KDL::Vector(pelvis_x, pelvis_y, pelvis_z)),
      KDL::RigidBodyInertia(0.0, KDL::Vector(0.0, 0.0, 0.0), KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("waist", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0, 0, -0.08425)),
                                      KDL::RigidBodyInertia(0.337, KDL::Vector(0.00125, 0.0, -0.04875),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("hip_r_roll", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0, -0.027, 0)),
                                      KDL::RigidBodyInertia(0.026, KDL::Vector(0.00125, -0.0065, -0.004125),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("hip_r_pitch", KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.015, -0.02925, -0.0305)),
                                      KDL::RigidBodyInertia(0.478, KDL::Vector(-0.00175, -0.049, -0.015),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("thigh_r_active", KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, -0.1)),
                                      KDL::RigidBodyInertia(0.0012, KDL::Vector(0.0, 0.0, -0.05),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("knee_r_passive", KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                      KDL::RigidBodyInertia(0.007, KDL::Vector(-0.01675, 0.0, 0.0),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment(
      "shin_r_front_passive", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, -0.1)),
      KDL::RigidBodyInertia(0.013, KDL::Vector(.0, 0.0, -0.05), KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("ankle_r_pitch_passive", KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(-0.015, 0, -0.027)),
                                      KDL::RigidBodyInertia(0.104, KDL::Vector(-0.0285, 0, -0.013),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("ankle_r_roll", KDL::Joint(KDL::Joint::RotX),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, -0.03825)),
                                      KDL::RigidBodyInertia(0.0136, KDL::Vector(0, -0.009375, -0.01175),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("ankle_r_yaw", KDL::Joint(KDL::Joint::RotZ),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, -0.004)),
                                      KDL::RigidBodyInertia(0.043, KDL::Vector(0.0, 0.0, -0.004),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment(
      "leg_r_end", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
      KDL::RigidBodyInertia(0.0, KDL::Vector(0.0, 0.0, 0.0), KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));

  // Left Leg Chain
  rleg_chain_.addSegment(KDL::Segment(
      "base", KDL::Joint(KDL::Joint::None),
      KDL::Frame(KDL::Rotation(pelvis_xx, pelvis_yx, pelvis_zx, pelvis_xy, pelvis_yy, pelvis_zy, pelvis_xz, pelvis_yz,
                               pelvis_zz),
                 KDL::Vector(pelvis_x, pelvis_y, pelvis_z)),
      KDL::RigidBodyInertia(0.0, KDL::Vector(0.0, 0.0, 0.0), KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("waist", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0, 0, -0.08425)),
                                      KDL::RigidBodyInertia(0.337, KDL::Vector(0.00125, 0.0, -0.04875),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("hip_l_roll", KDL::Joint(KDL::Joint::RotX), KDL::Frame(KDL::Vector(0, 0.027, 0)),
                                      KDL::RigidBodyInertia(0.026, KDL::Vector(0.00125, 0.0065, -0.004125),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("hip_l_pitch", KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.015, 0.02925, -0.0305)),
                                      KDL::RigidBodyInertia(0.478, KDL::Vector(-0.00175, 0.049, -0.015),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("thigh_l_active", KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, -0.1)),
                                      KDL::RigidBodyInertia(0.0012, KDL::Vector(0.0, 0.0, -0.05),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("knee_l_passive", KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                      KDL::RigidBodyInertia(0.007, KDL::Vector(-0.01675, 0.0, 0.0),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment(
      "shin_r_front_passive", KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, -0.1)),
      KDL::RigidBodyInertia(0.013, KDL::Vector(.0, 0.0, -0.05), KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("ankle_l_pitch_passive", KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(-0.015, 0, -0.027)),
                                      KDL::RigidBodyInertia(0.104, KDL::Vector(-0.0285, 0, -0.013),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("ankle_l_roll", KDL::Joint(KDL::Joint::RotX),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, -0.03825)),
                                      KDL::RigidBodyInertia(0.0136, KDL::Vector(0, -0.009375, -0.01175),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment("ankle_l_yaw", KDL::Joint(KDL::Joint::RotZ),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, -0.004)),
                                      KDL::RigidBodyInertia(0.043, KDL::Vector(0.0, 0.0, -0.004),
                                                            KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));
  rleg_chain_.addSegment(KDL::Segment(
      "leg_l_end", KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
      KDL::RigidBodyInertia(0.0, KDL::Vector(0.0, 0.0, 0.0), KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))));

  // Set Joint Limits
  min_position_limit.push_back(-180.0);
  max_position_limit.push_back(180.0);  // r_leg_hip_y
  min_position_limit.push_back(-180.0);
  max_position_limit.push_back(180.0);  // hip_r_roll
  min_position_limit.push_back(-180.0);
  max_position_limit.push_back(180.0);  // hip_r_pitch
  min_position_limit.push_back(-180.0);
  max_position_limit.push_back(180.0);  // knee_r_passive
  min_position_limit.push_back(-180.0);
  max_position_limit.push_back(180.0);  // r_leg_an_p
  min_position_limit.push_back(-180.0);
  max_position_limit.push_back(180.0);  // ankle_r_roll

  KDL::JntArray min_joint_position_limit(LEG_JOINT_NUM), max_joint_position_limit(LEG_JOINT_NUM);
  for (int index = 0; index < LEG_JOINT_NUM; index++)
  {
    min_joint_position_limit(index) = min_position_limit[index] * D2R;
    max_joint_position_limit(index) = max_position_limit[index] * D2R;
  }

  /* KDL Solver Initialization */
  //  rleg_dyn_param_ = new KDL::ChainDynParam(rleg_chain_, KDL::Vector(0.0,
  //  0.0, -9.81)); // kinematics & dynamics parameter rleg_jacobian_solver_ =
  //  new KDL::ChainJntToJacSolver(rleg_chain_); // jabocian solver
  rleg_fk_solver_ = new KDL::ChainFkSolverPos_recursive(rleg_chain_);  // forward kinematics solver

  // inverse kinematics solver
  rleg_ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(rleg_chain_);
  rleg_ik_pos_solver_ = new KDL::ChainIkSolverPos_NR_JL(rleg_chain_, min_joint_position_limit, max_joint_position_limit,
                                                        *rleg_fk_solver_, *rleg_ik_vel_solver_);

  //  lleg_dyn_param_ = new KDL::ChainDynParam(lleg_chain_, KDL::Vector(0.0,
  //  0.0, -9.81)); // kinematics & dynamics parameter lleg_jacobian_solver_ =
  //  new KDL::ChainJntToJacSolver(lleg_chain_); // jabocian solver
  lleg_fk_solver_ = new KDL::ChainFkSolverPos_recursive(lleg_chain_);  // forward kinematics solver

  // inverse kinematics solver
  lleg_ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(lleg_chain_);
  lleg_ik_pos_solver_ = new KDL::ChainIkSolverPos_NR_JL(lleg_chain_, min_joint_position_limit, max_joint_position_limit,
                                                        *lleg_fk_solver_, *lleg_ik_vel_solver_);
}

void KurokoKinematics::setJointPosition(Eigen::VectorXd rleg_joint_position, Eigen::VectorXd lleg_joint_position)
{
  rleg_joint_position_ = std::move(rleg_joint_position);
  lleg_joint_position_ = std::move(lleg_joint_position);

  //  for (int i=0; i<LEG_JOINT_NUM; i++)
  //    ROS_INFO("rleg_joint_position_(%d): %f", i, rleg_joint_position_(i));

  //  for (int i=0; i<LEG_JOINT_NUM; i++)
  //    ROS_INFO("lleg_joint_position_(%d): %f", i, lleg_joint_position_(i));
}

void KurokoKinematics::solveForwardKinematics(std::vector<double_t>& rleg_position,
                                              std::vector<double_t>& rleg_orientation,
                                              std::vector<double_t>& lleg_position,
                                              std::vector<double_t>& lleg_orientation)
{
  // rleg
  KDL::JntArray rleg_joint_position;
  rleg_joint_position.data = rleg_joint_position_;

  KDL::Frame rleg_pose;
  rleg_fk_solver_->JntToCart(rleg_joint_position, rleg_pose);

  rleg_pose_.position.x = rleg_pose.p.x();
  rleg_pose_.position.y = rleg_pose.p.y();
  rleg_pose_.position.z = rleg_pose.p.z();

  rleg_pose.M.GetQuaternion(rleg_pose_.orientation.x, rleg_pose_.orientation.y, rleg_pose_.orientation.z,
                            rleg_pose_.orientation.w);

  //  ROS_INFO("rleg position x : %f y: %f, z: %f", rleg_pose_.position.x,
  //  rleg_pose_.position.y, rleg_pose_.position.z);

  rleg_position.resize(3, 0.0);
  rleg_position[0] = rleg_pose_.position.x;
  rleg_position[1] = rleg_pose_.position.y;
  rleg_position[2] = rleg_pose_.position.z;

  rleg_orientation.resize(4, 0.0);
  rleg_orientation[0] = rleg_pose_.orientation.x;
  rleg_orientation[1] = rleg_pose_.orientation.y;
  rleg_orientation[2] = rleg_pose_.orientation.z;
  rleg_orientation[3] = rleg_pose_.orientation.w;

  // lleg
  KDL::JntArray lleg_joint_position;
  lleg_joint_position.data = lleg_joint_position_;

  KDL::Frame lleg_pose;
  lleg_fk_solver_->JntToCart(lleg_joint_position, lleg_pose);

  lleg_pose_.position.x = lleg_pose.p.x();
  lleg_pose_.position.y = lleg_pose.p.y();
  lleg_pose_.position.z = lleg_pose.p.z();

  lleg_pose.M.GetQuaternion(lleg_pose_.orientation.x, lleg_pose_.orientation.y, lleg_pose_.orientation.z,
                            lleg_pose_.orientation.w);

  //  ROS_INFO("lleg position x : %f y: %f, z: %f", lleg_pose_.position.x,
  //  lleg_pose_.position.y, lleg_pose_.position.z);

  lleg_position.resize(3, 0.0);
  lleg_position[0] = lleg_pose_.position.x;
  lleg_position[1] = lleg_pose_.position.y;
  lleg_position[2] = lleg_pose_.position.z;

  lleg_orientation.resize(4, 0.0);
  lleg_orientation[0] = lleg_pose_.orientation.x;
  lleg_orientation[1] = lleg_pose_.orientation.y;
  lleg_orientation[2] = lleg_pose_.orientation.z;
  lleg_orientation[3] = lleg_pose_.orientation.w;
}

bool KurokoKinematics::solveInverseKinematics(std::vector<double_t>& rleg_output,
                                              const Eigen::MatrixXd& rleg_target_position,
                                              Eigen::Quaterniond rleg_target_orientation,
                                              std::vector<double_t>& lleg_output,
                                              const Eigen::MatrixXd& lleg_target_position,
                                              Eigen::Quaterniond lleg_target_orientation)
{
  //  ROS_INFO("right x: %f, y: %f, z: %f", rleg_target_position(0),
  //  rleg_target_position(1), rleg_target_position(2)); ROS_INFO("left x: %f,
  //  y: %f, z: %f", lleg_target_position(0), lleg_target_position(1),
  //  lleg_target_position(2));

  // rleg
  KDL::JntArray rleg_joint_position;
  rleg_joint_position.data = rleg_joint_position_;

  KDL::Frame rleg_desired_pose;
  rleg_desired_pose.p.x(rleg_target_position.coeff(0, 0));
  rleg_desired_pose.p.y(rleg_target_position.coeff(1, 0));
  rleg_desired_pose.p.z(rleg_target_position.coeff(2, 0));

  rleg_desired_pose.M = KDL::Rotation::Quaternion(rleg_target_orientation.x(), rleg_target_orientation.y(),
                                                  rleg_target_orientation.z(), rleg_target_orientation.w());

  KDL::JntArray rleg_desired_joint_position;
  rleg_desired_joint_position.resize(LEG_JOINT_NUM);

  int rleg_err = rleg_ik_pos_solver_->CartToJnt(rleg_joint_position, rleg_desired_pose, rleg_desired_joint_position);

  if (rleg_err < 0)
  {
    ROS_WARN("RLEG IK ERR : %d", rleg_err);
    return false;
  }

  // lleg
  KDL::JntArray lleg_joint_position;
  lleg_joint_position.data = lleg_joint_position_;

  KDL::Frame lleg_desired_pose;
  lleg_desired_pose.p.x(lleg_target_position.coeff(0, 0));
  lleg_desired_pose.p.y(lleg_target_position.coeff(1, 0));
  lleg_desired_pose.p.z(lleg_target_position.coeff(2, 0));

  lleg_desired_pose.M = KDL::Rotation::Quaternion(lleg_target_orientation.x(), lleg_target_orientation.y(),
                                                  lleg_target_orientation.z(), lleg_target_orientation.w());

  KDL::JntArray lleg_desired_joint_position;
  lleg_desired_joint_position.resize(LEG_JOINT_NUM);

  int lleg_err = lleg_ik_pos_solver_->CartToJnt(lleg_joint_position, lleg_desired_pose, lleg_desired_joint_position);

  if (lleg_err < 0)
  {
    ROS_WARN("LLEG IK ERR : %d", lleg_err);
    return false;
  }

  // output
  rleg_output.resize(LEG_JOINT_NUM);
  lleg_output.resize(LEG_JOINT_NUM);

  for (int i = 0; i < LEG_JOINT_NUM; i++)
  {
    rleg_output[i] = rleg_desired_joint_position(i);
    lleg_output[i] = lleg_desired_joint_position(i);
  }

  return true;
}

void KurokoKinematics::finalize()
{
  //  delete rleg_chain_;
  //  delete rleg_dyn_param_;
  //  delete rleg_jacobian_solver_;
  delete rleg_fk_solver_;
  delete rleg_ik_vel_solver_;
  delete rleg_ik_pos_solver_;

  //  delete lleg_chain_;
  //  delete lleg_dyn_param_;
  //  delete lleg_jacobian_solver_;
  delete lleg_fk_solver_;
  delete lleg_ik_vel_solver_;
  delete lleg_ik_pos_solver_;
}
