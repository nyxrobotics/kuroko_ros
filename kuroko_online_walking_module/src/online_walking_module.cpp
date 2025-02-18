#include <utility>
#include <vector>
#include "robotis_math/robotis_linear_algebra.h"
#include "kuroko_online_walking_module/online_walking_module.h"
using namespace motion_control;

OnlineWalkingModule::OnlineWalkingModule()
  : control_cycle_sec_(0.008)
  , is_moving_(false)
  , is_balancing_(false)
  , is_offset_updating_(false)
  , goal_initialize_(false)
  , balance_control_initialize_(false)
  , body_offset_initialize_(false)
  , joint_control_initialize_(false)
  , wholebody_initialize_(false)
  , walking_initialize_(false)
  , is_foot_step_2d_(false)
  , walking_phase_(DSP)
  , total_mass_(3.5)
  , foot_distance_(0.07)
{
  enable_ = false;
  module_name_ = "online_walking_module";
  control_mode_ = robotis_framework::PositionControl;
  control_type_ = NONE;
  balance_type_ = OFF;

  kuroko_kinematics_ = new KurokoKinematics();

  // Buffer size
  number_of_joints_ = 12;

  curr_joint_accel_.resize(number_of_joints_, 0.0);
  curr_joint_vel_.resize(number_of_joints_, 0.0);
  curr_joint_pos_.resize(number_of_joints_, 0.0);

  des_joint_accel_.resize(number_of_joints_, 0.0);
  des_joint_vel_.resize(number_of_joints_, 0.0);
  des_joint_pos_.resize(number_of_joints_, 0.0);

  goal_joint_accel_.resize(number_of_joints_, 0.0);
  goal_joint_vel_.resize(number_of_joints_, 0.0);
  goal_joint_pos_.resize(number_of_joints_, 0.0);

  des_joint_feedback_.resize(number_of_joints_, 0.0);
  des_joint_feedforward_.resize(number_of_joints_, 0.0);
  des_joint_pos_to_robot_.resize(number_of_joints_, 0.0);

  joint_feedforward_gain_.resize(number_of_joints_, 0.0);

  // body position default
  des_body_position_.resize(3, 0.0);
  des_body_velocity_.resize(3, 0.0);
  des_body_accel_.resize(3, 0.0);
  des_body_rpy_.resize(3, 0.0);

  // left foot position default
  des_l_leg_position_.resize(3, 0.0);
  des_l_leg_velocity_.resize(3, 0.0);
  des_l_leg_accel_.resize(3, 0.0);
  des_l_leg_rpy_.resize(3, 0.0);

  // right foot position default
  des_r_leg_position_.resize(3, 0.0);
  des_r_leg_velocity_.resize(3, 0.0);
  des_r_leg_accel_.resize(3, 0.0);
  des_r_leg_rpy_.resize(3, 0.0);

  x_lipm_.resize(3, 0.0);
  y_lipm_.resize(3, 0.0);

  des_balance_gain_ratio_.resize(1, 0.0);
  goal_balance_gain_ratio_.resize(1, 0.0);

  // Body Offset
  des_body_offset_.resize(3, 0.0);
  goal_body_offset_.resize(3, 0.0);

  /* leg */
  result_["ankle_r_yaw"] = new robotis_framework::DynamixelState();
  result_["hip_r_roll"] = new robotis_framework::DynamixelState();
  result_["hip_r_pitch"] = new robotis_framework::DynamixelState();
  result_["thigh_r_active"] = new robotis_framework::DynamixelState();
  result_["shin_r_active"] = new robotis_framework::DynamixelState();
  result_["ankle_r_roll"] = new robotis_framework::DynamixelState();

  result_["ankle_l_yaw"] = new robotis_framework::DynamixelState();
  result_["hip_l_roll"] = new robotis_framework::DynamixelState();
  result_["hip_l_pitch"] = new robotis_framework::DynamixelState();
  result_["thigh_l_active"] = new robotis_framework::DynamixelState();
  result_["shin_l_active"] = new robotis_framework::DynamixelState();
  result_["ankle_l_roll"] = new robotis_framework::DynamixelState();

  /* leg */
  joint_name_to_dxl_id_["ankle_r_yaw"] = 1;
  joint_name_to_dxl_id_["ankle_l_yaw"] = 2;
  joint_name_to_dxl_id_["hip_r_roll"] = 3;
  joint_name_to_dxl_id_["hip_l_roll"] = 4;
  joint_name_to_dxl_id_["hip_r_pitch"] = 5;
  joint_name_to_dxl_id_["hip_l_pitch"] = 6;
  joint_name_to_dxl_id_["thigh_r_active"] = 7;
  joint_name_to_dxl_id_["thigh_l_active"] = 8;
  joint_name_to_dxl_id_["shin_r_active"] = 9;
  joint_name_to_dxl_id_["shin_l_active"] = 10;
  joint_name_to_dxl_id_["ankle_r_roll"] = 11;
  joint_name_to_dxl_id_["ankle_l_roll"] = 12;

  resetBodyPose();

  // walking parameter default
  walking_param_.dsp_ratio = 0.2;
  walking_param_.lipm_height = 0.12;
  walking_param_.foot_height_max = 0.05;
  walking_param_.zmp_offset_x = 0.0;  // not applied
  walking_param_.zmp_offset_y = 0.0;

  balance_control_.initialize(control_cycle_sec_ * 1000.0);
  balance_control_.setGyroBalanceEnable(false);         // Gyro
  balance_control_.setOrientationBalanceEnable(false);  // IMU
  balance_control_.setForceTorqueBalanceEnable(false);  // FT

  balance_l_foot_force_x_ = 0.0;
  balance_l_foot_force_y_ = 0.0;
  balance_l_foot_force_z_ = 0.0;
  balance_l_foot_torque_x_ = 0.0;
  balance_l_foot_torque_y_ = 0.0;
  balance_l_foot_torque_z_ = 0.0;

  balance_r_foot_force_x_ = 0.0;
  balance_r_foot_force_y_ = 0.0;
  balance_r_foot_force_z_ = 0.0;
  balance_r_foot_torque_x_ = 0.0;
  balance_r_foot_torque_y_ = 0.0;
  balance_r_foot_torque_z_ = 0.0;

  std::string balance_gain_path = ros::package::getPath("kuroko_online_walking_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  std::string joint_feedback_gain_path = ros::package::getPath("kuroko_online_walking_module") + "/config/"
                                                                                                 "joint_feedback_gain."
                                                                                                 "yaml";
  parseJointFeedbackGainData(joint_feedback_gain_path);

  std::string joint_feedforward_gain_path = ros::package::getPath("kuroko_online_walking_module") + "/config/"
                                                                                                    "joint_feedforward_"
                                                                                                    "gain.yaml";
  parseJointFeedforwardGainData(joint_feedforward_gain_path);
}

OnlineWalkingModule::~OnlineWalkingModule()
{
  if (queue_thread_.joinable())
    queue_thread_.join();
}

void OnlineWalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot* /*robot*/)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&OnlineWalkingModule::queueThread, this));

  ros::NodeHandle ros_node;

  // Publisher
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/motion_control/status", 1);
  movement_done_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/movement_done", 1);
  goal_joint_state_pub_ =
      ros_node.advertise<sensor_msgs::JointState>("/motion_control/online_walking/goal_joint_states", 1);
  pelvis_pose_pub_ = ros_node.advertise<geometry_msgs::PoseStamped>("/motion_control/pelvis_pose", 1);

  // Service
  //  get_preview_matrix_client_ =
  //  ros_node.serviceClient<KUROKO_ONLINE_WALKING_MODULE_msgs::GetPreviewMatrix>("/motion_control/online_walking/get_preview_matrix",
  //  0);
}

void OnlineWalkingModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  // Subscriber
  ros::Subscriber reset_body_sub = ros_node.subscribe("/motion_control/online_walking/reset_body", 5,
                                                      &OnlineWalkingModule::setResetBodyCallback, this);
  ros::Subscriber joint_pose_sub = ros_node.subscribe("/motion_control/online_walking/goal_joint_pose", 5,
                                                      &OnlineWalkingModule::goalJointPoseCallback, this);
  ros::Subscriber kinematics_pose_sub = ros_node.subscribe("/motion_control/online_walking/goal_kinematics_pose", 5,
                                                           &OnlineWalkingModule::goalKinematicsPoseCallback, this);
  ros::Subscriber foot_step_command_sub = ros_node.subscribe("/motion_control/online_walking/foot_step_command", 5,
                                                             &OnlineWalkingModule::footStepCommandCallback, this);
  ros::Subscriber walking_param_sub = ros_node.subscribe("/motion_control/online_walking/walking_param", 5,
                                                         &OnlineWalkingModule::walkingParamCallback, this);
  ros::Subscriber wholebody_balance_msg_sub =
      ros_node.subscribe("/motion_control/online_walking/wholebody_balance_msg", 5,
                         &OnlineWalkingModule::setWholebodyBalanceMsgCallback, this);
  ros::Subscriber body_offset_msg_sub = ros_node.subscribe("/motion_control/online_walking/body_offset", 5,
                                                           &OnlineWalkingModule::setBodyOffsetCallback, this);
  ros::Subscriber foot_distance_msg_sub = ros_node.subscribe("/motion_control/online_walking/foot_distance", 5,
                                                             &OnlineWalkingModule::setFootDistanceCallback, this);

  ros::Subscriber footsteps_sub = ros_node.subscribe("/motion_control/online_walking/footsteps_2d", 5,
                                                     &OnlineWalkingModule::footStep2DCallback, this);

  //  ros::Subscriber imu_data_sub =
  //  ros_node.subscribe("/motion_control/sensor/imu/imu", 5,
  //                                                    &OnlineWalkingModule::imuDataCallback,
  //                                                    this);
  //  ros::Subscriber l_foot_ft_sub =
  //  ros_node.subscribe("/motion_control/sensor/l_foot_ft", 3,
  //                                                     &OnlineWalkingModule::leftFootForceTorqueOutputCallback,
  //                                                     this);
  //  ros::Subscriber r_foot_ft_sub =
  //  ros_node.subscribe("/motion_control/sensor/r_foot_ft", 3,
  //                                                     &OnlineWalkingModule::rightFootForceTorqueOutputCallback,
  //                                                     this);

  // Service
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService(
      "/motion_control/online_walking/get_joint_pose", &OnlineWalkingModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService(
      "/motion_control/online_walking/get_kinematics_pose", &OnlineWalkingModule::getKinematicsPoseCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void OnlineWalkingModule::resetBodyPose()
{
  des_body_position_[0] = 0.0;
  des_body_position_[1] = 0.0;
  des_body_position_[2] = 0.3402256;

  des_body_rpy_[0] = 0.0;
  des_body_rpy_[1] = 0.0;
  des_body_rpy_[2] = 0.0;

  des_r_leg_position_[0] = 0.0;
  des_r_leg_position_[1] = -0.5 * foot_distance_;  //-0.045; //-0.035;
  des_r_leg_position_[2] = 0.0;

  des_r_leg_rpy_[0] = 0.0;
  des_r_leg_rpy_[1] = 0.0;
  des_r_leg_rpy_[2] = 0.0;

  des_l_leg_position_[0] = 0.0;
  des_l_leg_position_[1] = 0.5 * foot_distance_;  // 0.045; //0.035;
  des_l_leg_position_[2] = 0.0;

  des_l_leg_rpy_[0] = 0.0;
  des_l_leg_rpy_[1] = 0.0;
  des_l_leg_rpy_[2] = 0.0;

  x_lipm_[0] = des_body_position_[0];
  x_lipm_[1] = 0.0;
  x_lipm_[2] = 0.0;

  y_lipm_[0] = des_body_position_[1];
  y_lipm_[1] = 0.0;
  y_lipm_[2] = 0.0;

  walking_param_.zmp_offset_x = des_body_position_[0];
}

void OnlineWalkingModule::parseBalanceGainData(const std::string& path)
{
  YAML::Node doc;
  ROS_INFO("OnlineWalkingModule::parseBalanceGainData - Loading: %s", path.c_str());
  try
  {
    // load yaml
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  //  ROS_INFO("Parse Balance Gain Data");

  foot_roll_gyro_p_gain_ = doc["foot_roll_gyro_p_gain"].as<double>();
  foot_roll_gyro_d_gain_ = doc["foot_roll_gyro_d_gain"].as<double>();
  foot_pitch_gyro_p_gain_ = doc["foot_pitch_gyro_p_gain"].as<double>();
  foot_pitch_gyro_d_gain_ = doc["foot_pitch_gyro_d_gain"].as<double>();

  foot_roll_angle_p_gain_ = doc["foot_roll_angle_p_gain"].as<double>();
  foot_roll_angle_d_gain_ = doc["foot_roll_angle_d_gain"].as<double>();
  foot_pitch_angle_p_gain_ = doc["foot_pitch_angle_p_gain"].as<double>();
  foot_pitch_angle_d_gain_ = doc["foot_pitch_angle_d_gain"].as<double>();

  foot_x_force_p_gain_ = doc["foot_x_force_p_gain"].as<double>();
  foot_x_force_d_gain_ = doc["foot_x_force_d_gain"].as<double>();
  foot_y_force_p_gain_ = doc["foot_y_force_p_gain"].as<double>();
  foot_y_force_d_gain_ = doc["foot_y_force_d_gain"].as<double>();
  foot_z_force_p_gain_ = doc["foot_z_force_p_gain"].as<double>();
  foot_z_force_d_gain_ = doc["foot_z_force_d_gain"].as<double>();

  foot_roll_torque_p_gain_ = doc["foot_roll_torque_p_gain"].as<double>();
  foot_roll_torque_d_gain_ = doc["foot_roll_torque_d_gain"].as<double>();
  foot_pitch_torque_p_gain_ = doc["foot_pitch_torque_p_gain"].as<double>();
  foot_pitch_torque_d_gain_ = doc["foot_pitch_torque_d_gain"].as<double>();

  roll_gyro_cut_off_frequency_ = doc["roll_gyro_cut_off_frequency"].as<double>();
  pitch_gyro_cut_off_frequency_ = doc["pitch_gyro_cut_off_frequency"].as<double>();

  roll_angle_cut_off_frequency_ = doc["roll_angle_cut_off_frequency"].as<double>();
  pitch_angle_cut_off_frequency_ = doc["pitch_angle_cut_off_frequency"].as<double>();

  foot_x_force_cut_off_frequency_ = doc["foot_x_force_cut_off_frequency"].as<double>();
  foot_y_force_cut_off_frequency_ = doc["foot_y_force_cut_off_frequency"].as<double>();
  foot_z_force_cut_off_frequency_ = doc["foot_z_force_cut_off_frequency"].as<double>();

  foot_roll_torque_cut_off_frequency_ = doc["foot_roll_torque_cut_off_frequency"].as<double>();
  foot_pitch_torque_cut_off_frequency_ = doc["foot_pitch_torque_cut_off_frequency"].as<double>();

  balance_hip_roll_gain_ = doc["balance_hip_roll_gain"].as<double>();
  balance_knee_gain_ = doc["balance_knee_gain"].as<double>();
  balance_ankle_roll_gain_ = doc["balance_ankle_roll_gain"].as<double>();
  balance_ankle_pitch_gain_ = doc["balance_ankle_pitch_gain"].as<double>();
}

void OnlineWalkingModule::parseJointFeedbackGainData(const std::string& path)
{
  YAML::Node doc;
  ROS_INFO("OnlineWalkingModule::parseJointFeedbackGainData - Loading: %s", path.c_str());
  try
  {
    // load yaml
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  joint_feedback_[joint_name_to_dxl_id_["hip_r_roll"] - 1].p_gain_ = doc["hip_r_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["hip_r_roll"] - 1].d_gain_ = doc["hip_r_roll_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["hip_r_pitch"] - 1].p_gain_ = doc["hip_r_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["hip_r_pitch"] - 1].d_gain_ = doc["hip_r_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["thigh_r_active"] - 1].p_gain_ = doc["thigh_r_active_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["thigh_r_active"] - 1].d_gain_ = doc["thigh_r_active_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["shin_r_active"] - 1].p_gain_ = doc["shin_r_active_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["shin_r_active"] - 1].d_gain_ = doc["shin_r_active_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["ankle_r_roll"] - 1].p_gain_ = doc["ankle_r_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["ankle_r_roll"] - 1].d_gain_ = doc["ankle_r_roll_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["ankle_r_yaw"] - 1].p_gain_ = doc["ankle_r_yaw_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["ankle_r_yaw"] - 1].d_gain_ = doc["ankle_r_yaw_d_gain"].as<double>();

  joint_feedback_[joint_name_to_dxl_id_["hip_l_roll"] - 1].p_gain_ = doc["hip_l_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["hip_l_roll"] - 1].d_gain_ = doc["hip_l_roll_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["hip_l_pitch"] - 1].p_gain_ = doc["hip_l_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["hip_l_pitch"] - 1].d_gain_ = doc["hip_l_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["thigh_l_active"] - 1].p_gain_ = doc["thigh_l_active_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["thigh_l_active"] - 1].d_gain_ = doc["thigh_l_active_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["shin_l_active"] - 1].p_gain_ = doc["shin_l_active_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["shin_l_active"] - 1].d_gain_ = doc["shin_l_active_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["ankle_l_roll"] - 1].p_gain_ = doc["ankle_l_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["ankle_l_roll"] - 1].d_gain_ = doc["ankle_l_roll_d_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["ankle_l_yaw"] - 1].p_gain_ = doc["ankle_l_yaw_p_gain"].as<double>();
  joint_feedback_[joint_name_to_dxl_id_["ankle_l_yaw"] - 1].d_gain_ = doc["ankle_l_yaw_d_gain"].as<double>();
}

void OnlineWalkingModule::parseJointFeedforwardGainData(const std::string& path)
{
  YAML::Node doc;
  ROS_INFO("OnlineWalkingModule::parseJointFeedforwardGainData - Loading: %s", path.c_str());
  try
  {
    // load yaml
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  joint_feedforward_gain_[joint_name_to_dxl_id_["hip_r_roll"] - 1] = doc["hip_r_roll_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["hip_r_pitch"] - 1] = doc["hip_r_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["thigh_r_active"] - 1] = doc["thigh_r_active_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["shin_r_active"] - 1] = doc["shin_r_active_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["ankle_r_roll"] - 1] = doc["ankle_r_roll_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["ankle_r_yaw"] - 1] = doc["ankle_r_yaw_gain"].as<double>();

  joint_feedforward_gain_[joint_name_to_dxl_id_["hip_l_roll"] - 1] = doc["hip_l_roll_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["hip_l_pitch"] - 1] = doc["hip_l_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["thigh_l_active"] - 1] = doc["thigh_l_active_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["shin_l_active"] - 1] = doc["shin_l_active_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["ankle_l_roll"] - 1] = doc["ankle_l_roll_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_dxl_id_["ankle_l_yaw"] - 1] = doc["ankle_l_yaw_gain"].as<double>();
}

void OnlineWalkingModule::setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (!enable_)
    return;

  std::string balance_gain_path = ros::package::getPath("kuroko_online_walking_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  std::string joint_feedback_gain_path = ros::package::getPath("kuroko_online_walking_module") + "/config/"
                                                                                                 "joint_feedback_gain."
                                                                                                 "yaml";
  parseJointFeedbackGainData(joint_feedback_gain_path);

  std::string joint_feedforward_gain_path = ros::package::getPath("kuroko_online_walking_module") + "/config/"
                                                                                                    "joint_feedforward_"
                                                                                                    "gain.yaml";
  parseJointFeedforwardGainData(joint_feedforward_gain_path);

  if (msg->data == "balance_on")
    goal_balance_gain_ratio_[0] = 1.0;
  else if (msg->data == "balance_off")
    goal_balance_gain_ratio_[0] = 0.0;

  balance_control_initialize_ = false;
  balance_type_ = ON;
  walking_phase_ = DSP;
}

void OnlineWalkingModule::initBalanceControl()
{
  if (balance_control_initialize_)
    return;

  double ini_time = 0.0;
  double mov_time = 1.0;

  balance_step_ = 0;
  balance_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  std::vector<double_t> balance_zero;
  balance_zero.resize(1, 0.0);

  balance_trajectory_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time, des_balance_gain_ratio_, balance_zero, balance_zero,
                                         goal_balance_gain_ratio_, balance_zero, balance_zero);

  if (is_balancing_)
    ROS_INFO("[UPDATE] Balance Gain");
  else
  {
    is_balancing_ = true;
    ROS_INFO("[START] Balance Gain");
  }
  balance_control_initialize_ = true;
}

void OnlineWalkingModule::calcBalanceControl()
{
  if (is_balancing_)
  {
    double cur_time = (double)balance_step_ * control_cycle_sec_;
    des_balance_gain_ratio_ = balance_trajectory_->getPosition(cur_time);

    if (balance_step_ == balance_size_ - 1)
    {
      balance_step_ = 0;
      is_balancing_ = false;
      delete balance_trajectory_;

      if (des_balance_gain_ratio_[0] == 0.0)
      {
        control_type_ = NONE;
        balance_type_ = OFF;
      }

      ROS_INFO("[END] Balance Gain");
    }
    else
      balance_step_++;
  }
}

void OnlineWalkingModule::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_data_mutex_lock_.lock();

  imu_data_msg_ = *msg;

  imu_data_msg_.angular_velocity.x *= -1.0;
  imu_data_msg_.angular_velocity.y *= -1.0;

  imu_data_mutex_lock_.unlock();
}

void OnlineWalkingModule::leftFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::MatrixXd force = Eigen::MatrixXd::Zero(3, 1);
  force.coeffRef(0, 0) = msg->wrench.force.x;
  force.coeffRef(1, 0) = msg->wrench.force.y;
  force.coeffRef(2, 0) = msg->wrench.force.z;

  Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(3, 1);
  torque.coeffRef(0, 0) = msg->wrench.torque.x;
  torque.coeffRef(1, 0) = msg->wrench.torque.y;
  torque.coeffRef(2, 0) = msg->wrench.torque.z;

  Eigen::MatrixXd force_new =
      robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5 * M_PI) * force;
  Eigen::MatrixXd torque_new =
      robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5 * M_PI) * torque;

  double l_foot_fx_n = force_new.coeff(0, 0);
  double l_foot_fy_n = force_new.coeff(1, 0);
  double l_foot_fz_n = force_new.coeff(2, 0);
  double l_foot_tx_nm = torque_new.coeff(0, 0);
  double l_foot_ty_nm = torque_new.coeff(1, 0);
  double l_foot_tz_nm = torque_new.coeff(2, 0);

  l_foot_fx_n = robotis_framework::sign(l_foot_fx_n) * fmin(fabs(l_foot_fx_n), 2000.0);
  l_foot_fy_n = robotis_framework::sign(l_foot_fy_n) * fmin(fabs(l_foot_fy_n), 2000.0);
  l_foot_fz_n = robotis_framework::sign(l_foot_fz_n) * fmin(fabs(l_foot_fz_n), 2000.0);
  l_foot_tx_nm = robotis_framework::sign(l_foot_tx_nm) * fmin(fabs(l_foot_tx_nm), 300.0);
  l_foot_ty_nm = robotis_framework::sign(l_foot_ty_nm) * fmin(fabs(l_foot_ty_nm), 300.0);
  l_foot_tz_nm = robotis_framework::sign(l_foot_tz_nm) * fmin(fabs(l_foot_tz_nm), 300.0);

  l_foot_ft_data_msg_.force.x = l_foot_fx_n;
  l_foot_ft_data_msg_.force.y = l_foot_fy_n;
  l_foot_ft_data_msg_.force.z = l_foot_fz_n;
  l_foot_ft_data_msg_.torque.x = l_foot_tx_nm;
  l_foot_ft_data_msg_.torque.y = l_foot_ty_nm;
  l_foot_ft_data_msg_.torque.z = l_foot_tz_nm;
}

void OnlineWalkingModule::rightFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::MatrixXd force = Eigen::MatrixXd::Zero(3, 1);
  force.coeffRef(0, 0) = msg->wrench.force.x;
  force.coeffRef(1, 0) = msg->wrench.force.y;
  force.coeffRef(2, 0) = msg->wrench.force.z;

  Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(3, 1);
  torque.coeffRef(0, 0) = msg->wrench.torque.x;
  torque.coeffRef(1, 0) = msg->wrench.torque.y;
  torque.coeffRef(2, 0) = msg->wrench.torque.z;

  Eigen::MatrixXd force_new =
      robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5 * M_PI) * force;
  Eigen::MatrixXd torque_new =
      robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5 * M_PI) * torque;

  double r_foot_fx_n = force_new.coeff(0, 0);
  double r_foot_fy_n = force_new.coeff(1, 0);
  double r_foot_fz_n = force_new.coeff(2, 0);
  double r_foot_tx_nm = torque_new.coeff(0, 0);
  double r_foot_ty_nm = torque_new.coeff(1, 0);
  double r_foot_tz_nm = torque_new.coeff(2, 0);

  r_foot_fx_n = robotis_framework::sign(r_foot_fx_n) * fmin(fabs(r_foot_fx_n), 2000.0);
  r_foot_fy_n = robotis_framework::sign(r_foot_fy_n) * fmin(fabs(r_foot_fy_n), 2000.0);
  r_foot_fz_n = robotis_framework::sign(r_foot_fz_n) * fmin(fabs(r_foot_fz_n), 2000.0);
  r_foot_tx_nm = robotis_framework::sign(r_foot_tx_nm) * fmin(fabs(r_foot_tx_nm), 300.0);
  r_foot_ty_nm = robotis_framework::sign(r_foot_ty_nm) * fmin(fabs(r_foot_ty_nm), 300.0);
  r_foot_tz_nm = robotis_framework::sign(r_foot_tz_nm) * fmin(fabs(r_foot_tz_nm), 300.0);

  r_foot_ft_data_msg_.force.x = r_foot_fx_n;
  r_foot_ft_data_msg_.force.y = r_foot_fy_n;
  r_foot_ft_data_msg_.force.z = r_foot_fz_n;
  r_foot_ft_data_msg_.torque.x = r_foot_tx_nm;
  r_foot_ft_data_msg_.torque.y = r_foot_ty_nm;
  r_foot_ft_data_msg_.torque.z = r_foot_tz_nm;
}

void OnlineWalkingModule::setResetBodyCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (static_cast<bool>(msg->data))
  {
    des_body_offset_[0] = 0.0;
    des_body_offset_[1] = 0.0;
    des_body_offset_[2] = 0.0;

    resetBodyPose();
  }
}

void OnlineWalkingModule::walkingParamCallback(const op3_online_walking_module_msgs::WalkingParam& msg)
{
  walking_param_ = msg;
}

void OnlineWalkingModule::goalJointPoseCallback(const op3_online_walking_module_msgs::JointPose& msg)
{
  if (!enable_)
    return;

  size_t joint_size = msg.pose.name.size();

  if (control_type_ == NONE || control_type_ == JOINT_CONTROL)
  {
    mov_time_ = msg.mov_time;

    for (size_t i = 0; i < msg.pose.name.size(); i++)
    {
      std::string joint_name = msg.pose.name[i];
      goal_joint_pos_[joint_name_to_dxl_id_[joint_name] - 1] = msg.pose.position[i];
    }

    joint_control_initialize_ = false;
    control_type_ = JOINT_CONTROL;
    balance_type_ = OFF;
    des_balance_gain_ratio_[0] = 0.0;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::initJointControl()
{
  if (joint_control_initialize_)
    return;

  joint_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  joint_trajectory_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time, des_joint_pos_, des_joint_vel_, des_joint_accel_,
                                         goal_joint_pos_, goal_joint_vel_, goal_joint_accel_);
  if (is_moving_)
    ROS_INFO("[UPDATE] Joint Control");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Joint Control");
  }
}

void OnlineWalkingModule::calcJointControl()
{
  if (is_moving_)
  {
    double cur_time = (double)mov_step_ * control_cycle_sec_;

    queue_mutex_.lock();

    des_joint_pos_ = joint_trajectory_->getPosition(cur_time);
    des_joint_vel_ = joint_trajectory_->getVelocity(cur_time);
    des_joint_accel_ = joint_trajectory_->getAcceleration(cur_time);

    queue_mutex_.unlock();

    if (mov_step_ == mov_size_ - 1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      delete joint_trajectory_;

      control_type_ = NONE;

      ROS_INFO("[END] Joint Control");
    }
    else
      mov_step_++;
  }
}

void OnlineWalkingModule::setBodyOffsetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (!enable_)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  if (control_type_ == NONE || control_type_ == OFFSET_CONTROL)
  {
    goal_body_offset_[0] = msg->position.x;
    goal_body_offset_[1] = msg->position.y;
    goal_body_offset_[2] = msg->position.z;

    body_offset_initialize_ = false;
    control_type_ = OFFSET_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::setFootDistanceCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if (!enable_)
    return;

  foot_distance_ = msg->data;

  resetBodyPose();
}

void OnlineWalkingModule::initOffsetControl()
{
  if (body_offset_initialize_)
    return;

  body_offset_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = 1.0;

  body_offset_step_ = 0;
  body_offset_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  std::vector<double_t> offset_zero;
  offset_zero.resize(3, 0.0);

  body_offset_trajectory_ = new robotis_framework::MinimumJerk(
      ini_time, mov_time, des_body_offset_, offset_zero, offset_zero, goal_body_offset_, offset_zero, offset_zero);

  if (is_moving_)
    ROS_INFO("[UPDATE] Body Offset");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Body Offset");
  }
}

void OnlineWalkingModule::calcOffsetControl()
{
  if (is_moving_)
  {
    double cur_time = (double)body_offset_step_ * control_cycle_sec_;

    queue_mutex_.lock();

    des_body_offset_ = body_offset_trajectory_->getPosition(cur_time);

    queue_mutex_.unlock();

    if (body_offset_step_ == mov_size_ - 1)
    {
      body_offset_step_ = 0;
      is_moving_ = false;
      delete body_offset_trajectory_;

      control_type_ = NONE;

      ROS_INFO("[END] Body Offset");
    }
    else
      body_offset_step_++;
  }
}

void OnlineWalkingModule::goalKinematicsPoseCallback(const op3_online_walking_module_msgs::KinematicsPose& msg)
{
  if (!enable_)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  if (control_type_ == NONE || control_type_ == WHOLEBODY_CONTROL)
  {
    if (is_moving_)
    {
      if (wholegbody_control_group_ != msg.name)
      {
        ROS_WARN("[WARN] Control group is different!");
        return;
      }
    }
    mov_time_ = msg.mov_time;
    wholegbody_control_group_ = msg.name;
    wholebody_goal_msg_ = msg.pose;

    wholebody_initialize_ = false;
    control_type_ = WHOLEBODY_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::initWholebodyControl()
{
  if (wholebody_initialize_)
    return;

  wholebody_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  wholebody_control_ = new WholebodyControl(wholegbody_control_group_, ini_time, mov_time, wholebody_goal_msg_);

  if (is_moving_)
  {
    // TODO
  }
  else
  {
    ROS_INFO("[START] Wholebody Control");

    wholebody_control_->initialize(des_body_position_, des_body_rpy_, des_r_leg_position_, des_r_leg_rpy_,
                                   des_l_leg_position_, des_l_leg_rpy_);
    is_moving_ = true;
  }
}

void OnlineWalkingModule::calcWholebodyControl()
{
  if (is_moving_)
  {
    double cur_time = (double)mov_step_ * control_cycle_sec_;

    wholebody_control_->set(cur_time);

    wholebody_control_->getTaskPosition(des_l_leg_position_, des_r_leg_position_, des_body_position_);
    wholebody_control_->getTaskOrientation(des_l_leg_rpy_, des_r_leg_rpy_, des_body_rpy_);

    if (mov_step_ == mov_size_ - 1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      wholebody_control_->finalize();

      control_type_ = NONE;

      ROS_INFO("[END] Wholebody Control");
    }
    else
      mov_step_++;
  }
}

void OnlineWalkingModule::footStep2DCallback(const op3_online_walking_module_msgs::Step2DArray& msg)
{
  if (!enable_)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }
  Eigen::MatrixXd body_r =
      robotis_framework::convertRPYToRotation(des_body_rpy_[0], des_body_rpy_[1], des_body_rpy_[2]);
  Eigen::MatrixXd body_t = Eigen::MatrixXd::Identity(4, 4);
  body_t.block(0, 0, 3, 3) = body_r;
  body_t.coeffRef(0, 3) = des_body_position_[0];
  body_t.coeffRef(1, 3) = des_body_position_[1];

  op3_online_walking_module_msgs::Step2DArray foot_step_msg;

  int old_size = msg.footsteps_2d.size();
  int new_size = old_size + 3;

  op3_online_walking_module_msgs::Step2D first_msg;
  op3_online_walking_module_msgs::Step2D second_msg;

  first_msg.moving_foot = msg.footsteps_2d[0].moving_foot - 1;
  second_msg.moving_foot = first_msg.moving_foot + 1;

  if (first_msg.moving_foot == LEFT_LEG)
  {
    first_msg.step2d.x = des_l_leg_position_[0];
    first_msg.step2d.y = des_l_leg_position_[1];
    first_msg.step2d.theta = des_body_rpy_[2];

    second_msg.step2d.x = des_r_leg_position_[0];
    second_msg.step2d.y = des_r_leg_position_[1];
    second_msg.step2d.theta = des_body_rpy_[2];
  }
  else if (first_msg.moving_foot == RIGHT_LEG)
  {
    first_msg.step2d.x = des_r_leg_position_[0];
    first_msg.step2d.y = des_r_leg_position_[1];
    first_msg.step2d.theta = des_body_rpy_[2];

    second_msg.step2d.x = des_l_leg_position_[0];
    second_msg.step2d.y = des_l_leg_position_[1];
    second_msg.step2d.theta = des_body_rpy_[2];
  }

  foot_step_msg.footsteps_2d.push_back(first_msg);
  foot_step_msg.footsteps_2d.push_back(second_msg);

  double step_final_theta;

  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    for (int i = 0; i < old_size; i++)
    {
      op3_online_walking_module_msgs::Step2D step_msg = msg.footsteps_2d[i];
      step_msg.moving_foot -= 1;

      Eigen::MatrixXd step_r = robotis_framework::convertRPYToRotation(0.0, 0.0, step_msg.step2d.theta);
      Eigen::MatrixXd step_t = Eigen::MatrixXd::Identity(4, 4);
      step_t.block(0, 0, 3, 3) = step_r;
      step_t.coeffRef(0, 3) = step_msg.step2d.x;
      step_t.coeffRef(1, 3) = step_msg.step2d.y;

      Eigen::MatrixXd step_t_new = body_t * step_t;
      Eigen::MatrixXd step_r_new = step_t_new.block(0, 0, 3, 3);

      double step_new_x = step_t_new.coeff(0, 3);
      double step_new_y = step_t_new.coeff(1, 3);
      Eigen::MatrixXd step_new_rpy = robotis_framework::convertRotationToRPY(step_r_new);
      double step_new_theta = step_new_rpy.coeff(2, 0);

      step_msg.step2d.x = step_new_x;
      step_msg.step2d.y = step_new_y;
      step_msg.step2d.theta = step_new_theta;

      if (i == old_size - 1)
        step_final_theta = step_new_theta;

      foot_step_msg.footsteps_2d.push_back(step_msg);
    }

    op3_online_walking_module_msgs::Step2D step_msg = msg.footsteps_2d[old_size - 1];

    if (step_msg.moving_foot - 1 == LEFT_LEG)
      first_msg.moving_foot = RIGHT_LEG;
    else
      first_msg.moving_foot = LEFT_LEG;

    first_msg.step2d.x = 0.0;
    first_msg.step2d.y = 0.0;
    first_msg.step2d.theta = step_final_theta;  // step_msg.step2d.theta;

    foot_step_msg.footsteps_2d.push_back(first_msg);

    foot_step_2d_ = foot_step_msg;
    foot_step_2d_.step_time = msg.step_time;

    walking_size_ = new_size;
    mov_time_ = msg.step_time;  // 1.0;
    is_foot_step_2d_ = true;
    control_type_ = WALKING_CONTROL;

    if (!is_moving_)
      initWalkingControl();
    else
      ROS_WARN("[WARN] Previous task is alive!");
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::footStepCommandCallback(const op3_online_walking_module_msgs::FootStepCommand& msg)
{
  if (!enable_)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  is_foot_step_2d_ = false;

  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    walking_size_ = msg.step_num + 3;  // msg.step_num + 2;
    mov_time_ = msg.step_time;

    foot_step_command_ = msg;
    foot_step_command_.step_num = walking_size_;

    control_type_ = WALKING_CONTROL;

    if (!is_moving_)
      initWalkingControl();
    else
      ROS_WARN("[WARN] Previous task is alive!");
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::initWalkingControl()
{
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  walking_step_ = 0;

  walking_control_ = new WalkingControl(control_cycle_sec_, walking_param_.dsp_ratio, walking_param_.lipm_height,
                                        walking_param_.foot_height_max, walking_param_.zmp_offset_x,
                                        walking_param_.zmp_offset_y, x_lipm_, y_lipm_, foot_distance_);

  double lipm_height = walking_control_->getLipmHeight();
  preview_request_.lipm_height = lipm_height;
  preview_request_.control_cycle = control_cycle_sec_;

  bool get_preview_matrix = false;
  get_preview_matrix = definePreviewMatrix();

  if (get_preview_matrix)
  {
    if (is_moving_)
    {
      // TODO
    }
    else
    {
      if (is_foot_step_2d_)
      {
        walking_control_->initialize(foot_step_2d_, des_body_position_, des_body_rpy_, des_r_leg_position_,
                                     des_r_leg_rpy_, des_l_leg_position_, des_l_leg_rpy_);
      }
      else
      {
        walking_control_->initialize(foot_step_command_, des_body_position_, des_body_rpy_, des_r_leg_position_,
                                     des_r_leg_rpy_, des_l_leg_position_, des_l_leg_rpy_);
      }

      walking_control_->calcPreviewParam(preview_response_k_, preview_response_k_row_, preview_response_k_col_,
                                         preview_response_p_, preview_response_p_row_, preview_response_p_row_);

      is_moving_ = true;

      initFeedforwardControl();

      ROS_INFO("[START] Walking Control (%d/%d)", walking_step_ + 1, walking_size_);
    }

    walking_initialize_ = true;
  }
  else
    ROS_WARN("[FAIL] Cannot get preview matrix");
}

void OnlineWalkingModule::calcWalkingControl()
{
  if (is_moving_)
  {
    double cur_time = (double)mov_step_ * control_cycle_sec_;
    walking_control_->set(cur_time, walking_step_, is_foot_step_2d_);

    walking_control_->getWalkingPosition(des_l_leg_position_, des_r_leg_position_, des_body_position_);
    walking_control_->getWalkingOrientation(des_l_leg_rpy_, des_r_leg_rpy_, des_body_rpy_);

    walking_control_->getLIPM(x_lipm_, y_lipm_);

    walking_control_->getWalkingState(walking_leg_, walking_phase_);

    if (mov_step_ == mov_size_ - 1)
    {
      ROS_INFO("[END] Walking Control (%d/%d)", walking_step_ + 1, walking_size_);

      mov_step_ = 0;
      walking_control_->next();

      if (walking_step_ == walking_size_ - 1)
      {
        is_moving_ = false;
        is_foot_step_2d_ = false;
        walking_control_->finalize();

        control_type_ = NONE;
        walking_phase_ = DSP;
      }
      else
      {
        walking_step_++;
        ROS_INFO("[START] Walking Control (%d/%d)", walking_step_ + 1, walking_size_);
      }
    }
    else
      mov_step_++;
  }
}

void OnlineWalkingModule::initFeedforwardControl()
{
  // feedforward trajectory
  std::vector<double_t> zero_vector;
  zero_vector.resize(1, 0.0);

  std::vector<double_t> via_pos;
  via_pos.resize(3, 0.0);
  via_pos[0] = 1.0 * DEGREE2RADIAN;

  double init_time = 0.0;
  double fin_time = mov_time_;
  double via_time = 0.5 * (init_time + fin_time);
  double dsp_ratio = walking_param_.dsp_ratio;

  feed_forward_trajectory_ =
      new robotis_framework::MinimumJerkViaPoint(init_time, fin_time, via_time, dsp_ratio, zero_vector, zero_vector,
                                                 zero_vector, zero_vector, zero_vector, zero_vector, via_pos,
                                                 zero_vector, zero_vector);
}

void OnlineWalkingModule::calcRobotPose()
{
  Eigen::MatrixXd des_body_pos = Eigen::MatrixXd::Zero(3, 1);
  des_body_pos.coeffRef(0, 0) = des_body_position_[0];
  des_body_pos.coeffRef(1, 0) = des_body_position_[1];
  des_body_pos.coeffRef(2, 0) = des_body_position_[2];

  Eigen::MatrixXd des_body_rot =
      robotis_framework::convertRPYToRotation(des_body_rpy_[0], des_body_rpy_[1], des_body_rpy_[2]);

  // Forward Kinematics

  std::vector<double_t> r_leg_joint_pos, l_leg_joint_pos;
  r_leg_joint_pos.resize(6);
  r_leg_joint_pos[0] = des_joint_pos_[joint_name_to_dxl_id_["hip_r_roll"] - 1];
  r_leg_joint_pos[1] = des_joint_pos_[joint_name_to_dxl_id_["hip_r_pitch"] - 1];
  r_leg_joint_pos[2] = des_joint_pos_[joint_name_to_dxl_id_["thigh_r_active"] - 1];
  r_leg_joint_pos[3] = des_joint_pos_[joint_name_to_dxl_id_["shin_r_active"] - 1];
  r_leg_joint_pos[4] = des_joint_pos_[joint_name_to_dxl_id_["ankle_r_roll"] - 1];
  r_leg_joint_pos[5] = des_joint_pos_[joint_name_to_dxl_id_["ankle_r_yaw"] - 1];

  l_leg_joint_pos.resize(6);
  l_leg_joint_pos[0] = des_joint_pos_[joint_name_to_dxl_id_["hip_l_roll"] - 1];
  l_leg_joint_pos[1] = des_joint_pos_[joint_name_to_dxl_id_["hip_l_pitch"] - 1];
  l_leg_joint_pos[2] = des_joint_pos_[joint_name_to_dxl_id_["thigh_l_active"] - 1];
  l_leg_joint_pos[3] = des_joint_pos_[joint_name_to_dxl_id_["shin_l_active"] - 1];
  l_leg_joint_pos[4] = des_joint_pos_[joint_name_to_dxl_id_["ankle_l_roll"] - 1];
  l_leg_joint_pos[5] = des_joint_pos_[joint_name_to_dxl_id_["ankle_l_yaw"] - 1];

  std::vector<double_t> r_leg_current_pose;
  std::vector<double_t> l_leg_current_pose;
  r_leg_current_pose.resize(6, 0.0);
  l_leg_current_pose.resize(6, 0.0);

  kuroko_kinematics_->solveForwardKinematicsForRightLeg(r_leg_joint_pos, r_leg_current_pose);
  kuroko_kinematics_->solveForwardKinematicsForLeftLeg(l_leg_joint_pos, l_leg_current_pose);
  std::vector<double_t> r_leg_position(r_leg_current_pose.begin(), r_leg_current_pose.begin() + 3);
  std::vector<double_t> r_leg_rpy(r_leg_current_pose.begin() + 3, r_leg_current_pose.end());
  std::vector<double_t> l_leg_position(l_leg_current_pose.begin(), l_leg_current_pose.begin() + 3);
  std::vector<double_t> l_leg_rpy(l_leg_current_pose.begin() + 3, l_leg_current_pose.end());

  Eigen::MatrixXd curr_r_leg_rot = robotis_framework::convertRPYToRotation(r_leg_rpy[0], r_leg_rpy[1], r_leg_rpy[2]);
  Eigen::MatrixXd g_to_r_leg = Eigen::MatrixXd::Identity(4, 4);
  g_to_r_leg.block(0, 0, 3, 3) = curr_r_leg_rot;
  g_to_r_leg.coeffRef(0, 3) = r_leg_position[0];
  g_to_r_leg.coeffRef(1, 3) = r_leg_position[1];
  g_to_r_leg.coeffRef(2, 3) = r_leg_position[2];

  Eigen::MatrixXd curr_l_leg_rot = robotis_framework::convertRPYToRotation(l_leg_rpy[0], l_leg_rpy[1], l_leg_rpy[2]);
  Eigen::MatrixXd g_to_l_leg = Eigen::MatrixXd::Identity(4, 4);
  g_to_l_leg.block(0, 0, 3, 3) = curr_l_leg_rot;
  g_to_l_leg.coeffRef(0, 3) = l_leg_position[0];
  g_to_l_leg.coeffRef(1, 3) = l_leg_position[1];
  g_to_l_leg.coeffRef(2, 3) = l_leg_position[2];
}

void OnlineWalkingModule::setTargetForceTorque()
{
  if (walking_phase_ == DSP)
  {
    balance_r_foot_force_x_ = -0.5 * total_mass_ * x_lipm_[2];
    balance_r_foot_force_y_ = -0.5 * total_mass_ * y_lipm_[2];
    balance_r_foot_force_z_ = -0.5 * total_mass_ * 9.81;

    balance_l_foot_force_x_ = -0.5 * total_mass_ * x_lipm_[2];
    balance_l_foot_force_y_ = -0.5 * total_mass_ * y_lipm_[2];
    balance_l_foot_force_z_ = -0.5 * total_mass_ * 9.81;
  }
  else if (walking_phase_ == SSP)
  {
    if (walking_leg_ == LEFT_LEG)
    {
      balance_r_foot_force_x_ = -1.0 * total_mass_ * x_lipm_[2];
      balance_r_foot_force_y_ = -1.0 * total_mass_ * y_lipm_[2];
      balance_r_foot_force_z_ = -1.0 * total_mass_ * 9.81;

      balance_l_foot_force_x_ = 0.0;
      balance_l_foot_force_y_ = 0.0;
      balance_l_foot_force_z_ = 0.0;
    }
    else if (walking_leg_ == RIGHT_LEG)
    {
      balance_r_foot_force_x_ = 0.0;
      balance_r_foot_force_y_ = 0.0;
      balance_r_foot_force_z_ = 0.0;

      balance_l_foot_force_x_ = -1.0 * total_mass_ * x_lipm_[2];
      balance_l_foot_force_y_ = -1.0 * total_mass_ * y_lipm_[2];
      balance_l_foot_force_z_ = -1.0 * total_mass_ * 9.81;
    }
  }
}

void OnlineWalkingModule::setBalanceControlGain()
{
  //// set gain
  // gyro
  balance_control_.foot_roll_gyro_ctrl_.p_gain_ = foot_roll_gyro_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_roll_gyro_ctrl_.d_gain_ = foot_roll_gyro_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.p_gain_ = foot_pitch_gyro_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.d_gain_ = foot_pitch_gyro_d_gain_ * des_balance_gain_ratio_[0];

  // orientation
  balance_control_.foot_roll_angle_ctrl_.p_gain_ = foot_roll_angle_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_roll_angle_ctrl_.d_gain_ = foot_roll_angle_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.p_gain_ = foot_pitch_angle_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.d_gain_ = foot_pitch_angle_d_gain_ * des_balance_gain_ratio_[0];

  // force torque
  balance_control_.right_foot_force_x_ctrl_.p_gain_ = foot_x_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.p_gain_ = foot_y_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.p_gain_ = foot_z_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_x_ctrl_.d_gain_ = foot_x_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.d_gain_ = foot_y_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.d_gain_ = foot_z_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];

  balance_control_.left_foot_force_x_ctrl_.p_gain_ = foot_x_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.p_gain_ = foot_y_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.p_gain_ = foot_z_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_x_ctrl_.d_gain_ = foot_x_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.d_gain_ = foot_y_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.d_gain_ = foot_z_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];

  //// set cut off freq
  balance_control_.roll_gyro_lpf_.setCutOffFrequency(roll_gyro_cut_off_frequency_);
  balance_control_.pitch_gyro_lpf_.setCutOffFrequency(pitch_gyro_cut_off_frequency_);
  balance_control_.roll_angle_lpf_.setCutOffFrequency(roll_angle_cut_off_frequency_);
  balance_control_.pitch_angle_lpf_.setCutOffFrequency(pitch_angle_cut_off_frequency_);

  balance_control_.right_foot_force_x_lpf_.setCutOffFrequency(foot_x_force_cut_off_frequency_);
  balance_control_.right_foot_force_y_lpf_.setCutOffFrequency(foot_y_force_cut_off_frequency_);
  balance_control_.right_foot_force_z_lpf_.setCutOffFrequency(foot_z_force_cut_off_frequency_);
  balance_control_.right_foot_torque_roll_lpf_.setCutOffFrequency(foot_roll_torque_cut_off_frequency_);
  balance_control_.right_foot_torque_pitch_lpf_.setCutOffFrequency(foot_pitch_torque_cut_off_frequency_);

  balance_control_.left_foot_force_x_lpf_.setCutOffFrequency(foot_x_force_cut_off_frequency_);
  balance_control_.left_foot_force_y_lpf_.setCutOffFrequency(foot_y_force_cut_off_frequency_);
  balance_control_.left_foot_force_z_lpf_.setCutOffFrequency(foot_z_force_cut_off_frequency_);
  balance_control_.left_foot_torque_roll_lpf_.setCutOffFrequency(foot_roll_torque_cut_off_frequency_);
  balance_control_.left_foot_torque_pitch_lpf_.setCutOffFrequency(foot_pitch_torque_cut_off_frequency_);
}

bool OnlineWalkingModule::setBalanceControl()
{
  // Set Balance Control
  balance_control_.setGyroBalanceEnable(true);
  balance_control_.setOrientationBalanceEnable(true);
  balance_control_.setForceTorqueBalanceEnable(true);

  balance_control_.setCOBManualAdjustment(des_body_offset_[0], des_body_offset_[1], des_body_offset_[2]);

  setBalanceControlGain();
  setTargetForceTorque();

  bool ik_success = true;

  // Body Pose
  Eigen::Vector3d des_body_pos = Eigen::Vector3d::Zero();
  des_body_pos.coeffRef(0, 0) = des_body_position_[0];
  des_body_pos.coeffRef(1, 0) = des_body_position_[1];
  des_body_pos.coeffRef(2, 0) = des_body_position_[2];

  Eigen::MatrixXd des_body_rot =
      robotis_framework::convertRPYToRotation(des_body_rpy_[0], des_body_rpy_[1], des_body_rpy_[2]);

  // Right Leg Pose
  Eigen::Vector3d des_r_foot_pos = Eigen::Vector3d::Zero();
  des_r_foot_pos.coeffRef(0, 0) = des_r_leg_position_[0];
  des_r_foot_pos.coeffRef(1, 0) = des_r_leg_position_[1];
  des_r_foot_pos.coeffRef(2, 0) = des_r_leg_position_[2];
  Eigen::MatrixXd des_r_foot_rot =
      robotis_framework::convertRPYToRotation(des_r_leg_rpy_[0], des_r_leg_rpy_[1], des_r_leg_rpy_[2]);

  Eigen::Vector3d des_l_foot_pos = Eigen::Vector3d::Zero();
  des_l_foot_pos.coeffRef(0, 0) = des_l_leg_position_[0];
  des_l_foot_pos.coeffRef(1, 0) = des_l_leg_position_[1];
  des_l_foot_pos.coeffRef(2, 0) = des_l_leg_position_[2];
  Eigen::MatrixXd des_l_foot_rot =
      robotis_framework::convertRPYToRotation(des_l_leg_rpy_[0], des_l_leg_rpy_[1], des_l_leg_rpy_[2]);

  // Set Desired Value for Balance Control
  Eigen::Matrix4d body_pose = Eigen::Matrix4d::Identity();
  body_pose.block<3, 3>(0, 0) = des_body_rot;
  body_pose.block<3, 1>(0, 3) = des_body_pos;

  Eigen::Matrix4d l_foot_pose = Eigen::Matrix4d::Identity();
  l_foot_pose.block<3, 3>(0, 0) = des_l_foot_rot;
  l_foot_pose.block<3, 1>(0, 3) = des_l_foot_pos;

  Eigen::Matrix4d r_foot_pose = Eigen::Matrix4d::Identity();
  r_foot_pose.block<3, 3>(0, 0) = des_r_foot_rot;
  r_foot_pose.block<3, 1>(0, 3) = des_r_foot_pos;

  // ===== Transformation =====
  Eigen::Matrix4d robot_to_body = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd robot_to_l_foot = body_pose.inverse() * l_foot_pose;
  Eigen::MatrixXd robot_to_r_foot = body_pose.inverse() * r_foot_pose;
  // =====

  // Set IMU
  imu_data_mutex_lock_.lock();

  balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

  Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w, imu_data_msg_.orientation.x,
                                    imu_data_msg_.orientation.y, imu_data_msg_.orientation.z);
  Eigen::Vector3d imu_rpy = robotis_framework::convertRotationToRPY(robotis_framework::getRotationX(M_PI) *
                                                                    imu_quaternion.toRotationMatrix() *
                                                                    robotis_framework::getRotationZ(M_PI));

  imu_data_mutex_lock_.unlock();

  // Set FT
  Eigen::MatrixXd robot_to_r_foot_force =
      robot_to_r_foot.block(0, 0, 3, 3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y,
                                          r_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd robot_to_r_foot_torque =
      robot_to_r_foot.block(0, 0, 3, 3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y,
                                          r_foot_ft_data_msg_.torque.z);

  Eigen::MatrixXd robot_to_l_foot_force =
      robot_to_l_foot.block(0, 0, 3, 3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y,
                                          l_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd robot_to_l_foot_torque =
      robot_to_l_foot.block(0, 0, 3, 3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y,
                                          l_foot_ft_data_msg_.torque.z);

  balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0, 0), imu_rpy.coeff(1, 0));

  balance_control_.setCurrentFootForceTorqueSensorOutput(
      robot_to_r_foot_force.coeff(0, 0), robot_to_r_foot_force.coeff(1, 0), robot_to_r_foot_force.coeff(2, 0),
      robot_to_r_foot_torque.coeff(0, 0), robot_to_r_foot_torque.coeff(1, 0), robot_to_r_foot_torque.coeff(2, 0),
      robot_to_l_foot_force.coeff(0, 0), robot_to_l_foot_force.coeff(1, 0), robot_to_l_foot_force.coeff(2, 0),
      robot_to_l_foot_torque.coeff(0, 0), robot_to_l_foot_torque.coeff(1, 0), robot_to_l_foot_torque.coeff(2, 0));

  balance_control_.setDesiredCOBGyro(0.0, 0.0);

  balance_control_.setDesiredCOBOrientation(des_body_rpy_[0], des_body_rpy_[1]);

  balance_control_.setDesiredFootForceTorque(balance_r_foot_force_x_, balance_r_foot_force_y_, balance_r_foot_force_z_,
                                             balance_r_foot_torque_x_, balance_r_foot_torque_y_,
                                             balance_r_foot_torque_z_, balance_l_foot_force_x_, balance_l_foot_force_y_,
                                             balance_l_foot_force_z_, balance_l_foot_torque_x_,
                                             balance_l_foot_torque_y_, balance_l_foot_torque_z_);

  balance_control_.setDesiredPose(robot_to_body, robot_to_r_foot, robot_to_l_foot);

  int error;
  Eigen::MatrixXd robot_to_body_mod, robot_to_r_foot_mod, robot_to_l_foot_mod;
  balance_control_.process(&error, &robot_to_body_mod, &robot_to_r_foot_mod, &robot_to_l_foot_mod);

  // ===== Transformation =====
  Eigen::Matrix4d body_pose_mod = body_pose * robot_to_body_mod;
  Eigen::Matrix4d r_foot_pose_mod = body_pose * robot_to_r_foot_mod;
  Eigen::Matrix4d l_foot_pose_mod = body_pose * robot_to_l_foot_mod;
  // =====

  Eigen::MatrixXd des_body_rot_mod = body_pose_mod.block<3, 3>(0, 0);
  Eigen::MatrixXd des_body_position_mod = body_pose_mod.block<3, 1>(0, 3);

  Eigen::MatrixXd des_r_foot_rot_mod = r_foot_pose_mod.block<3, 3>(0, 0);
  Eigen::MatrixXd des_r_foot_pos_mod = r_foot_pose_mod.block<3, 1>(0, 3);
  Eigen::MatrixXd des_l_foot_rot_mod = l_foot_pose_mod.block<3, 3>(0, 0);
  Eigen::MatrixXd des_l_foot_pos_mod = l_foot_pose_mod.block<3, 1>(0, 3);

  // Inverse Kinematics
  std::vector<double_t> r_leg_joint_pos, l_leg_joint_pos;
  r_leg_joint_pos.resize(6);
  l_leg_joint_pos.resize(6);

  Eigen::Vector3d des_r_foot_rpy_mod = robotis_framework::convertRotationToRPY(des_r_foot_rot_mod);
  Eigen::Vector3d des_l_foot_rpy_mod = robotis_framework::convertRotationToRPY(des_l_foot_rot_mod);
  std::vector<double_t> r_leg_pose, l_leg_pose;
  r_leg_pose.resize(6);
  l_leg_pose.resize(6);
  r_leg_pose[0] = des_r_foot_pos_mod.x();
  r_leg_pose[1] = des_r_foot_pos_mod.y();
  r_leg_pose[2] = des_r_foot_pos_mod.z();
  r_leg_pose[3] = des_r_foot_rpy_mod(0);
  r_leg_pose[4] = des_r_foot_rpy_mod(1);
  r_leg_pose[5] = des_r_foot_rpy_mod(2);

  l_leg_pose[0] = des_l_foot_pos_mod.x();
  l_leg_pose[1] = des_l_foot_pos_mod.y();
  l_leg_pose[2] = des_l_foot_pos_mod.z();
  l_leg_pose[3] = des_l_foot_rpy_mod(0);
  l_leg_pose[4] = des_l_foot_rpy_mod(1);
  l_leg_pose[5] = des_l_foot_rpy_mod(2);

  ik_success = (kuroko_kinematics_->solveInverseKinematicsForRightLeg(r_leg_pose, r_leg_joint_pos) &&
                kuroko_kinematics_->solveInverseKinematicsForLeftLeg(l_leg_pose, l_leg_joint_pos));

  if (ik_success)
  {
    des_joint_pos_[joint_name_to_dxl_id_["hip_r_roll"] - 1] = r_leg_joint_pos[0];
    des_joint_pos_[joint_name_to_dxl_id_["hip_r_pitch"] - 1] = r_leg_joint_pos[1];
    des_joint_pos_[joint_name_to_dxl_id_["thigh_r_active"] - 1] = r_leg_joint_pos[2];
    des_joint_pos_[joint_name_to_dxl_id_["shin_r_active"] - 1] = r_leg_joint_pos[3];
    des_joint_pos_[joint_name_to_dxl_id_["ankle_r_roll"] - 1] = r_leg_joint_pos[4];
    des_joint_pos_[joint_name_to_dxl_id_["ankle_r_yaw"] - 1] = r_leg_joint_pos[5];

    des_joint_pos_[joint_name_to_dxl_id_["hip_l_roll"] - 1] = l_leg_joint_pos[0];
    des_joint_pos_[joint_name_to_dxl_id_["hip_l_pitch"] - 1] = l_leg_joint_pos[1];
    des_joint_pos_[joint_name_to_dxl_id_["thigh_l_active"] - 1] = l_leg_joint_pos[2];
    des_joint_pos_[joint_name_to_dxl_id_["shin_l_active"] - 1] = l_leg_joint_pos[3];
    des_joint_pos_[joint_name_to_dxl_id_["ankle_l_roll"] - 1] = l_leg_joint_pos[4];
    des_joint_pos_[joint_name_to_dxl_id_["ankle_l_yaw"] - 1] = l_leg_joint_pos[5];
  }

  return ik_success;
}

void OnlineWalkingModule::setFeedbackControl()
{
  for (int i = 0; i < number_of_joints_; i++)
  {
    des_joint_pos_to_robot_[i] = des_joint_pos_[i] + des_joint_feedforward_[i];

    joint_feedback_[i].desired_ = des_joint_pos_[i];
    des_joint_feedback_[i] = joint_feedback_[i].getFeedBack(curr_joint_pos_[i]);

    des_joint_pos_to_robot_[i] += des_joint_feedback_[i];
  }
}

void OnlineWalkingModule::setFeedforwardControl()
{
  double cur_time = (double)mov_step_ * control_cycle_sec_;

  std::vector<double_t> feed_forward_value = feed_forward_trajectory_->getPosition(cur_time);

  if (walking_phase_ == DSP)
    feed_forward_value[0] = 0.0;

  std::vector<double_t> support_leg_gain;
  support_leg_gain.resize(number_of_joints_, 0.0);

  if (walking_leg_ == LEFT_LEG)
  {
    support_leg_gain[joint_name_to_dxl_id_["hip_r_roll"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["hip_r_pitch"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["thigh_r_active"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["shin_r_active"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["ankle_r_roll"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["ankle_r_yaw"] - 1] = 1.0;

    support_leg_gain[joint_name_to_dxl_id_["hip_l_roll"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["hip_l_pitch"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["thigh_l_active"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["shin_l_active"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["ankle_l_roll"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["ankle_l_yaw"] - 1] = 0.0;
  }
  else if (walking_leg_ == RIGHT_LEG)
  {
    support_leg_gain[joint_name_to_dxl_id_["hip_r_roll"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["hip_r_pitch"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["thigh_r_active"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["shin_r_active"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["ankle_r_roll"] - 1] = 0.0;
    support_leg_gain[joint_name_to_dxl_id_["ankle_r_yaw"] - 1] = 0.0;

    support_leg_gain[joint_name_to_dxl_id_["hip_l_roll"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["hip_l_pitch"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["thigh_l_active"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["shin_l_active"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["ankle_l_roll"] - 1] = 1.0;
    support_leg_gain[joint_name_to_dxl_id_["ankle_l_yaw"] - 1] = 1.0;
  }

  for (int i = 0; i < number_of_joints_; i++)
    des_joint_feedforward_[i] = joint_feedforward_gain_[i] * feed_forward_value[0] * support_leg_gain[i];
}

void OnlineWalkingModule::balanceFeedback(const double& roll_gyro_err, const double& pitch_gyro_err,
                                          std::vector<double>& balance_angle)
{
  // adjust balance offset
  balance_angle.resize(number_of_joints_, 0.0);

  balance_angle[joint_name_to_dxl_id_["hip_r_roll"] - 1] = -1.0 * roll_gyro_err * balance_hip_roll_gain_;
  balance_angle[joint_name_to_dxl_id_["thigh_r_active"] - 1] = 1.0 * pitch_gyro_err * balance_knee_gain_;
  balance_angle[joint_name_to_dxl_id_["shin_r_active"] - 1] = -1.0 * pitch_gyro_err * balance_ankle_pitch_gain_;
  balance_angle[joint_name_to_dxl_id_["ankle_r_roll"] - 1] = -1.0 * roll_gyro_err * balance_ankle_roll_gain_;
  balance_angle[joint_name_to_dxl_id_["hip_l_roll"] - 1] = -1.0 * roll_gyro_err * balance_hip_roll_gain_;
  balance_angle[joint_name_to_dxl_id_["thigh_l_active"] - 1] = -1.0 * pitch_gyro_err * balance_knee_gain_;
  balance_angle[joint_name_to_dxl_id_["shin_l_active"] - 1] = 1.0 * pitch_gyro_err * balance_ankle_pitch_gain_;
  balance_angle[joint_name_to_dxl_id_["ankle_l_roll"] - 1] = -1.0 * roll_gyro_err * balance_ankle_roll_gain_;
}

void OnlineWalkingModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                                  std::map<std::string, double> sensors)
{
  if (!enable_)
    return;

  std::vector<double> balance_angle;
  balance_angle.resize(number_of_joints_, 0.0);
  for (int i = 0; i < number_of_joints_; i++)
    balance_angle[i] = 0.0;

  double rl_gyro_err = 0.0 - sensors["gyro_x"];
  double fb_gyro_err = 0.0 - sensors["gyro_y"];

  balanceFeedback(rl_gyro_err, fb_gyro_err, balance_angle);

  /*----- write curr position -----*/
  for (auto& state_iter : result_)
  {
    std::string joint_name = state_iter.first;

    robotis_framework::Dynamixel* dxl = nullptr;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double curr_joint_pos = dxl->dxl_state_->present_position_;
    double goal_joint_pos = dxl->dxl_state_->goal_position_;

    if (!goal_initialize_)
      des_joint_pos_[joint_name_to_dxl_id_[joint_name] - 1] = goal_joint_pos;

    curr_joint_pos_[joint_name_to_dxl_id_[joint_name] - 1] = curr_joint_pos;
  }

  goal_initialize_ = true;

  /* Trajectory Calculation */
  ros::Time begin = ros::Time::now();

  if (control_type_ == JOINT_CONTROL)
  {
    initJointControl();
    calcJointControl();
  }
  else if (control_type_ == WHOLEBODY_CONTROL)
  {
    initWholebodyControl();
    calcWholebodyControl();
  }
  else if (control_type_ == WALKING_CONTROL)
  {
    if (walking_initialize_)
    {
      calcWalkingControl();
      setFeedforwardControl();
    }
  }
  else if (control_type_ == OFFSET_CONTROL)
  {
    initOffsetControl();
    calcOffsetControl();
  }

  //  calcRobotPose();

  if (balance_type_ == ON)
  {
    initBalanceControl();
    calcBalanceControl();

    if (!setBalanceControl())
    {
      is_moving_ = false;
      is_balancing_ = false;
      is_foot_step_2d_ = false;

      balance_type_ = OFF;
      control_type_ = NONE;

      resetBodyPose();

      ROS_INFO("[FAIL] Task Space Control");
    }
  }

  ros::Duration time_duration = ros::Time::now() - begin;

  if (time_duration.toSec() > 0.003)
    ROS_INFO("[Wholebody Module] Calc Time: %f", time_duration.toSec());

  setFeedbackControl();

  for (int i = 0; i < number_of_joints_; i++)
    des_joint_pos_to_robot_[i] += balance_angle[i];

  sensor_msgs::JointState goal_joint_msg;
  geometry_msgs::PoseStamped pelvis_pose_msg;

  goal_joint_msg.header.stamp = ros::Time::now();
  pelvis_pose_msg.header.stamp = ros::Time::now();

  pelvis_pose_msg.pose.position.x = des_body_position_[0];
  pelvis_pose_msg.pose.position.y = des_body_position_[1];
  pelvis_pose_msg.pose.position.z = des_body_position_[2] - 0.0907;

  pelvis_pose_msg.pose.orientation.x = des_body_rpy_[0];
  pelvis_pose_msg.pose.orientation.y = des_body_rpy_[1];
  pelvis_pose_msg.pose.orientation.z = des_body_rpy_[2];
  pelvis_pose_msg.pose.orientation.w = des_body_rpy_[3];

  /*----- set joint data -----*/
  for (auto& state_iter : result_)
  {
    std::string joint_name = state_iter.first;
    result_[joint_name]->goal_position_ = des_joint_pos_to_robot_[joint_name_to_dxl_id_[joint_name] - 1];
    goal_joint_msg.name.push_back(joint_name);
    goal_joint_msg.position.push_back(des_joint_pos_[joint_name_to_dxl_id_[joint_name] - 1]);
  }

  pelvis_pose_pub_.publish(pelvis_pose_msg);
  goal_joint_state_pub_.publish(goal_joint_msg);
}

void OnlineWalkingModule::stop()
{
  for (int i = 0; i < number_of_joints_; i++)
  {
    des_joint_pos_[i] = 0.0;
    des_joint_vel_[i] = 0.0;
    des_joint_accel_[i] = 0.0;
  }

  goal_initialize_ = false;

  is_moving_ = false;
  is_balancing_ = false;

  joint_control_initialize_ = false;
  wholebody_initialize_ = false;
  walking_initialize_ = false;
  balance_control_initialize_ = false;

  control_type_ = NONE;

  return;
}

bool OnlineWalkingModule::isRunning()
{
  return is_moving_;
}

void OnlineWalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type = type;
  status.module_name = "Wholebody";
  status.status_msg = std::move(msg);

  status_msg_pub_.publish(status);
}

bool OnlineWalkingModule::getJointPoseCallback(op3_online_walking_module_msgs::GetJointPose::Request& /*req*/,
                                               op3_online_walking_module_msgs::GetJointPose::Response& res)
{
  for (int i = 0; i < number_of_joints_; i++)
  {
    res.pose.pose.name.push_back(joint_name_[i]);
    res.pose.pose.position.push_back(des_joint_pos_[i]);
  }

  return true;
}

bool OnlineWalkingModule::getKinematicsPoseCallback(op3_online_walking_module_msgs::GetKinematicsPose::Request& req,
                                                    op3_online_walking_module_msgs::GetKinematicsPose::Response& res)
{
  std::string group_name = req.name;

  geometry_msgs::Pose msg;

  if (group_name == "body")
  {
    msg.position.x = des_body_position_[0];
    msg.position.y = des_body_position_[1];
    msg.position.z = des_body_position_[2];

    msg.orientation.x = des_body_rpy_[0];
    msg.orientation.y = des_body_rpy_[1];
    msg.orientation.z = des_body_rpy_[2];
    msg.orientation.w = des_body_rpy_[3];
  }
  else if (group_name == "left_leg")
  {
    msg.position.x = des_l_leg_position_[0];
    msg.position.y = des_l_leg_position_[1];
    msg.position.z = des_l_leg_position_[2];

    msg.orientation.x = des_l_leg_rpy_[0];
    msg.orientation.y = des_l_leg_rpy_[1];
    msg.orientation.z = des_l_leg_rpy_[2];
    msg.orientation.w = des_l_leg_rpy_[3];
  }
  else if (group_name == "right_leg")
  {
    msg.position.x = des_r_leg_position_[0];
    msg.position.y = des_r_leg_position_[1];
    msg.position.z = des_r_leg_position_[2];

    msg.orientation.x = des_r_leg_rpy_[0];
    msg.orientation.y = des_r_leg_rpy_[1];
    msg.orientation.z = des_r_leg_rpy_[2];
    msg.orientation.w = des_r_leg_rpy_[3];
  }

  res.pose.pose = msg;

  return true;
}

bool OnlineWalkingModule::definePreviewMatrix()
{
  std::vector<double_t> k;
  k.push_back(739.200064);
  k.push_back(24489.822984);
  k.push_back(3340.410380);
  k.push_back(69.798325);

  preview_response_k_ = k;
  preview_response_k_row_ = 1;
  preview_response_k_col_ = 4;

  std::vector<double_t> p;
  p.push_back(33.130169);
  p.push_back(531.738962);
  p.push_back(60.201291);
  p.push_back(0.327533);
  p.push_back(531.738962);
  p.push_back(10092.440286);
  p.push_back(1108.851055);
  p.push_back(7.388990);
  p.push_back(60.201291);
  p.push_back(1108.851055);
  p.push_back(130.194694);
  p.push_back(0.922502);
  p.push_back(0.327533);
  p.push_back(7.388990);
  p.push_back(0.922502);
  p.push_back(0.012336);

  preview_response_p_ = p;
  preview_response_p_row_ = 4;
  preview_response_p_col_ = 4;

  return true;
}
