#include <utility>
#include <vector>
#include "robotis_math/robotis_linear_algebra.h"
#include "kuroko_online_walking_module/online_walking_module.h"
using namespace motion_control;

OnlineWalkingModule::OnlineWalkingModule()
  : control_cycle_sec_(0.008)
  , is_robot_moving_(false)
  , is_balance_active_(false)
  , is_offset_adjusting_(false)
  , is_goal_initialized_(false)
  , is_balance_control_initialized_(false)
  , is_body_offset_initialized_(false)
  , is_joint_control_initialized_(false)
  , is_wholebody_control_initialized_(false)
  , is_walking_control_initialized_(false)
  , is_footstep_2d_active_(false)
  , walking_phase_(DSP)
  , robot_mass_(4.0)
  , foot_distance_(0.07)
  , control_type_(NONE)
  , balance_type_(OFF)
{
  ROS_INFO("[OnlineWalkingModule::OnlineWalkingModule()]");
  module_name_ = "online_walking_module";
  enable_ = false;
  control_mode_ = robotis_framework::PositionControl;
  initializeKinematics();
  initializeLegJointNames();
  initializeWalkingParameters();
  initializeBodyOffsets();
  initializeJointStates();
  initializeMotionControl();
  loadParametersFromYAML();
  initializeBalanceControl();
  resetBodyPose();
}
OnlineWalkingModule::~OnlineWalkingModule()
{
  ROS_INFO("[OnlineWalkingModule::~OnlineWalkingModule()]");
  if (queue_thread_.joinable())
    queue_thread_.join();
}

void OnlineWalkingModule::initializeLegJointNames()
{
  joint_name_ = { "hip_r_roll", "hip_r_pitch", "thigh_r_active", "shin_r_active", "ankle_r_roll", "ankle_r_yaw",
                  "hip_l_roll", "hip_l_pitch", "thigh_l_active", "shin_l_active", "ankle_l_roll", "ankle_l_yaw" };
}

void OnlineWalkingModule::loadParametersFromYAML()
{
  std::string package_path = ros::package::getPath("kuroko_online_walking_module") + "/config/";

  parseBalanceGainData(package_path + "balance_gain.yaml");
  parseJointFeedbackGainData(package_path + "joint_feedback_gain.yaml");
  parseJointFeedforwardGainData(package_path + "joint_feedforward_gain.yaml");
}

void OnlineWalkingModule::initializeKinematics()
{
  kuroko_kinematics_ = new KurokoKinematics(WHOLE_BODY);
  if (!kuroko_kinematics_)
  {
    ROS_ERROR("Failed to initialize KurokoKinematics");
    return;
  }
  leg_default_length_ = kuroko_kinematics_->leg_max_height_;
  leg_default_separaion_ = kuroko_kinematics_->leg_side_offset_;
}

void OnlineWalkingModule::initializeWalkingParameters()
{
  online_walking_param_.dsp_ratio = 0.1;
  online_walking_param_.lipm_height = 0.25;
  online_walking_param_.foot_height_max = 0.05;
  online_walking_param_.zmp_offset_x = 0.0;
  online_walking_param_.zmp_offset_y = 0.0;
  foot_distance_ = leg_default_separaion_ + 0.13;
  robot_mass_ = 4.0;
}

void OnlineWalkingModule::initializeBodyOffsets()
{
  curr_body_offset_.resize(3, 0.0);
  goal_body_offset_.resize(3, 0.0);
}

void OnlineWalkingModule::initializeJointStates()
{
  for (const auto& joint_name : joint_name_)
  {
    result_[joint_name] = new robotis_framework::DynamixelState();
  }

  std::vector<std::vector<double>*> buffers = { &motor_curr_acc_,
                                                &motor_curr_vel_,
                                                &motor_curr_pos_,
                                                &motor_target_acc_,
                                                &motor_target_vel_,
                                                &motor_target_pos_,
                                                &motor_goal_acc_,
                                                &motor_goal_vel_,
                                                &motor_goal_pos_,
                                                &command_joint_feedback_,
                                                &command_joint_feedforward_,
                                                &command_joint_pos_,
                                                &joint_feedforward_gain_ };

  for (auto& buffer : buffers)
  {
    buffer->resize(joint_name_.size(), 0.0);
  }
}

void OnlineWalkingModule::initializeMotionControl()
{
  body_target_pos_.resize(3, 0.0);
  body_target_rpy_.resize(3, 0.0);

  l_leg_target_pos_.resize(3, 0.0);
  l_leg_target_rpy_.resize(3, 0.0);

  r_leg_target_pos_.resize(3, 0.0);
  r_leg_target_rpy_.resize(3, 0.0);
}

void OnlineWalkingModule::initializeBalanceControl()
{
  lipm_x_.resize(3, 0.0);
  lipm_y_.resize(3, 0.0);
  curr_balance_gain_ratio_.resize(1, 0.0);
  goal_balance_gain_ratio_.resize(1, 0.0);

  balance_l_foot_force_x_ = balance_l_foot_force_y_ = balance_l_foot_force_z_ = 0.0;
  balance_l_foot_torque_x_ = balance_l_foot_torque_y_ = balance_l_foot_torque_z_ = 0.0;
  balance_r_foot_force_x_ = balance_r_foot_force_y_ = balance_r_foot_force_z_ = 0.0;
  balance_r_foot_torque_x_ = balance_r_foot_torque_y_ = balance_r_foot_torque_z_ = 0.0;

  balance_control_.initialize(control_cycle_sec_ * 1000.0);
  balance_control_.setGyroBalanceEnable(false);
  balance_control_.setOrientationBalanceEnable(false);
  balance_control_.setForceTorqueBalanceEnable(false);
}

int OnlineWalkingModule::getJointIndex(const std::string& joint_name)
{
  for (int i = 0; i < joint_name_.size(); i++)
  {
    if (joint_name == joint_name_[i])
      return i;
  }
  return -1;
}

void OnlineWalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot* /*robot*/)
{
  ROS_INFO("[OnlineWalkingModule::initialize]");
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&OnlineWalkingModule::queueThread, this));

  ros::NodeHandle ros_node;

  // Publisher
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/motion_control/status", 1);
  movement_done_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/movement_done", 1);
  goal_joint_state_pub_ =
      ros_node.advertise<sensor_msgs::JointState>("/motion_control/online_walking/goal_joint_states", 1);
  pelvis_pose_pub_ = ros_node.advertise<geometry_msgs::PoseStamped>("/motion_control/pelvis_pose", 1);
  world_frame_id_ = "world";
  robot_frame_id_ = "body_link";

  // Service
  //  get_preview_matrix_client_ =
  //  ros_node.serviceClient<KUROKO_ONLINE_WALKING_MODULE_msgs::GetPreviewMatrix>("/motion_control/online_walking/get_preview_matrix",
  //  0);
}

void OnlineWalkingModule::onModuleEnable()
{
  ROS_INFO("[OnlineWalkingModule] Module Enabled");
  is_robot_moving_ = false;
  is_balance_active_ = false;
  is_footstep_2d_active_ = false;
  balance_type_ = OFF;
  control_type_ = NONE;
  resetBodyPose();
  // Initial pose
  // initJointControl();
  // runJointControl();
}

void OnlineWalkingModule::onModuleDisable()
{
  ROS_INFO("[OnlineWalkingModule] Module Disabled");
  enable_ = false;
  is_robot_moving_ = false;
  is_balance_active_ = false;
  is_footstep_2d_active_ = false;
  balance_type_ = OFF;
  control_type_ = NONE;
  resetBodyPose();
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
  ros::Subscriber footstep_command_sub = ros_node.subscribe("/motion_control/online_walking/footstep_command", 5,
                                                            &OnlineWalkingModule::footStepCommandCallback, this);
  ros::Subscriber online_walking_param_sub = ros_node.subscribe("/motion_control/online_walking/walking_param", 5,
                                                                &OnlineWalkingModule::onlineWalkingParamCallback, this);
  // ros::Subscriber walking_param_sub =
  //     ros_node.subscribe("/motion_control/walking/set_params", 5, &OnlineWalkingModule::walkingParamCallback, this);
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
  ROS_INFO("OnlineWalkingModule::resetBodyPose");

  // wholebody_control_->getTaskPosition(l_leg_target_pos_, r_leg_target_pos_, body_target_pos_);
  // wholebody_control_->getTaskOrientation(l_leg_target_rpy_, r_leg_target_rpy_, body_target_rpy_);
  // TODO: Set body height offset from lipm height
  body_target_pos_[0] = 0.0;
  body_target_pos_[1] = 0.0;
  body_target_pos_[2] = online_walking_param_.lipm_height;

  body_target_rpy_[0] = 0.0;
  body_target_rpy_[1] = 0.0;
  body_target_rpy_[2] = 0.0;

  r_leg_target_pos_[0] = 0.0;
  r_leg_target_pos_[1] = -0.5 * foot_distance_;
  r_leg_target_pos_[2] = 0.0;

  r_leg_target_rpy_[0] = 0.0;
  r_leg_target_rpy_[1] = 0.0;
  r_leg_target_rpy_[2] = 0.0;

  l_leg_target_pos_[0] = 0.0;
  l_leg_target_pos_[1] = 0.5 * foot_distance_;
  l_leg_target_pos_[2] = 0.0;

  l_leg_target_rpy_[0] = 0.0;
  l_leg_target_rpy_[1] = 0.0;
  l_leg_target_rpy_[2] = 0.0;

  lipm_x_[0] = body_target_pos_[0];
  lipm_x_[1] = 0.0;
  lipm_x_[2] = 0.0;

  lipm_y_[0] = body_target_pos_[1];
  lipm_y_[1] = 0.0;
  lipm_y_[2] = 0.0;

  online_walking_param_.zmp_offset_x = body_target_pos_[0];
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

  if (!doc["foot_roll_gyro_p_gain"])
    ROS_ERROR("[ERROR] foot_roll_gyro_p_gain not found in YAML!");
  if (!doc["foot_roll_gyro_d_gain"])
    ROS_ERROR("[ERROR] foot_roll_gyro_d_gain not found in YAML!");
  ROS_INFO("[DEBUG] foot_roll_gyro_p_gain_: %f", doc["foot_roll_gyro_p_gain"].as<double>());
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

  joint_feedback_[getJointIndex("hip_r_roll")].p_gain_ = doc["hip_r_roll_p_gain"].as<double>();
  joint_feedback_[getJointIndex("hip_r_pitch")].p_gain_ = doc["hip_r_pitch_p_gain"].as<double>();
  joint_feedback_[getJointIndex("thigh_r_active")].p_gain_ = doc["thigh_r_active_p_gain"].as<double>();
  joint_feedback_[getJointIndex("shin_r_active")].p_gain_ = doc["shin_r_active_p_gain"].as<double>();
  joint_feedback_[getJointIndex("ankle_r_roll")].p_gain_ = doc["ankle_r_roll_p_gain"].as<double>();
  joint_feedback_[getJointIndex("ankle_r_yaw")].p_gain_ = doc["ankle_r_yaw_p_gain"].as<double>();

  joint_feedback_[getJointIndex("hip_r_roll")].d_gain_ = doc["hip_r_roll_d_gain"].as<double>();
  joint_feedback_[getJointIndex("hip_r_pitch")].d_gain_ = doc["hip_r_pitch_d_gain"].as<double>();
  joint_feedback_[getJointIndex("thigh_r_active")].d_gain_ = doc["thigh_r_active_d_gain"].as<double>();
  joint_feedback_[getJointIndex("shin_r_active")].d_gain_ = doc["shin_r_active_d_gain"].as<double>();
  joint_feedback_[getJointIndex("ankle_r_roll")].d_gain_ = doc["ankle_r_roll_d_gain"].as<double>();
  joint_feedback_[getJointIndex("ankle_r_yaw")].d_gain_ = doc["ankle_r_yaw_d_gain"].as<double>();

  joint_feedback_[getJointIndex("hip_l_roll")].p_gain_ = doc["hip_l_roll_p_gain"].as<double>();
  joint_feedback_[getJointIndex("hip_l_pitch")].p_gain_ = doc["hip_l_pitch_p_gain"].as<double>();
  joint_feedback_[getJointIndex("thigh_l_active")].p_gain_ = doc["thigh_l_active_p_gain"].as<double>();
  joint_feedback_[getJointIndex("shin_l_active")].p_gain_ = doc["shin_l_active_p_gain"].as<double>();
  joint_feedback_[getJointIndex("ankle_l_roll")].p_gain_ = doc["ankle_l_roll_p_gain"].as<double>();
  joint_feedback_[getJointIndex("ankle_l_yaw")].p_gain_ = doc["ankle_l_yaw_p_gain"].as<double>();

  joint_feedback_[getJointIndex("hip_l_roll")].d_gain_ = doc["hip_l_roll_d_gain"].as<double>();
  joint_feedback_[getJointIndex("hip_l_pitch")].d_gain_ = doc["hip_l_pitch_d_gain"].as<double>();
  joint_feedback_[getJointIndex("thigh_l_active")].d_gain_ = doc["thigh_l_active_d_gain"].as<double>();
  joint_feedback_[getJointIndex("shin_l_active")].d_gain_ = doc["shin_l_active_d_gain"].as<double>();
  joint_feedback_[getJointIndex("ankle_l_roll")].d_gain_ = doc["ankle_l_roll_d_gain"].as<double>();
  joint_feedback_[getJointIndex("ankle_l_yaw")].d_gain_ = doc["ankle_l_yaw_d_gain"].as<double>();
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

  joint_feedforward_gain_[getJointIndex("hip_r_roll")] = doc["hip_r_roll_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("hip_r_pitch")] = doc["hip_r_pitch_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("thigh_r_active")] = doc["thigh_r_active_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("shin_r_active")] = doc["shin_r_active_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("ankle_r_roll")] = doc["ankle_r_roll_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("ankle_r_yaw")] = doc["ankle_r_yaw_gain"].as<double>();

  joint_feedforward_gain_[getJointIndex("hip_l_roll")] = doc["hip_l_roll_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("hip_l_pitch")] = doc["hip_l_pitch_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("thigh_l_active")] = doc["thigh_l_active_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("shin_l_active")] = doc["shin_l_active_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("ankle_l_roll")] = doc["ankle_l_roll_gain"].as<double>();
  joint_feedforward_gain_[getJointIndex("ankle_l_yaw")] = doc["ankle_l_yaw_gain"].as<double>();
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
  {
    balance_type_ = ON;
    goal_balance_gain_ratio_[0] = 1.0;
  }
  else if (msg->data == "balance_off")
  {
    balance_type_ = OFF;
    goal_balance_gain_ratio_[0] = 0.0;
  }
  else
  {
    ROS_ERROR("[ERROR] Invalid balance type: %s (balance_on, balance_off)", msg->data.c_str());
    return;
  }

  is_balance_control_initialized_ = false;
  walking_phase_ = DSP;
}

void OnlineWalkingModule::initBalanceGain()
{
  if (!enable_ || is_balance_control_initialized_)
    return;
  if (control_type_ == NONE)
    control_type_ = APPLY_BALANCE_GAIN;
  if (control_type_ != APPLY_BALANCE_GAIN)
    return;
  double ini_time = 0.0;
  double mov_time = 1.0;

  balance_step_ = 0;
  balance_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  std::vector<double_t> balance_zero;
  balance_zero.resize(1, 0.0);

  balance_trajectory_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time, curr_balance_gain_ratio_, balance_zero, balance_zero,
                                         goal_balance_gain_ratio_, balance_zero, balance_zero);

  if (is_balance_active_)
    ROS_INFO("[UPDATE] Balance Gain");
  else
  {
    is_balance_active_ = true;
    ROS_INFO("[START] Balance Gain");
  }
  is_balance_control_initialized_ = true;
}

void OnlineWalkingModule::applyBalanceGain()
{
  if (!enable_ || !is_balance_active_)
    return;
  if (control_type_ != APPLY_BALANCE_GAIN || !is_balance_control_initialized_)
    return;

  if (balance_trajectory_ == nullptr)
  {
    ROS_ERROR("[ERROR] balance_trajectory_ is NULL!");
    delete balance_trajectory_;
    return;
  }
  double cur_time = (double)balance_step_ * control_cycle_sec_;
  curr_balance_gain_ratio_ = balance_trajectory_->getPosition(cur_time);

  if (balance_step_ == balance_size_ - 1)
  {
    balance_step_ = 0;
    is_balance_active_ = false;
    delete balance_trajectory_;

    if (curr_balance_gain_ratio_[0] == 0.0)
    {
      balance_type_ = OFF;
    }
    control_type_ = NONE;
    ROS_INFO("[END] Balance Gain");
  }
  else
    balance_step_++;
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
    // TODO: Edit Initial pose
    // curr_body_offset_[0] = -0.02;
    // curr_body_offset_[1] = 0.0;
    // curr_body_offset_[2] = 0.02;
    resetBodyPose();
  }
}

// void OnlineWalkingModule::walkingParamCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg)
// {
//   ROS_INFO("OnlineWalkingModule::walkingParamCallback");
//   walking_param_ = *msg;
// }

void OnlineWalkingModule::goalJointPoseCallback(const op3_online_walking_module_msgs::JointPose& msg)
{
  if (!enable_)
    return;
  if (control_type_ == NONE)
    control_type_ = JOINT_CONTROL;
  if (control_type_ != JOINT_CONTROL)
  {
    ROS_WARN("[OnlineWalkingModule::goalJointPoseCallback] Control type is different!");
    return;
  }

  size_t joint_size = msg.pose.name.size();
  mov_time_ = msg.mov_time;
  for (size_t i = 0; i < msg.pose.name.size(); i++)
  {
    std::string name = msg.pose.name[i];
    int index = getJointIndex(name);
    if (index != -1)
      motor_goal_pos_[index] = msg.pose.position[i];
    else
      ROS_WARN("[OnlineWalkingModule::goalJointPoseCallback] Invalid joint name: %s", name.c_str());
  }
  is_joint_control_initialized_ = false;
  control_type_ = JOINT_CONTROL;
  balance_type_ = OFF;
  curr_balance_gain_ratio_[0] = 0.0;
}

void OnlineWalkingModule::initJointControl()
{
  if (!enable_)
    return;
  if (control_type_ == NONE)
    control_type_ = JOINT_CONTROL;
  if (control_type_ != JOINT_CONTROL)
  {
    ROS_WARN("[OnlineWalkingModule::initJointControl]: Control type is different!");
    return;
  }
  if (is_joint_control_initialized_)
  {
    // ROS_WARN("[OnlineWalkingModule::initJointControl]: Already initialized!");
    return;
  }

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  joint_trajectory_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time, motor_target_pos_, motor_target_vel_, motor_target_acc_,
                                         motor_goal_pos_, motor_goal_vel_, motor_goal_acc_);
  if (is_robot_moving_)
    ROS_INFO("[OnlineWalkingModule::initJointControl]: UPDATE Joint Trajectory Goal");
  else
  {
    is_robot_moving_ = true;
    ROS_INFO("[OnlineWalkingModule::initJointControl]: SET Joint Trajectory Goal");
  }
  is_joint_control_initialized_ = true;
}

void OnlineWalkingModule::runJointControl()
{
  if (!enable_ || !is_joint_control_initialized_ || control_type_ != JOINT_CONTROL)
    return;
  if (is_robot_moving_)
  {
    double cur_time = (double)mov_step_ * control_cycle_sec_;
    // Set values
    motor_target_pos_ = joint_trajectory_->getPosition(cur_time);
    motor_target_vel_ = joint_trajectory_->getVelocity(cur_time);
    motor_target_acc_ = joint_trajectory_->getAcceleration(cur_time);
    queue_mutex_.unlock();

    if (mov_step_ == mov_size_ - 1)
    {
      mov_step_ = 0;
      is_robot_moving_ = false;
      delete joint_trajectory_;
      control_type_ = NONE;
      ROS_INFO("[OnlineWalkingModule::runJointControl]: END Joint Control");
      resetBodyPose();
    }
    else
      mov_step_++;
  }
}

void OnlineWalkingModule::onlineWalkingParamCallback(const op3_online_walking_module_msgs::WalkingParam& msg)
{
  ROS_INFO("OnlineWalkingModule::onlineWalkingParamCallback");
  online_walking_param_ = msg;  // TODO: Edit params
  resetBodyPose();
}

void OnlineWalkingModule::setBodyOffsetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  ROS_INFO("OnlineWalkingModule::setBodyOffsetCallback");
  if (!enable_)
    return;
  if (control_type_ == NONE)
    control_type_ = APPLY_BODY_OFFSET;
  if (control_type_ != APPLY_BODY_OFFSET)
  {
    ROS_WARN("[OnlineWalkingModule::setBodyOffsetCallback]: Control type is different!");
    return;
  }

  // if (balance_type_ == OFF)
  // {
  //   ROS_WARN("[WARN] Balance is off!");
  //   return;
  // }

  goal_body_offset_[0] = msg->position.x;
  goal_body_offset_[1] = msg->position.y;
  goal_body_offset_[2] = msg->position.z + leg_default_length_;
  ROS_INFO("goal_body_offset: %f, %f, %f", goal_body_offset_[0], goal_body_offset_[1], goal_body_offset_[2]);
  is_body_offset_initialized_ = false;
  resetBodyPose();
}

void OnlineWalkingModule::setFootDistanceCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("OnlineWalkingModule::setFootDistanceCallback");
  if (!enable_)
    return;

  foot_distance_ = msg->data + leg_default_separaion_;
  resetBodyPose();
}

void OnlineWalkingModule::initBodyOffset()
{
  if (!enable_ || is_body_offset_initialized_)
    return;
  if (control_type_ == NONE)
    control_type_ = APPLY_BODY_OFFSET;
  if (control_type_ != APPLY_BODY_OFFSET)
  {
    ROS_WARN("[OnlineWalkingModule::initBodyOffset] Control type is different!");
    return;
  }

  double ini_time = 0.0;
  double mov_time = 1.0;

  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  std::vector<double_t> offset_zero;
  offset_zero.resize(3, 0.0);

  body_offset_trajectory_ = new robotis_framework::MinimumJerk(
      ini_time, mov_time, curr_body_offset_, offset_zero, offset_zero, goal_body_offset_, offset_zero, offset_zero);

  if (is_robot_moving_)
    ROS_INFO("[UPDATE] Body Offset");
  else
  {
    is_robot_moving_ = true;
    ROS_INFO("[START] Body Offset");
  }
  is_body_offset_initialized_ = true;
}

void OnlineWalkingModule::applyBodyOffset()
{
  if (!enable_ || control_type_ != APPLY_BODY_OFFSET || !is_body_offset_initialized_)
  {
    return;
  }
  if (is_robot_moving_)
  {
    double cur_time = (double)mov_step_ * control_cycle_sec_;
    queue_mutex_.lock();
    curr_body_offset_ = body_offset_trajectory_->getPosition(cur_time);
    queue_mutex_.unlock();

    if (mov_step_ == mov_size_ - 1)
    {
      mov_step_ = 0;
      is_robot_moving_ = false;
      delete body_offset_trajectory_;
      control_type_ = NONE;
      ROS_INFO("[END] Body Offset");
    }
    else
      mov_step_++;
  }
}

void OnlineWalkingModule::goalKinematicsPoseCallback(const op3_online_walking_module_msgs::KinematicsPose& msg)
{
  if (!enable_)
    return;

  // if (balance_type_ == OFF)
  // {
  //   ROS_WARN("[WARN] Balance is off!");
  //   return;
  // }

  if (control_type_ == NONE)
    control_type_ = WHOLEBODY_CONTROL;
  if (control_type_ != WHOLEBODY_CONTROL)
  {
    ROS_WARN("[OnlineWalkingModule::goalKinematicsPoseCallback] Control type is different!");
    return;
  }

  if (is_robot_moving_)
  {
    if (wholegbody_control_group_ != msg.name)
    {
      ROS_WARN("[OnlineWalkingModule::goalKinematicsPoseCallback] Control group is different!");
      return;
    }
  }
  mov_time_ = msg.mov_time;
  wholegbody_control_group_ = msg.name;
  wholebody_goal_msg_ = msg.pose;
  is_wholebody_control_initialized_ = false;
}

void OnlineWalkingModule::initWholebodyControl()
{
  if (!enable_)
    return;
  if (control_type_ == NONE)
    control_type_ = APPLY_BODY_OFFSET;
  if (control_type_ != WHOLEBODY_CONTROL)
  {
    ROS_WARN("[OnlineWalkingModule::initWholebodyControl] Control type is different!");
    return;
  }
  if (is_wholebody_control_initialized_)
  {
    // ROS_WARN("[OnlineWalkingModule::initWholebodyControl] Already initialized!");
    return;
  }
  if (is_robot_moving_)
  {
    ROS_WARN("[OnlineWalkingModule::initWholebodyControl] Previous task is alive!");
    return;
  }

  is_wholebody_control_initialized_ = true;
  double ini_time = 0.0;
  double mov_time = mov_time_;
  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;
  wholebody_control_ = new WholebodyControl(wholegbody_control_group_, ini_time, mov_time, wholebody_goal_msg_);

  ROS_INFO("[START] Wholebody Control");
  wholebody_control_->initialize(body_target_pos_, body_target_rpy_, r_leg_target_pos_, r_leg_target_rpy_,
                                 l_leg_target_pos_, l_leg_target_rpy_);
  is_robot_moving_ = true;
}

void OnlineWalkingModule::runWholebodyControl()
{
  if (!enable_ || control_type_ != WHOLEBODY_CONTROL)
    return;
  if (is_robot_moving_)
  {
    double cur_time = (double)mov_step_ * control_cycle_sec_;
    wholebody_control_->set(cur_time);
    wholebody_control_->getTaskPosition(l_leg_target_pos_, r_leg_target_pos_, body_target_pos_);
    wholebody_control_->getTaskOrientation(l_leg_target_rpy_, r_leg_target_rpy_, body_target_rpy_);
    if (mov_step_ == mov_size_ - 1)
    {
      mov_step_ = 0;
      is_robot_moving_ = false;
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
  if (control_type_ == NONE)
    control_type_ = WALKING_CONTROL;
  if (control_type_ != WALKING_CONTROL)
  {
    ROS_WARN("[OnlineWalkingModule::footStep2DCallback] Control type is different!");
    return;
  }
  if (is_robot_moving_)
  {
    ROS_WARN("[OnlineWalkingModule::footStep2DCallback] Previous task is alive!");
    return;
  }
  // if (balance_type_ == OFF)
  // {
  //   ROS_WARN("[WARN] Balance is off!");
  //   return;
  // }

  Eigen::MatrixXd body_r =
      robotis_framework::convertRPYToRotation(body_target_rpy_[0], body_target_rpy_[1], body_target_rpy_[2]);
  Eigen::MatrixXd body_t = Eigen::MatrixXd::Identity(4, 4);
  body_t.block(0, 0, 3, 3) = body_r;
  body_t.coeffRef(0, 3) = body_target_pos_[0];
  body_t.coeffRef(1, 3) = body_target_pos_[1];

  op3_online_walking_module_msgs::Step2DArray footstep_msg;

  int old_size = msg.footsteps_2d.size();
  int new_size = old_size + 3;

  op3_online_walking_module_msgs::Step2D first_msg;
  op3_online_walking_module_msgs::Step2D second_msg;

  first_msg.moving_foot = msg.footsteps_2d[0].moving_foot - 1;
  second_msg.moving_foot = first_msg.moving_foot + 1;

  if (first_msg.moving_foot == LEFT_LEG)
  {
    first_msg.step2d.x = l_leg_target_pos_[0];
    first_msg.step2d.y = l_leg_target_pos_[1];
    first_msg.step2d.theta = body_target_rpy_[2];

    second_msg.step2d.x = r_leg_target_pos_[0];
    second_msg.step2d.y = r_leg_target_pos_[1];
    second_msg.step2d.theta = body_target_rpy_[2];
  }
  else if (first_msg.moving_foot == RIGHT_LEG)
  {
    first_msg.step2d.x = r_leg_target_pos_[0];
    first_msg.step2d.y = r_leg_target_pos_[1];
    first_msg.step2d.theta = body_target_rpy_[2];

    second_msg.step2d.x = l_leg_target_pos_[0];
    second_msg.step2d.y = l_leg_target_pos_[1];
    second_msg.step2d.theta = body_target_rpy_[2];
  }

  footstep_msg.footsteps_2d.push_back(first_msg);
  footstep_msg.footsteps_2d.push_back(second_msg);

  double step_final_theta = 0.0;

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

    footstep_msg.footsteps_2d.push_back(step_msg);
  }

  op3_online_walking_module_msgs::Step2D step_msg = msg.footsteps_2d[old_size - 1];

  if (step_msg.moving_foot - 1 == LEFT_LEG)
    first_msg.moving_foot = RIGHT_LEG;
  else
    first_msg.moving_foot = LEFT_LEG;

  first_msg.step2d.x = 0.0;
  first_msg.step2d.y = 0.0;
  first_msg.step2d.theta = step_final_theta;  // step_msg.step2d.theta;

  footstep_msg.footsteps_2d.push_back(first_msg);

  footstep_2d_ = footstep_msg;
  footstep_2d_.step_time = msg.step_time;

  walking_size_ = new_size;
  mov_time_ = msg.step_time;  // 1.0;
  is_footstep_2d_active_ = true;
  control_type_ = WALKING_CONTROL;

  initWalkingControl();
}

void OnlineWalkingModule::footStepCommandCallback(const op3_online_walking_module_msgs::FootStepCommand& msg)
{
  if (!enable_)
    return;
  if (control_type_ == NONE)
    control_type_ = WALKING_CONTROL;
  if (control_type_ != WALKING_CONTROL)
  {
    ROS_WARN("[OnlineWalkingModule::footStepCommandCallback] Control type is different!");
    return;
  }
  if (is_robot_moving_)
  {
    ROS_WARN("[OnlineWalkingModule::footStepCommandCallback] Previous task is alive!");
    return;
  }
  // if (balance_type_ == OFF)
  // {
  //   ROS_WARN("[WARN] Balance is off!");
  //   return;
  // }

  is_footstep_2d_active_ = false;

  walking_size_ = msg.step_num + 3;  // msg.step_num + 2;
  mov_time_ = msg.step_time;
  footstep_command_ = msg;
  footstep_command_.step_num = walking_size_;

  initWalkingControl();
}

void OnlineWalkingModule::initWalkingControl()
{
  if (is_robot_moving_)
  {
    ROS_WARN("[OnlineWalkingModule::initWalkingControl] Previous task is alive!");
    return;
  }
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  walking_step_ = 0;

  walking_control_ =
      new WalkingControl(control_cycle_sec_, online_walking_param_.dsp_ratio, online_walking_param_.lipm_height,
                         online_walking_param_.foot_height_max, online_walking_param_.zmp_offset_x,
                         online_walking_param_.zmp_offset_y, lipm_x_, lipm_y_, foot_distance_);

  double lipm_height = online_walking_param_.lipm_height;
  preview_request_.lipm_height = lipm_height;
  preview_request_.control_cycle = control_cycle_sec_;

  bool get_preview_matrix = false;
  get_preview_matrix = definePreviewMatrix();

  if (get_preview_matrix)
  {
    ROS_INFO("[OnlineWalkingModule::initWalkingControl] Get preview matrix");
    if (is_footstep_2d_active_)
    {
      walking_control_->initialize(footstep_2d_, body_target_pos_, body_target_rpy_, r_leg_target_pos_,
                                   r_leg_target_rpy_, l_leg_target_pos_, l_leg_target_rpy_);
    }
    else
    {
      walking_control_->initialize(footstep_command_, body_target_pos_, body_target_rpy_, r_leg_target_pos_,
                                   r_leg_target_rpy_, l_leg_target_pos_, l_leg_target_rpy_);
    }

    walking_control_->calcPreviewParam(preview_response_k_, preview_response_k_row_, preview_response_k_col_,
                                       preview_response_p_, preview_response_p_row_, preview_response_p_row_);
    initFeedforwardControl();
    is_robot_moving_ = true;
    is_walking_control_initialized_ = true;
  }
  else
    ROS_WARN("[OnlineWalkingModule::initWalkingControl] Cannot get preview matrix");
}

void OnlineWalkingModule::runWalkingControl()
{
  if (is_robot_moving_)
  {
    double cur_time = (double)mov_step_ * control_cycle_sec_;
    walking_control_->set(cur_time, walking_step_, is_footstep_2d_active_);
    walking_control_->getWalkingPosition(l_leg_target_pos_, r_leg_target_pos_, body_target_pos_);
    walking_control_->getWalkingOrientation(l_leg_target_rpy_, r_leg_target_rpy_, body_target_rpy_);
    walking_control_->getLinearInvertedPendulumModel(lipm_x_, lipm_y_);
    walking_control_->getWalkingState(walking_leg_, walking_phase_);
    ROS_INFO("Target position: Body(%f, %f, %f), LLeg(%f, %f, %f), RLeg(%f, %f, %f)", body_target_pos_[0],
             body_target_pos_[1], body_target_pos_[2], l_leg_target_pos_[0], l_leg_target_pos_[1], l_leg_target_pos_[2],
             r_leg_target_pos_[0], r_leg_target_pos_[1], r_leg_target_pos_[2]);

    if (mov_step_ == mov_size_ - 1)
    {
      ROS_INFO("[END] Walking Control (%d/%d)", walking_step_ + 1, walking_size_);

      mov_step_ = 0;
      walking_control_->next();

      if (walking_step_ == walking_size_ - 1)
      {
        is_robot_moving_ = false;
        is_footstep_2d_active_ = false;
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
  double dsp_ratio = online_walking_param_.dsp_ratio;

  feed_forward_trajectory_ =
      new robotis_framework::MinimumJerkViaPoint(init_time, fin_time, via_time, dsp_ratio, zero_vector, zero_vector,
                                                 zero_vector, zero_vector, zero_vector, zero_vector, via_pos,
                                                 zero_vector, zero_vector);
}

void OnlineWalkingModule::setTargetForceTorque()
{
  if (walking_phase_ == DSP)
  {
    balance_r_foot_force_x_ = -0.5 * robot_mass_ * lipm_x_[2];
    balance_r_foot_force_y_ = -0.5 * robot_mass_ * lipm_y_[2];
    balance_r_foot_force_z_ = -0.5 * robot_mass_ * 9.81;

    balance_l_foot_force_x_ = -0.5 * robot_mass_ * lipm_x_[2];
    balance_l_foot_force_y_ = -0.5 * robot_mass_ * lipm_y_[2];
    balance_l_foot_force_z_ = -0.5 * robot_mass_ * 9.81;
  }
  else if (walking_phase_ == SSP)
  {
    if (walking_leg_ == LEFT_LEG)
    {
      balance_r_foot_force_x_ = -1.0 * robot_mass_ * lipm_x_[2];
      balance_r_foot_force_y_ = -1.0 * robot_mass_ * lipm_y_[2];
      balance_r_foot_force_z_ = -1.0 * robot_mass_ * 9.81;

      balance_l_foot_force_x_ = 0.0;
      balance_l_foot_force_y_ = 0.0;
      balance_l_foot_force_z_ = 0.0;
    }
    else if (walking_leg_ == RIGHT_LEG)
    {
      balance_r_foot_force_x_ = 0.0;
      balance_r_foot_force_y_ = 0.0;
      balance_r_foot_force_z_ = 0.0;

      balance_l_foot_force_x_ = -1.0 * robot_mass_ * lipm_x_[2];
      balance_l_foot_force_y_ = -1.0 * robot_mass_ * lipm_y_[2];
      balance_l_foot_force_z_ = -1.0 * robot_mass_ * 9.81;
    }
  }
}

void OnlineWalkingModule::setBalanceControlGain()
{
  //// set gain
  // gyro
  balance_control_.foot_roll_gyro_ctrl_.p_gain_ = foot_roll_gyro_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.foot_roll_gyro_ctrl_.d_gain_ = foot_roll_gyro_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.p_gain_ = foot_pitch_gyro_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.d_gain_ = foot_pitch_gyro_d_gain_ * curr_balance_gain_ratio_[0];

  // orientation
  balance_control_.foot_roll_angle_ctrl_.p_gain_ = foot_roll_angle_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.foot_roll_angle_ctrl_.d_gain_ = foot_roll_angle_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.p_gain_ = foot_pitch_angle_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.d_gain_ = foot_pitch_angle_d_gain_ * curr_balance_gain_ratio_[0];

  // force torque
  balance_control_.right_foot_force_x_ctrl_.p_gain_ = foot_x_force_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.p_gain_ = foot_y_force_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.p_gain_ = foot_z_force_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_force_x_ctrl_.d_gain_ = foot_x_force_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.d_gain_ = foot_y_force_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.d_gain_ = foot_z_force_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * curr_balance_gain_ratio_[0];

  balance_control_.left_foot_force_x_ctrl_.p_gain_ = foot_x_force_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.p_gain_ = foot_y_force_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.p_gain_ = foot_z_force_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_force_x_ctrl_.d_gain_ = foot_x_force_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.d_gain_ = foot_y_force_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.d_gain_ = foot_z_force_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * curr_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * curr_balance_gain_ratio_[0];

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
  balance_control_.setCOBManualAdjustment(curr_body_offset_[0], curr_body_offset_[1], curr_body_offset_[2]);
  setBalanceControlGain();
  setTargetForceTorque();

  // Body Desired Pose
  // ROS_INFO("body_target_pos: %f, %f, %f", body_target_pos_[0], body_target_pos_[1], body_target_pos_[2]);
  // ROS_INFO("body_target_rpy: %f, %f, %f", body_target_rpy_[0], body_target_rpy_[1], body_target_rpy_[2]);
  Eigen::MatrixXd body_target_pos = Eigen::MatrixXd::Zero(3, 1);
  body_target_pos.coeffRef(0, 0) = body_target_pos_[0];
  body_target_pos.coeffRef(1, 0) = body_target_pos_[1];
  body_target_pos.coeffRef(2, 0) = body_target_pos_[2];

  Eigen::MatrixXd body_target_rot =
      robotis_framework::convertRPYToRotation(body_target_rpy_[0], body_target_rpy_[1], body_target_rpy_[2]);

  // Right Leg Desired Pose
  // ROS_INFO("r_leg_target_pos: %f, %f, %f", r_leg_target_pos_[0], r_leg_target_pos_[1], r_leg_target_pos_[2]);
  // ROS_INFO("r_leg_target_rpy: %f, %f, %f", r_leg_target_rpy_[0], r_leg_target_rpy_[1], r_leg_target_rpy_[2]);
  Eigen::MatrixXd r_leg_target_pos = Eigen::MatrixXd::Zero(3, 1);
  r_leg_target_pos.coeffRef(0, 0) = r_leg_target_pos_[0];
  r_leg_target_pos.coeffRef(1, 0) = r_leg_target_pos_[1];
  r_leg_target_pos.coeffRef(2, 0) = r_leg_target_pos_[2];
  Eigen::MatrixXd r_leg_target_rot =
      robotis_framework::convertRPYToRotation(r_leg_target_rpy_[0], r_leg_target_rpy_[1], r_leg_target_rpy_[2]);

  // Left Leg Desired Pose
  // ROS_INFO("l_leg_target_pos: %f, %f, %f", l_leg_target_pos_[0], l_leg_target_pos_[1], l_leg_target_pos_[2]);
  // ROS_INFO("l_leg_target_rpy: %f, %f, %f", l_leg_target_rpy_[0], l_leg_target_rpy_[1], l_leg_target_rpy_[2]);
  Eigen::MatrixXd l_leg_target_pos = Eigen::MatrixXd::Zero(3, 1);
  l_leg_target_pos.coeffRef(0, 0) = l_leg_target_pos_[0];
  l_leg_target_pos.coeffRef(1, 0) = l_leg_target_pos_[1];
  l_leg_target_pos.coeffRef(2, 0) = l_leg_target_pos_[2];
  Eigen::MatrixXd l_leg_target_rot =
      robotis_framework::convertRPYToRotation(l_leg_target_rpy_[0], l_leg_target_rpy_[1], l_leg_target_rpy_[2]);

  // Set Desired Value for Balance Control
  Eigen::MatrixXd body_pose = Eigen::MatrixXd::Identity(4, 4);
  body_pose.block<3, 3>(0, 0) = body_target_rot;
  body_pose.block<3, 1>(0, 3) = body_target_pos;

  Eigen::MatrixXd l_foot_pose = Eigen::MatrixXd::Identity(4, 4);
  l_foot_pose.block<3, 3>(0, 0) = l_leg_target_rot;
  l_foot_pose.block<3, 1>(0, 3) = l_leg_target_pos;

  Eigen::MatrixXd r_foot_pose = Eigen::MatrixXd::Identity(4, 4);
  r_foot_pose.block<3, 3>(0, 0) = r_leg_target_rot;
  r_foot_pose.block<3, 1>(0, 3) = r_leg_target_pos;

  // Transformation Matrix from Robot to Foot
  Eigen::MatrixXd robot_to_body = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd robot_to_l_foot = body_pose.inverse() * l_foot_pose;
  Eigen::MatrixXd robot_to_r_foot = body_pose.inverse() * r_foot_pose;

  // Set IMU
  imu_data_mutex_lock_.lock();
  balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

  Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w, imu_data_msg_.orientation.x,
                                    imu_data_msg_.orientation.y, imu_data_msg_.orientation.z);
  Eigen::MatrixXd imu_rpy = robotis_framework::convertRotationToRPY(robotis_framework::getRotationX(M_PI) *
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
  balance_control_.setDesiredCOBOrientation(body_target_rpy_[0], body_target_rpy_[1]);
  balance_control_.setDesiredFootForceTorque(balance_r_foot_force_x_, balance_r_foot_force_y_, balance_r_foot_force_z_,
                                             balance_r_foot_torque_x_, balance_r_foot_torque_y_,
                                             balance_r_foot_torque_z_, balance_l_foot_force_x_, balance_l_foot_force_y_,
                                             balance_l_foot_force_z_, balance_l_foot_torque_x_,
                                             balance_l_foot_torque_y_, balance_l_foot_torque_z_);
  balance_control_.setDesiredPose(robot_to_body, robot_to_r_foot, robot_to_l_foot);

  int error;
  Eigen::MatrixXd robot_to_body_balanced, robot_to_r_foot_balanced, robot_to_l_foot_balanced;
  balance_control_.process(&error, &robot_to_body_balanced, &robot_to_r_foot_balanced, &robot_to_l_foot_balanced);

  // Inverse Kinematics
  bool ik_success = true;
  // Get desired body to desired foot transformation
  Eigen::MatrixXd body_to_r_leg_target_pose = Eigen::MatrixXd::Identity(4, 4);
  body_to_r_leg_target_pose.block(0, 0, 3, 3) = body_target_rot.inverse() * r_leg_target_rot;
  body_to_r_leg_target_pose.coeffRef(0, 3) = r_leg_target_pos.coeff(0, 0) - body_target_pos.coeff(0, 0);
  body_to_r_leg_target_pose.coeffRef(1, 3) = r_leg_target_pos.coeff(1, 0) - body_target_pos.coeff(1, 0);
  body_to_r_leg_target_pose.coeffRef(2, 3) = r_leg_target_pos.coeff(2, 0) - body_target_pos.coeff(2, 0);
  Eigen::MatrixXd body_to_r_leg_target_rpy =
      robotis_framework::convertRotationToRPY(body_to_r_leg_target_pose.block(0, 0, 3, 3));
  // ROS_INFO("body_to_r_leg_target_pose: pos(%f, %f, %f), rpy(%f, %f, %f)", body_to_r_leg_target_pose.coeff(0, 3),
  //          body_to_r_leg_target_pose.coeff(1, 3), body_to_r_leg_target_pose.coeff(2, 3),
  //          body_to_r_leg_target_rpy.coeff(0, 0), body_to_r_leg_target_rpy.coeff(1, 0),
  //          body_to_r_leg_target_rpy.coeff(2, 0));

  Eigen::MatrixXd body_to_l_leg_target_pose = Eigen::MatrixXd::Identity(4, 4);
  body_to_l_leg_target_pose.block(0, 0, 3, 3) = body_target_rot.inverse() * l_leg_target_rot;
  body_to_l_leg_target_pose.coeffRef(0, 3) = l_leg_target_pos.coeff(0, 0) - body_target_pos.coeff(0, 0);
  body_to_l_leg_target_pose.coeffRef(1, 3) = l_leg_target_pos.coeff(1, 0) - body_target_pos.coeff(1, 0);
  body_to_l_leg_target_pose.coeffRef(2, 3) = l_leg_target_pos.coeff(2, 0) - body_target_pos.coeff(2, 0);
  Eigen::MatrixXd body_to_l_leg_target_rpy =
      robotis_framework::convertRotationToRPY(body_to_l_leg_target_pose.block(0, 0, 3, 3));
  // ROS_INFO("body_to_l_leg_target_pose: pos(%f, %f, %f), rpy(%f, %f, %f)", body_to_l_leg_target_pose.coeff(0, 3),
  //          body_to_l_leg_target_pose.coeff(1, 3), body_to_l_leg_target_pose.coeff(2, 3),
  //          body_to_l_leg_target_rpy.coeff(0, 0), body_to_l_leg_target_rpy.coeff(1, 0),
  //          body_to_l_leg_target_rpy.coeff(2, 0));

  std::vector<double_t> r_leg_target_pose, l_leg_target_pose;
  r_leg_target_pose.resize(6);
  l_leg_target_pose.resize(6);

  r_leg_target_pose[0] = body_to_r_leg_target_pose.coeff(0, 3);
  r_leg_target_pose[1] = body_to_r_leg_target_pose.coeff(1, 3);
  r_leg_target_pose[2] = body_to_r_leg_target_pose.coeff(2, 3);
  r_leg_target_pose[3] = body_to_r_leg_target_rpy.coeff(0, 0);
  r_leg_target_pose[4] = body_to_r_leg_target_rpy.coeff(1, 0);
  r_leg_target_pose[5] = body_to_r_leg_target_rpy.coeff(2, 0);

  l_leg_target_pose[0] = body_to_l_leg_target_pose.coeff(0, 3);
  l_leg_target_pose[1] = body_to_l_leg_target_pose.coeff(1, 3);
  l_leg_target_pose[2] = body_to_l_leg_target_pose.coeff(2, 3);
  l_leg_target_pose[3] = body_to_l_leg_target_rpy.coeff(0, 0);
  l_leg_target_pose[4] = body_to_l_leg_target_rpy.coeff(1, 0);
  l_leg_target_pose[5] = body_to_l_leg_target_rpy.coeff(2, 0);

  std::vector<double_t> r_leg_joint_target_pos, l_leg_joint_target_pos;
  r_leg_joint_target_pos.resize(6);
  l_leg_joint_target_pos.resize(6);
  ik_success = (kuroko_kinematics_->solveInverseKinematicsForRightLeg(r_leg_joint_target_pos, r_leg_target_pose) &&
                kuroko_kinematics_->solveInverseKinematicsForLeftLeg(l_leg_joint_target_pos, l_leg_target_pose));

  if (ik_success)
  {
    motor_target_pos_[getJointIndex("hip_r_roll")] = r_leg_joint_target_pos[0];
    motor_target_pos_[getJointIndex("hip_r_pitch")] = r_leg_joint_target_pos[1];
    motor_target_pos_[getJointIndex("thigh_r_active")] = r_leg_joint_target_pos[2];
    motor_target_pos_[getJointIndex("shin_r_active")] = r_leg_joint_target_pos[3];
    motor_target_pos_[getJointIndex("ankle_r_roll")] = r_leg_joint_target_pos[4];
    motor_target_pos_[getJointIndex("ankle_r_yaw")] = r_leg_joint_target_pos[5];

    motor_target_pos_[getJointIndex("hip_l_roll")] = l_leg_joint_target_pos[0];
    motor_target_pos_[getJointIndex("hip_l_pitch")] = l_leg_joint_target_pos[1];
    motor_target_pos_[getJointIndex("thigh_l_active")] = l_leg_joint_target_pos[2];
    motor_target_pos_[getJointIndex("shin_l_active")] = l_leg_joint_target_pos[3];
    motor_target_pos_[getJointIndex("ankle_l_roll")] = l_leg_joint_target_pos[4];
    motor_target_pos_[getJointIndex("ankle_l_yaw")] = l_leg_joint_target_pos[5];
  }

  return ik_success;
}

void OnlineWalkingModule::setFeedbackControl()
{
  for (int i = 0; i < joint_name_.size(); i++)
  {
    command_joint_pos_[i] = motor_target_pos_[i] + command_joint_feedforward_[i];

    joint_feedback_[i].desired_ = motor_target_pos_[i];
    command_joint_feedback_[i] = joint_feedback_[i].getFeedBack(motor_curr_pos_[i]);

    command_joint_pos_[i] += command_joint_feedback_[i];
  }
}

void OnlineWalkingModule::setFeedforwardControl()
{
  double cur_time = (double)mov_step_ * control_cycle_sec_;

  std::vector<double_t> feed_forward_value = feed_forward_trajectory_->getPosition(cur_time);

  if (walking_phase_ == DSP)
    feed_forward_value[0] = 0.0;

  std::vector<double_t> support_leg_gain;
  support_leg_gain.resize(joint_name_.size(), 0.0);

  if (walking_leg_ == LEFT_LEG)
  {
    support_leg_gain[getJointIndex("hip_r_roll")] = 1.0;
    support_leg_gain[getJointIndex("hip_r_pitch")] = 1.0;
    support_leg_gain[getJointIndex("thigh_r_active")] = 1.0;
    support_leg_gain[getJointIndex("shin_r_active")] = 1.0;
    support_leg_gain[getJointIndex("ankle_r_roll")] = 1.0;
    support_leg_gain[getJointIndex("ankle_r_yaw")] = 1.0;

    support_leg_gain[getJointIndex("hip_l_roll")] = 0.0;
    support_leg_gain[getJointIndex("hip_l_pitch")] = 0.0;
    support_leg_gain[getJointIndex("thigh_l_active")] = 0.0;
    support_leg_gain[getJointIndex("shin_l_active")] = 0.0;
    support_leg_gain[getJointIndex("ankle_l_roll")] = 0.0;
    support_leg_gain[getJointIndex("ankle_l_yaw")] = 0.0;
  }
  else if (walking_leg_ == RIGHT_LEG)
  {
    support_leg_gain[getJointIndex("hip_r_roll")] = 0.0;
    support_leg_gain[getJointIndex("hip_r_pitch")] = 0.0;
    support_leg_gain[getJointIndex("thigh_r_active")] = 0.0;
    support_leg_gain[getJointIndex("shin_r_active")] = 0.0;
    support_leg_gain[getJointIndex("ankle_r_roll")] = 0.0;
    support_leg_gain[getJointIndex("ankle_r_yaw")] = 0.0;

    support_leg_gain[getJointIndex("hip_l_roll")] = 1.0;
    support_leg_gain[getJointIndex("hip_l_pitch")] = 1.0;
    support_leg_gain[getJointIndex("thigh_l_active")] = 1.0;
    support_leg_gain[getJointIndex("shin_l_active")] = 1.0;
    support_leg_gain[getJointIndex("ankle_l_roll")] = 1.0;
    support_leg_gain[getJointIndex("ankle_l_yaw")] = 1.0;
  }

  for (int i = 0; i < joint_name_.size(); i++)
    command_joint_feedforward_[i] = joint_feedforward_gain_[i] * feed_forward_value[0] * support_leg_gain[i];
}

void OnlineWalkingModule::gyroFeedback(const double& roll_gyro_err, const double& pitch_gyro_err,
                                       std::vector<double>& balance_angle)
{
  // adjust balance offset
  balance_angle.resize(joint_name_.size(), 0.0);

  balance_angle[getJointIndex("hip_r_roll")] = -1.0 * roll_gyro_err * balance_hip_roll_gain_;
  balance_angle[getJointIndex("thigh_r_active")] = 1.0 * pitch_gyro_err * balance_knee_gain_;
  balance_angle[getJointIndex("shin_r_active")] = -1.0 * pitch_gyro_err * balance_ankle_pitch_gain_;
  balance_angle[getJointIndex("ankle_r_roll")] = -1.0 * roll_gyro_err * balance_ankle_roll_gain_;
  balance_angle[getJointIndex("hip_l_roll")] = -1.0 * roll_gyro_err * balance_hip_roll_gain_;
  balance_angle[getJointIndex("thigh_l_active")] = -1.0 * pitch_gyro_err * balance_knee_gain_;
  balance_angle[getJointIndex("shin_l_active")] = 1.0 * pitch_gyro_err * balance_ankle_pitch_gain_;
  balance_angle[getJointIndex("ankle_l_roll")] = -1.0 * roll_gyro_err * balance_ankle_roll_gain_;
}

void OnlineWalkingModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                                  std::map<std::string, double> sensors)
{
  if (!enable_)
    return;

  ros::Time current_time = ros::Time::now();

  std::vector<double> balance_angle;
  balance_angle.resize(joint_name_.size(), 0.0);

  double rl_gyro_err = 0.0 - sensors["gyro_x"];
  double fb_gyro_err = 0.0 - sensors["gyro_y"];

  gyroFeedback(rl_gyro_err, fb_gyro_err, balance_angle);

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

    if (!is_goal_initialized_)
      motor_target_pos_[getJointIndex(joint_name)] = goal_joint_pos;

    motor_curr_pos_[getJointIndex(joint_name)] = curr_joint_pos;
  }

  is_goal_initialized_ = true;

  /* Trajectory Calculation */

  if (control_type_ == JOINT_CONTROL)
  {
    initJointControl();
    runJointControl();
  }
  else if (control_type_ == WHOLEBODY_CONTROL)
  {
    initWholebodyControl();
    runWholebodyControl();
  }
  else if (control_type_ == APPLY_BODY_OFFSET)
  {
    initBodyOffset();
    applyBodyOffset();
  }
  else if (control_type_ == WALKING_CONTROL)
  {
    if (is_walking_control_initialized_)
    {
      runWalkingControl();
      setFeedforwardControl();
    }
  }

  // Joint position feedback and feedforward
  setFeedbackControl();

  // IMU feedback
  if (!setBalanceControl())
  {
    is_robot_moving_ = false;
    is_balance_active_ = false;
    is_footstep_2d_active_ = false;
    balance_type_ = OFF;
    control_type_ = NONE;
    resetBodyPose();
    ROS_INFO("[OnlineWalkingModule::process] Set Balance Control Failed");
  }
  if (balance_type_ == ON)
  {
    initBalanceGain();
    applyBalanceGain();
    for (int i = 0; i < joint_name_.size(); i++)
      command_joint_pos_[i] += balance_angle[i];
  }

  // Publish topcs
  sensor_msgs::JointState goal_joint_msg;
  geometry_msgs::PoseStamped pelvis_pose_msg;

  goal_joint_msg.header.stamp = current_time;
  pelvis_pose_msg.header.stamp = current_time;
  pelvis_pose_msg.header.frame_id = world_frame_id_;

  pelvis_pose_msg.pose.position.x = body_target_pos_[0];
  pelvis_pose_msg.pose.position.y = body_target_pos_[1];
  pelvis_pose_msg.pose.position.z = body_target_pos_[2];

  Eigen::Quaterniond body_quaternionuaternion =
      robotis_framework::convertRPYToQuaternion(body_target_rpy_[0], body_target_rpy_[1], body_target_rpy_[2]);
  pelvis_pose_msg.pose.orientation.x = body_quaternionuaternion.x();
  pelvis_pose_msg.pose.orientation.y = body_quaternionuaternion.y();
  pelvis_pose_msg.pose.orientation.z = body_quaternionuaternion.z();
  pelvis_pose_msg.pose.orientation.w = body_quaternionuaternion.w();

  /*----- set joint data -----*/
  for (auto& state_iter : result_)
  {
    std::string joint_name = state_iter.first;
    if (getJointIndex(joint_name) != -1)
    {
      result_[joint_name]->goal_position_ = command_joint_pos_[getJointIndex(joint_name)];
      goal_joint_msg.name.push_back(joint_name);
      goal_joint_msg.position.push_back(motor_target_pos_[getJointIndex(joint_name)]);
    }
    else
      ROS_WARN("[WARN] Joint name %s not found in mapping!", joint_name.c_str());
  }

  pelvis_pose_pub_.publish(pelvis_pose_msg);
  goal_joint_state_pub_.publish(goal_joint_msg);

  // Check process time
  ros::Duration time_duration = ros::Time::now() - current_time;
  if (time_duration.toSec() > control_cycle_sec_)
    ROS_INFO("[OnlineWalkingModule::process] Process Time: %f > Control Cycle Time: %f", time_duration.toSec(),
             control_cycle_sec_);
}

void OnlineWalkingModule::stop()
{
  for (int i = 0; i < joint_name_.size(); i++)
  {
    motor_target_pos_[i] = 0.0;
    motor_target_vel_[i] = 0.0;
    motor_target_acc_[i] = 0.0;
  }

  is_goal_initialized_ = false;
  is_robot_moving_ = false;
  is_balance_active_ = false;

  is_joint_control_initialized_ = false;
  is_wholebody_control_initialized_ = false;
  is_walking_control_initialized_ = false;
  is_balance_control_initialized_ = false;

  control_type_ = NONE;
  return;
}

bool OnlineWalkingModule::isRunning()
{
  return is_robot_moving_;
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
  for (int i = 0; i < joint_name_.size(); i++)
  {
    res.pose.pose.name.push_back(joint_name_[i]);
    res.pose.pose.position.push_back(motor_target_pos_[i]);
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
    msg.position.x = body_target_pos_[0];
    msg.position.y = body_target_pos_[1];
    msg.position.z = body_target_pos_[2];

    Eigen::Quaterniond body_quaternionuaternion =
        robotis_framework::convertRPYToQuaternion(body_target_rpy_[0], body_target_rpy_[1], body_target_rpy_[2]);
    msg.orientation.x = body_quaternionuaternion.x();
    msg.orientation.y = body_quaternionuaternion.y();
    msg.orientation.z = body_quaternionuaternion.z();
    msg.orientation.w = body_quaternionuaternion.w();
  }
  else if (group_name == "left_leg")
  {
    msg.position.x = l_leg_target_pos_[0];
    msg.position.y = l_leg_target_pos_[1];
    msg.position.z = l_leg_target_pos_[2];

    Eigen::Quaterniond l_leg_quaternion =
        robotis_framework::convertRPYToQuaternion(l_leg_target_rpy_[0], l_leg_target_rpy_[1], l_leg_target_rpy_[2]);
    msg.orientation.x = l_leg_quaternion.x();
    msg.orientation.y = l_leg_quaternion.y();
    msg.orientation.z = l_leg_quaternion.z();
    msg.orientation.w = l_leg_quaternion.w();
  }
  else if (group_name == "right_leg")
  {
    msg.position.x = r_leg_target_pos_[0];
    msg.position.y = r_leg_target_pos_[1];
    msg.position.z = r_leg_target_pos_[2];

    Eigen::Quaterniond r_leg_quaternion =
        robotis_framework::convertRPYToQuaternion(r_leg_target_rpy_[0], r_leg_target_rpy_[1], r_leg_target_rpy_[2]);
    msg.orientation.x = r_leg_quaternion.x();
    msg.orientation.y = r_leg_quaternion.y();
    msg.orientation.z = r_leg_quaternion.z();
    msg.orientation.w = r_leg_quaternion.w();
  }

  res.pose.pose = msg;

  return true;
}

bool OnlineWalkingModule::definePreviewMatrix()
{
  // Maybe parameters for zmp walking odometry estimation
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
