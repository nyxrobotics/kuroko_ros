#include <iostream>
#include <utility>

#include "kuroko_walking_module/kuroko_walking_module.h"

namespace motion_control
{
WalkingModule::WalkingModule() : control_cycle_msec_(8), debug_(false)
{
  enable_ = false;
  module_name_ = "walking_module";
  control_mode_ = robotis_framework::PositionControl;

  init_pose_count_ = 0;
  walking_state_ = WALK_READY;
  previous_x_move_amplitude_ = 0.0;

  kuroko_kinematics_ = new KurokoKinematics(WHOLE_BODY);

  // Robot is in initial posture with legs extended directly below
  // Height of the hips when the legs are fully extended
  leg_default_length_ = kuroko_kinematics_->leg_max_height_;
  // Distance between left and right feet
  leg_default_separaion_ = kuroko_kinematics_->leg_side_offset_;

  // result
  result_["hip_r_roll"] = new robotis_framework::DynamixelState();
  result_["hip_r_pitch"] = new robotis_framework::DynamixelState();
  result_["thigh_r_active"] = new robotis_framework::DynamixelState();
  result_["shin_r_active"] = new robotis_framework::DynamixelState();
  result_["ankle_r_roll"] = new robotis_framework::DynamixelState();
  result_["ankle_r_yaw"] = new robotis_framework::DynamixelState();

  result_["hip_l_roll"] = new robotis_framework::DynamixelState();
  result_["hip_l_pitch"] = new robotis_framework::DynamixelState();
  result_["thigh_l_active"] = new robotis_framework::DynamixelState();
  result_["shin_l_active"] = new robotis_framework::DynamixelState();
  result_["ankle_l_roll"] = new robotis_framework::DynamixelState();
  result_["ankle_l_yaw"] = new robotis_framework::DynamixelState();

  // joint table
  joint_table_["hip_r_roll"] = 0;
  joint_table_["hip_r_pitch"] = 1;
  joint_table_["thigh_r_active"] = 2;
  joint_table_["shin_r_active"] = 3;
  joint_table_["ankle_r_roll"] = 4;
  joint_table_["ankle_r_yaw"] = 5;

  joint_table_["hip_l_roll"] = 6;
  joint_table_["hip_l_pitch"] = 7;
  joint_table_["thigh_l_active"] = 8;
  joint_table_["shin_l_active"] = 9;
  joint_table_["ankle_l_roll"] = 10;
  joint_table_["ankle_l_yaw"] = 11;

  target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  init_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  joint_axis_direction_ = Eigen::MatrixXi::Zero(1, result_.size());
  std::cout << "WalkingModule: Initialization, result size: " << result_.size() << std::endl;
  std::cout << "WalkingModule: Initialization, joint_table size: " << joint_table_.size() << std::endl;
}

WalkingModule::~WalkingModule()
{
  queue_thread_.join();
}

void WalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot* /*robot*/)
{
  queue_thread_ = boost::thread(boost::bind(&WalkingModule::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;

  // m, s, rad
  // init pose
  walking_param_.init_x_offset = -0.010;
  walking_param_.init_y_offset = 0.005;
  walking_param_.init_z_offset = 0.020;
  walking_param_.init_roll_offset = 0.0;
  walking_param_.init_pitch_offset = 0.0 * DEGREE2RADIAN;
  walking_param_.init_yaw_offset = 0.0 * DEGREE2RADIAN;
  walking_param_.hip_pitch_offset = 13.0 * DEGREE2RADIAN;
  // time
  walking_param_.period_time = 600 * 0.001;
  walking_param_.dsp_ratio = 0.1;
  walking_param_.step_fb_ratio = 0.28;
  // walking
  walking_param_.x_move_amplitude = 0.0;
  walking_param_.y_move_amplitude = 0.0;
  walking_param_.z_move_amplitude = 0.040;  // foot height
  walking_param_.angle_move_amplitude = 0.0;
  // balance
  walking_param_.balance_enable = false;
  walking_param_.balance_hip_roll_gain = 0.5;
  walking_param_.balance_knee_gain = 0.3;
  walking_param_.balance_ankle_roll_gain = 1.0;
  walking_param_.balance_ankle_pitch_gain = 0.9;
  walking_param_.y_swap_amplitude = 0.020;
  walking_param_.z_swap_amplitude = 0.005;
  walking_param_.pelvis_offset = 3.0 * DEGREE2RADIAN;
  walking_param_.arm_swing_gain = 1.5;

  // member variable
  body_swing_y_ = 0;
  body_swing_z_ = 0;

  x_swap_phase_shift_ = M_PI;
  x_swap_amplitude_shift_ = 0;
  x_move_phase_shift_ = M_PI / 2;
  x_move_amplitude_shift_ = 0;
  y_swap_phase_shift_ = 0;
  y_swap_amplitude_shift_ = 0;
  y_move_phase_shift_ = M_PI / 2;
  z_swap_phase_shift_ = M_PI * 3 / 2;
  z_move_phase_shift_ = M_PI / 2;
  a_move_phase_shift_ = M_PI / 2;

  ctrl_running_ = false;
  real_running_ = false;
  time_ = 0;
  // TODO: set joint directions from robot model
  joint_axis_direction_ << 1, -1, 1, 1, 1,
      -1,                    // hip_r_roll, hip_r_pitch, thigh_r_active, shin_r_active, ankle_r_roll, ankle_r_yaw
      -1, 1, -1, -1, 1, -1;  // hip_l_roll, hip_l_pitch, thigh_l_active, shin_l_active, ankle_l_roll, ankle_l_yaw
  init_position_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  init_position_ *= DEGREE2RADIAN;

  ros::NodeHandle ros_node;

  std::string default_param_path = ros::package::getPath("kuroko_walking_module") + "/config/param.yaml";
  ros_node.param<std::string>("walking_param_path", param_path_, default_param_path);

  loadWalkingParam(param_path_);

  updateTimeParam();
  updateMovementParam();
}

void WalkingModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("motion_control/status", 1);

  /* ROS Service Callback Functions */
  ros::ServiceServer get_walking_param_server =
      ros_node.advertiseService("/motion_control/walking/get_params", &WalkingModule::getWalkigParameterCallback, this);

  /* sensor topic subscribe */
  ros::Subscriber walking_command_sub =
      ros_node.subscribe("/motion_control/walking/command", 0, &WalkingModule::walkingCommandCallback, this);
  ros::Subscriber walking_param_sub =
      ros_node.subscribe("/motion_control/walking/set_params", 0, &WalkingModule::walkingParameterCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void WalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = std::move(msg);

  status_msg_pub_.publish(status_msg);
}

void WalkingModule::walkingCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (!enable_)
  {
    ROS_WARN("walking module is not ready.");
    return;
  }

  if (msg->data == "start")
    startWalking();
  else if (msg->data == "stop")
    stop();
  else if (msg->data == "balance on")
    walking_param_.balance_enable = true;
  else if (msg->data == "balance off")
    walking_param_.balance_enable = false;
  else if (msg->data == "save")
    saveWalkingParam(param_path_);
}

void WalkingModule::walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg)
{
  walking_param_ = *msg;
}

bool WalkingModule::getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request& /*req*/,
                                               op3_walking_module_msgs::GetWalkingParam::Response& res)
{
  res.parameters = walking_param_;

  return true;
}

double WalkingModule::wSin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

void WalkingModule::updateTimeParam()
{
  period_time_ = walking_param_.period_time;  // * 1000;   // s -> ms
  dsp_ratio_ = walking_param_.dsp_ratio;
  ssp_ratio_ = 1 - dsp_ratio_;

  x_swap_period_time_ = period_time_ / 2;
  x_move_period_time_ = period_time_ * ssp_ratio_;
  y_swap_period_time_ = period_time_;
  y_move_period_time_ = period_time_ * ssp_ratio_;
  z_swap_period_time_ = period_time_ / 2;
  z_move_period_time_ = period_time_ * ssp_ratio_ / 2;
  a_move_period_time_ = period_time_ * ssp_ratio_;

  ssp_time_ = period_time_ * ssp_ratio_;
  l_ssp_start_time_ = (1 - ssp_ratio_) * period_time_ / 4;
  l_ssp_end_time_ = (1 + ssp_ratio_) * period_time_ / 4;
  r_ssp_start_time_ = (3 - ssp_ratio_) * period_time_ / 4;
  r_ssp_end_time_ = (3 + ssp_ratio_) * period_time_ / 4;

  phase1_time_ = (l_ssp_start_time_ + l_ssp_end_time_) / 2;
  phase2_time_ = (l_ssp_end_time_ + r_ssp_start_time_) / 2;
  phase3_time_ = (r_ssp_start_time_ + r_ssp_end_time_) / 2;

  pelvis_offset_ = walking_param_.pelvis_offset;
  pelvis_swing_ = pelvis_offset_ * 0.35;
  arm_swing_gain_ = walking_param_.arm_swing_gain;
}

void WalkingModule::updateMovementParam()
{
  // Forward/Back
  x_move_amplitude_ = walking_param_.x_move_amplitude;
  x_swap_amplitude_ = walking_param_.x_move_amplitude * walking_param_.step_fb_ratio;

  if (previous_x_move_amplitude_ == 0)
  {
    x_move_amplitude_ *= 0.5;
    x_swap_amplitude_ *= 0.5;
  }

  // Right/Left
  y_move_amplitude_ = walking_param_.y_move_amplitude / 2;
  if (y_move_amplitude_ > 0)
    y_move_amplitude_shift_ = y_move_amplitude_;
  else
    y_move_amplitude_shift_ = -y_move_amplitude_;
  y_swap_amplitude_ = walking_param_.y_swap_amplitude + y_move_amplitude_shift_ * 0.04;

  z_move_amplitude_ = walking_param_.z_move_amplitude / 2;
  z_move_amplitude_shift_ = z_move_amplitude_ / 2;
  z_swap_amplitude_ = walking_param_.z_swap_amplitude;
  z_swap_amplitude_shift_ = z_swap_amplitude_;

  // Direction
  if (!static_cast<bool>(walking_param_.move_aim_on))
  {
    a_move_amplitude_ = walking_param_.angle_move_amplitude / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = a_move_amplitude_;
    else
      a_move_amplitude_shift_ = -a_move_amplitude_;
  }
  else
  {
    a_move_amplitude_ = -walking_param_.angle_move_amplitude / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = -a_move_amplitude_;
    else
      a_move_amplitude_shift_ = a_move_amplitude_;
  }
}

void WalkingModule::updatePoseParam()
{
  x_offset_ = walking_param_.init_x_offset;
  y_offset_ = walking_param_.init_y_offset;
  z_offset_ = walking_param_.init_z_offset;
  r_offset_ = walking_param_.init_roll_offset;
  p_offset_ = walking_param_.init_pitch_offset;
  a_offset_ = walking_param_.init_yaw_offset;
  hip_pitch_offset_ = walking_param_.hip_pitch_offset;
}

void WalkingModule::startWalking()
{
  ctrl_running_ = true;
  real_running_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start walking");
}

void WalkingModule::stop()
{
  ctrl_running_ = false;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Stop walking");
}

bool WalkingModule::isRunning()
{
  return real_running_ || (walking_state_ == WALK_INITIAL_POSE);
}

// default [angle : radian, length : m]
void WalkingModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                            std::map<std::string, double> sensors)
{
  if (!enable_)
    return;

  const double time_unit = control_cycle_msec_ * 0.001;  // ms -> s
  int joint_size = result_.size();
  std::vector<double> current_angle(joint_size, 0.0);
  std::vector<double> balance_angle(joint_size, 0.0);

  if (walking_state_ == WALK_INITIAL_POSE)
  {
    int total_count = calc_joint_trajectory_.rows();
    for (int id = 0; id < result_.size(); id++)
      target_position_.coeffRef(0, id) = calc_joint_trajectory_(init_pose_count_, id + 1);

    init_pose_count_ += 1;
    if (init_pose_count_ >= total_count)
    {
      walking_state_ = WALK_READY;
      if (debug_)
        std::cout << "End moving to Init : " << init_pose_count_ << std::endl;
    }
  }
  else if (walking_state_ == WALK_READY || walking_state_ == WALK_ENABLE)
  {
    // current_angle
    for (auto& state_iter : result_)
    {
      std::string joint_name = state_iter.first;
      int joint_index = joint_table_[joint_name];

      robotis_framework::Dynamixel* dxl = nullptr;
      std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
      if (dxl_it != dxls.end())
        dxl = dxl_it->second;
      else
        continue;

      goal_position_.coeffRef(0, joint_index) = dxl->dxl_state_->goal_position_;
    }

    processPhase(time_unit);

    bool get_current_angle = updateLegTargetAngles(current_angle);

    double rl_gyro_err = 0.0 - sensors["gyro_x"];
    double fb_gyro_err = 0.0 - sensors["gyro_y"];

    gyroFeedback(rl_gyro_err, fb_gyro_err, balance_angle);

    double err_total = 0.0, err_max = 0.0;
    // set goal position
    for (int idx = 0; idx < 12; idx++)
    {
      double goal_position = (!get_current_angle) ?
                                 goal_position_.coeff(0, idx) :
                                 init_position_.coeff(0, idx) + current_angle[idx] + balance_angle[idx];
      target_position_.coeffRef(0, idx) = goal_position;

      double err = fabs(target_position_.coeff(0, idx) - goal_position_.coeff(0, idx)) * RADIAN2DEGREE;
      if (err > err_max)
        err_max = err;
      err_total += err;
    }

    // Check Enable
    if (walking_state_ == WALK_ENABLE && err_total > 5.0)
    {
      if (debug_)
        std::cout << "Check Err : " << err_max << std::endl;
      int mov_time = err_max / 30;
      iniPoseTraGene(mov_time < 1 ? 1 : mov_time);
      target_position_ = goal_position_;
      walking_state_ = WALK_INITIAL_POSE;
      ROS_INFO_STREAM_COND(debug_, "x_offset: " << walking_param_.init_x_offset);
      ROS_INFO_STREAM_COND(debug_, "y_offset: " << walking_param_.init_y_offset);
      ROS_INFO_STREAM_COND(debug_, "z_offset: " << walking_param_.init_z_offset);
      ROS_INFO_STREAM_COND(debug_, "roll_offset: " << walking_param_.init_roll_offset * RADIAN2DEGREE);
      ROS_INFO_STREAM_COND(debug_, "pitch_offset: " << walking_param_.init_pitch_offset * RADIAN2DEGREE);
      ROS_INFO_STREAM_COND(debug_, "yaw_offset: " << walking_param_.init_yaw_offset * RADIAN2DEGREE);
      ROS_INFO_STREAM_COND(debug_, "hip_pitch_offset: " << walking_param_.hip_pitch_offset * RADIAN2DEGREE);
      ROS_INFO_STREAM_COND(debug_, "period_time: " << walking_param_.period_time * 1000);
      ROS_INFO_STREAM_COND(debug_, "dsp_ratio: " << walking_param_.dsp_ratio);
      ROS_INFO_STREAM_COND(debug_, "step_forward_back_ratio: " << walking_param_.step_fb_ratio);
      ROS_INFO_STREAM_COND(debug_, "foot_height: " << walking_param_.z_move_amplitude);
      ROS_INFO_STREAM_COND(debug_, "swing_right_left: " << walking_param_.y_swap_amplitude);
      ROS_INFO_STREAM_COND(debug_, "swing_top_down: " << walking_param_.z_swap_amplitude);
      ROS_INFO_STREAM_COND(debug_, "pelvis_offset: " << walking_param_.pelvis_offset * RADIAN2DEGREE);
      ROS_INFO_STREAM_COND(debug_, "arm_swing_gain: " << walking_param_.arm_swing_gain);
      ROS_INFO_STREAM_COND(debug_, "balance_hip_roll_gain: " << walking_param_.balance_hip_roll_gain);
      ROS_INFO_STREAM_COND(debug_, "balance_knee_gain: " << walking_param_.balance_knee_gain);
      ROS_INFO_STREAM_COND(debug_, "balance_ankle_roll_gain: " << walking_param_.balance_ankle_roll_gain);
      ROS_INFO_STREAM_COND(debug_, "balance_ankle_pitch_gain: " << walking_param_.balance_ankle_pitch_gain);
      ROS_INFO_STREAM_COND(debug_, "balance : " << (walking_param_.balance_enable ? "TRUE" : "FALSE"));
    }
    else
    {
      walking_state_ = WALK_READY;
    }
  }

  for (auto& state_it : result_)
  {
    std::string joint_name = state_it.first;
    int joint_index = joint_table_[joint_name];
    result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);
  }

  if (real_running_)
  {
    time_ += time_unit;
    if (time_ >= period_time_)
    {
      time_ = 0;
      previous_x_move_amplitude_ = walking_param_.x_move_amplitude * 0.5;
    }
  }
}

void WalkingModule::processPhase(const double& time_unit)
{
  // Update walk parameters
  if (time_ == 0)
  {
    updateTimeParam();
    phase_ = PHASE0;
    if (!ctrl_running_)
    {
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // set walking param to init
        walking_param_.x_move_amplitude = 0;
        walking_param_.y_move_amplitude = 0;
        walking_param_.angle_move_amplitude = 0;

        previous_x_move_amplitude_ = 0;
      }
    }
  }
  else if (time_ >= (phase1_time_ - time_unit / 2) && time_ < (phase1_time_ + time_unit / 2))  // the position of left
                                                                                               // foot is the highest.
  {
    updateMovementParam();
    phase_ = PHASE1;
  }
  else if (time_ >= (phase2_time_ - time_unit / 2) && time_ < (phase2_time_ + time_unit / 2))  // middle of double
                                                                                               // support state
  {
    updateTimeParam();

    time_ = phase2_time_;
    phase_ = PHASE2;
    if (!ctrl_running_)
    {
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // set walking param to init
        walking_param_.x_move_amplitude = previous_x_move_amplitude_;
        walking_param_.y_move_amplitude = 0;
        walking_param_.angle_move_amplitude = 0;
      }
    }
  }
  else if (time_ >= (phase3_time_ - time_unit / 2) && time_ < (phase3_time_ + time_unit / 2))  // the position of right
                                                                                               // foot is the highest.
  {
    updateMovementParam();
    phase_ = PHASE3;
  }
}

bool WalkingModule::updateLegTargetAngles(std::vector<double>& leg_joints)
{
  Pose3D swap, right_leg_move, left_leg_move;
  double pelvis_offset_r, pelvis_offset_l;
  std::vector<double> right_target_point(6, 0);
  std::vector<double> left_target_point(6, 0);

  std::vector<double> right_joints(6, 0);
  std::vector<double> left_joints(6, 0);

  updatePoseParam();

  // Compute endpoints
  swap.x_ = wSin(time_, x_swap_period_time_, x_swap_phase_shift_, x_swap_amplitude_, x_swap_amplitude_shift_);
  swap.y_ = wSin(time_, y_swap_period_time_, y_swap_phase_shift_, y_swap_amplitude_, y_swap_amplitude_shift_);
  swap.z_ = wSin(time_, z_swap_period_time_, z_swap_phase_shift_, z_swap_amplitude_, z_swap_amplitude_shift_);
  swap.roll_ = 0.0;
  swap.pitch_ = 0.0;
  swap.yaw_ = 0.0;

  if (time_ <= l_ssp_start_time_)
  {
    left_leg_move.x_ = wSin(l_ssp_start_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                            x_move_amplitude_shift_);
    left_leg_move.y_ = wSin(l_ssp_start_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                            y_move_amplitude_shift_);
    left_leg_move.z_ = wSin(l_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    left_leg_move.yaw_ = wSin(l_ssp_start_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x_ = wSin(l_ssp_start_time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                             -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y_ = wSin(l_ssp_start_time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                             -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z_ = wSin(r_ssp_start_time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_,
                             z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw_ = wSin(l_ssp_start_time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                               -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (time_ <= l_ssp_end_time_)
  {
    left_leg_move.x_ =
        wSin(time_, x_move_period_time_, x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
             x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y_ =
        wSin(time_, y_move_period_time_, y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
             y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z_ =
        wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_,
             z_move_amplitude_, z_move_amplitude_shift_);
    left_leg_move.yaw_ =
        wSin(time_, a_move_period_time_, a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x_ =
        wSin(time_, x_move_period_time_, x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
             -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y_ =
        wSin(time_, y_move_period_time_, y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
             -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z_ = wSin(r_ssp_start_time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_,
                             z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw_ =
        wSin(time_, a_move_period_time_, a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
             -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l =
        wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_,
             pelvis_swing_ / 2, pelvis_swing_ / 2);
    pelvis_offset_r =
        wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_,
             -pelvis_offset_ / 2, -pelvis_offset_ / 2);
  }
  else if (time_ <= r_ssp_start_time_)
  {
    left_leg_move.x_ = wSin(l_ssp_end_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                            x_move_amplitude_shift_);
    left_leg_move.y_ = wSin(l_ssp_end_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                            y_move_amplitude_shift_);
    left_leg_move.z_ = wSin(l_ssp_end_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    left_leg_move.yaw_ = wSin(l_ssp_end_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x_ = wSin(l_ssp_end_time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                             -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y_ = wSin(l_ssp_end_time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                             -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z_ = wSin(r_ssp_start_time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_,
                             z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw_ = wSin(l_ssp_end_time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                               -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (time_ <= r_ssp_end_time_)
  {
    left_leg_move.x_ = wSin(time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                            x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y_ = wSin(time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                            y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z_ = wSin(l_ssp_end_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    left_leg_move.yaw_ = wSin(time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                              a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x_ = wSin(time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                             -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y_ = wSin(time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                             -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z_ =
        wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_,
             z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw_ = wSin(time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                               -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l =
        wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_,
             pelvis_offset_ / 2, pelvis_offset_ / 2);
    pelvis_offset_r =
        wSin(time_, z_move_period_time_, z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_,
             -pelvis_swing_ / 2, -pelvis_swing_ / 2);
  }
  else
  {
    left_leg_move.x_ = wSin(r_ssp_end_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                            x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y_ = wSin(r_ssp_end_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                            y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z_ = wSin(l_ssp_end_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    left_leg_move.yaw_ = wSin(r_ssp_end_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                              a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x_ = wSin(r_ssp_end_time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                             -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y_ = wSin(r_ssp_end_time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                             -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z_ = wSin(r_ssp_end_time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_,
                             z_move_amplitude_, z_move_amplitude_shift_);
    right_leg_move.yaw_ = wSin(r_ssp_end_time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                               -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }

  left_leg_move.roll_ = 0;
  left_leg_move.pitch_ = 0;
  right_leg_move.roll_ = 0;
  right_leg_move.pitch_ = 0;

  // mm, rad
  // Right leg target point
  right_target_point[0] = swap.x_ + right_leg_move.x_ + x_offset_;
  right_target_point[1] = swap.y_ + right_leg_move.y_ - (y_offset_ + leg_default_separaion_) / 2.0;
  right_target_point[2] = swap.z_ + right_leg_move.z_ + z_offset_ - leg_default_length_;
  right_target_point[3] = swap.roll_ + right_leg_move.roll_ - r_offset_ / 2.0;
  right_target_point[4] = swap.pitch_ + right_leg_move.pitch_ + p_offset_;
  right_target_point[5] = swap.yaw_ + right_leg_move.yaw_ - a_offset_ / 2.0;
  // Left leg target point
  left_target_point[0] = swap.x_ + left_leg_move.x_ + x_offset_;
  left_target_point[1] = swap.y_ + left_leg_move.y_ + (y_offset_ + leg_default_separaion_) / 2.0;
  left_target_point[2] = swap.z_ + left_leg_move.z_ + z_offset_ - leg_default_length_;
  left_target_point[3] = swap.roll_ + left_leg_move.roll_ + r_offset_ / 2.0;
  left_target_point[4] = swap.pitch_ + left_leg_move.pitch_ + p_offset_;
  left_target_point[5] = swap.yaw_ + left_leg_move.yaw_ + a_offset_ / 2.0;

  // Compute body swing
  if (time_ <= l_ssp_end_time_)
  {
    body_swing_y_ = -left_target_point[1];
    body_swing_z_ = left_target_point[2];
  }
  else
  {
    body_swing_y_ = -right_target_point[1];
    body_swing_z_ = right_target_point[2];
  }
  body_swing_z_ -= leg_default_length_;

  // Right leg IK
  if (!kuroko_kinematics_->solveInverseKinematicsForRightLeg(right_joints, right_target_point))
  {
    printf("IK not Solved EPR : %f %f %f %f %f %f\n", right_target_point[0], right_target_point[1],
           right_target_point[2], right_target_point[3], right_target_point[4], right_target_point[5]);
    return false;
  }
  // Left leg IK
  if (!kuroko_kinematics_->solveInverseKinematicsForLeftLeg(left_joints, left_target_point))
  {
    printf("IK not Solved EPL : %f %f %f %f %f %f\n", left_target_point[0], left_target_point[1], left_target_point[2],
           left_target_point[3], left_target_point[4], left_target_point[5]);
    return false;
  }

  // Check IK result with FK result
  // printf("---Right IK--- : %.4f %.4f %.4f %.4f %.4f %.4f\n", right_joints[0], right_joints[1], right_joints[2],
  //        right_joints[3], right_joints[4], right_joints[5]);
  // printf("Right Target Point        : %.4f %.4f %.4f %.4f %.4f %.4f\n", right_target_point[0], right_target_point[1],
  //        right_target_point[2], right_target_point[3], right_target_point[4], right_target_point[5]);
  // kuroko_kinematics_->solveForwardKinematicsForRightLeg(right_joints, right_target_point);
  // printf("Right  Forward Kinematics : %.4f %.4f %.4f %.4f %.4f %.4f\n", right_target_point[0], right_target_point[1],
  //        right_target_point[2], right_target_point[3], right_target_point[4], right_target_point[5]);
  // printf("---Left IK--- : %.4f %.4f %.4f %.4f %.4f %.4f\n", left_joints[0], left_joints[1], left_joints[2],
  //        left_joints[3], left_joints[4], left_joints[5]);
  // printf("Left Target Point       : %.4f %.4f %.4f %.4f %.4f %.4f\n", left_target_point[0], left_target_point[1],
  //        left_target_point[2], left_target_point[3], left_target_point[4], left_target_point[5]);
  // kuroko_kinematics_->solveForwardKinematicsForLeftLeg(left_joints, left_target_point);
  // printf("Left Forward Kinematics : %.4f %.4f %.4f %.4f %.4f %.4f\n", left_target_point[0], left_target_point[1],
  //        left_target_point[2], left_target_point[3], left_target_point[4], left_target_point[5]);

  // Add offset angles [rad]
  // Hip Roll Offset
  right_joints[0] += kuroko_kinematics_->getJointDirection("hip_r_roll") * pelvis_offset_r;
  left_joints[0] += kuroko_kinematics_->getJointDirection("hip_l_roll") * pelvis_offset_l;
  // Hip Pitch Offset
  right_joints[1] -= kuroko_kinematics_->getJointDirection("hip_r_pitch") * hip_pitch_offset_;
  left_joints[1] -= kuroko_kinematics_->getJointDirection("hip_l_pitch") * hip_pitch_offset_;

  leg_joints.resize(12);
  for (int i = 0; i < 6; i++)
  {
    leg_joints[i] = right_joints[i];
    leg_joints[i + 6] = left_joints[i];
  }

  return true;
}

void WalkingModule::gyroFeedback(const double& roll_gyro_err, const double& pitch_gyro_err,
                                 std::vector<double>& balance_angle)
{
  // adjust balance offset
  if (!static_cast<bool>(walking_param_.balance_enable))
    return;

  // std::cout << "roll_gyro_err : " << roll_gyro_err << ", pitch_gyro_err : " << pitch_gyro_err << std::endl;

  // Roll joints
  balance_angle[joint_table_["hip_r_roll"]] =
      kuroko_kinematics_->getJointDirection("hip_r_roll") * roll_gyro_err * walking_param_.balance_hip_roll_gain;
  balance_angle[joint_table_["hip_l_roll"]] =
      kuroko_kinematics_->getJointDirection("hip_l_roll") * roll_gyro_err * walking_param_.balance_hip_roll_gain;
  balance_angle[joint_table_["ankle_r_roll"]] =
      -kuroko_kinematics_->getJointDirection("ankle_r_roll") * roll_gyro_err * walking_param_.balance_ankle_roll_gain;
  balance_angle[joint_table_["ankle_l_roll"]] =
      -kuroko_kinematics_->getJointDirection("ankle_l_roll") * roll_gyro_err * walking_param_.balance_ankle_roll_gain;

  // Pitch joints
  balance_angle[joint_table_["hip_r_pitch"]] =
      kuroko_kinematics_->getJointDirection("hip_r_pitch") * pitch_gyro_err * walking_param_.balance_knee_gain;
  balance_angle[joint_table_["hip_l_pitch"]] =
      kuroko_kinematics_->getJointDirection("hip_l_pitch") * pitch_gyro_err * walking_param_.balance_knee_gain;
  balance_angle[joint_table_["thigh_r_active"]] = -kuroko_kinematics_->getJointDirection("thigh_r_active") *
                                                  pitch_gyro_err * walking_param_.balance_ankle_pitch_gain;
  balance_angle[joint_table_["thigh_l_active"]] = -kuroko_kinematics_->getJointDirection("thigh_l_active") *
                                                  pitch_gyro_err * walking_param_.balance_ankle_pitch_gain;
  balance_angle[joint_table_["shin_r_active"]] = -kuroko_kinematics_->getJointDirection("shin_r_active") *
                                                 pitch_gyro_err * walking_param_.balance_ankle_pitch_gain;
  balance_angle[joint_table_["shin_l_active"]] = -kuroko_kinematics_->getJointDirection("shin_l_active") *
                                                 pitch_gyro_err * walking_param_.balance_ankle_pitch_gain;
}

void WalkingModule::loadWalkingParam(const std::string& path)
{
  YAML::Node doc;
  ROS_INFO("WAlkingModule::loadWalkingParam - Loading: %s", path.c_str());
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

  // Initial pose offset
  walking_param_.init_x_offset = doc["x_offset"].as<double>();
  walking_param_.init_y_offset = doc["y_offset"].as<double>();
  walking_param_.init_z_offset = doc["z_offset"].as<double>();
  walking_param_.init_roll_offset = doc["roll_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_pitch_offset = doc["pitch_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_yaw_offset = doc["yaw_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.hip_pitch_offset = doc["hip_pitch_offset"].as<double>() * DEGREE2RADIAN;
  // Cycle Time
  walking_param_.period_time = doc["period_time"].as<double>() * 0.001;  // ms -> s
  walking_param_.dsp_ratio = doc["dsp_ratio"].as<double>();
  walking_param_.step_fb_ratio = doc["step_forward_back_ratio"].as<double>();
  // Foot Height
  walking_param_.z_move_amplitude = doc["foot_height"].as<double>();
  // Target step length
  // walking_param_.x_move_amplitude = doc["x_move_amplitude"].as<double>();
  // walking_param_.y_move_amplitude = doc["y_move_amplitude"].as<double>();
  // walking_param_.angle_move_amplitude = doc["angle_move_amplitude"].as<double>();
  // walking_param_.move_aim_on = doc["move_aim_on"].as<double>();

  // balance
  // walking_param_.balance_enable = doc["balance_enable"].as<uint8_t>();
  walking_param_.balance_hip_roll_gain = doc["balance_hip_roll_gain"].as<double>();
  walking_param_.balance_knee_gain = doc["balance_knee_gain"].as<double>();
  walking_param_.balance_ankle_roll_gain = doc["balance_ankle_roll_gain"].as<double>();
  walking_param_.balance_ankle_pitch_gain = doc["balance_ankle_pitch_gain"].as<double>();
  walking_param_.y_swap_amplitude = doc["swing_right_left"].as<double>();
  walking_param_.z_swap_amplitude = doc["swing_top_down"].as<double>();
  walking_param_.pelvis_offset = doc["pelvis_offset"].as<double>() * DEGREE2RADIAN;
}

void WalkingModule::saveWalkingParam(std::string& path)
{
  YAML::Emitter out_emitter;

  out_emitter << YAML::BeginMap;
  out_emitter << YAML::Key << "x_offset" << YAML::Value << walking_param_.init_x_offset;
  out_emitter << YAML::Key << "y_offset" << YAML::Value << walking_param_.init_y_offset;
  out_emitter << YAML::Key << "z_offset" << YAML::Value << walking_param_.init_z_offset;
  out_emitter << YAML::Key << "roll_offset" << YAML::Value << walking_param_.init_roll_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "pitch_offset" << YAML::Value << walking_param_.init_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "yaw_offset" << YAML::Value << walking_param_.init_yaw_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "hip_pitch_offset" << YAML::Value << walking_param_.hip_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "period_time" << YAML::Value << walking_param_.period_time * 1000;
  out_emitter << YAML::Key << "dsp_ratio" << YAML::Value << walking_param_.dsp_ratio;
  out_emitter << YAML::Key << "step_forward_back_ratio" << YAML::Value << walking_param_.step_fb_ratio;
  out_emitter << YAML::Key << "foot_height" << YAML::Value << walking_param_.z_move_amplitude;
  out_emitter << YAML::Key << "swing_right_left" << YAML::Value << walking_param_.y_swap_amplitude;
  out_emitter << YAML::Key << "swing_top_down" << YAML::Value << walking_param_.z_swap_amplitude;
  out_emitter << YAML::Key << "pelvis_offset" << YAML::Value << walking_param_.pelvis_offset * RADIAN2DEGREE;
  // out_emitter << YAML::Key << "arm_swing_gain" << YAML::Value << walking_param_.arm_swing_gain;
  out_emitter << YAML::Key << "balance_hip_roll_gain" << YAML::Value << walking_param_.balance_hip_roll_gain;
  out_emitter << YAML::Key << "balance_knee_gain" << YAML::Value << walking_param_.balance_knee_gain;
  out_emitter << YAML::Key << "balance_ankle_roll_gain" << YAML::Value << walking_param_.balance_ankle_roll_gain;
  out_emitter << YAML::Key << "balance_ankle_pitch_gain" << YAML::Value << walking_param_.balance_ankle_pitch_gain;
  out_emitter << YAML::EndMap;

  // output to file
  std::ofstream fout(path.c_str());
  fout << out_emitter.c_str();
}

void WalkingModule::onModuleEnable()
{
  ROS_INFO("[WalkingModule] Module Enabled");
  walking_state_ = WALK_ENABLE;
}

void WalkingModule::onModuleDisable()
{
  ROS_INFO("[WalkingModule] Module Disabled");
  walking_state_ = WALK_DISABLE;
}

void WalkingModule::iniPoseTraGene(double mov_time)
{
  double smp_time = control_cycle_msec_ * 0.001;
  int all_time_steps = int(mov_time / smp_time) + 1;
  calc_joint_trajectory_.resize(all_time_steps, result_.size() + 1);

  for (int id = 0; id < result_.size(); id++)
  {
    double ini_value = goal_position_.coeff(0, id);
    double tar_value = target_position_.coeff(0, id);

    Eigen::MatrixXd tra;

    tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, smp_time, mov_time);

    calc_joint_trajectory_.block(0, id + 1, all_time_steps, 1) = tra;
  }

  if (debug_)
    std::cout << "Generate Trajecotry : " << mov_time << "s [" << all_time_steps << "]" << std::endl;

  init_pose_count_ = 0;
}
}  // namespace motion_control
