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
  previous_step_length_x_ = 0;
  previous_step_length_y_ = 0;
  previous_step_length_yaw_ = 0;
  previous_foot_lift_height_ = 0;

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
  walking_param_.init_hip_pitch_offset = 13.0 * DEGREE2RADIAN;
  // time
  walking_param_.period_time = 600 * 0.001;
  walking_param_.dsp_ratio = 0.1;
  walking_param_.step_forward_back_ratio = 0.28;
  // walking
  walking_param_.x_step = 0.0;
  walking_param_.y_step = 0.0;
  walking_param_.foot_height = 0.040;  // foot height
  walking_param_.yaw_step = 0.0;
  // balance
  walking_param_.balance_enable = false;
  walking_param_.balance_gyro_roll_gain = 0.5;
  walking_param_.balance_gyro_pitch_gain = 0.3;
  walking_param_.balance_gyro_y_gain = 1.0;
  walking_param_.balance_gyro_x_gain = 0.9;
  walking_param_.y_swing_amplitude = 0.020;
  walking_param_.z_swing_amplitude = 0.005;
  walking_param_.hip_swing_up_amplitude = 3.0 * DEGREE2RADIAN;
  walking_param_.shoulder_swing_amplitude = 1.5;

  // member variable
  body_swing_y_ = 0;
  body_swing_z_ = 0;

  request_walk_ = false;
  is_walking_ = false;
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

void WalkingModule::walkingParameterCallback(const kuroko_walking_module_msgs::WalkingParam::ConstPtr& msg)
{
  walking_param_ = *msg;
}

bool WalkingModule::getWalkigParameterCallback(kuroko_walking_module_msgs::GetWalkingParam::Request& /*req*/,
                                               kuroko_walking_module_msgs::GetWalkingParam::Response& res)
{
  res.parameters = walking_param_;

  return true;
}

double WalkingModule::wSin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2.0 * M_PI / period * time - period_shift) + mag_shift;
}

void WalkingModule::updateTimeParam()
{
  walk_period_ = walking_param_.period_time;
  dsp_ratio_ = walking_param_.dsp_ratio;

  l_ssp_start_time_ = dsp_ratio_ * walk_period_ / 4.0;
  l_ssp_end_time_ = (2.0 - dsp_ratio_) * walk_period_ / 4.0;
  r_ssp_start_time_ = (2.0 + dsp_ratio_) * walk_period_ / 4.0;
  r_ssp_end_time_ = (4.0 - dsp_ratio_) * walk_period_ / 4.0;

  phase1_time_ = (l_ssp_start_time_ + l_ssp_end_time_) / 2.0;
  phase2_time_ = (l_ssp_end_time_ + r_ssp_start_time_) / 2.0;
  phase3_time_ = (r_ssp_start_time_ + r_ssp_end_time_) / 2.0;

  hip_swing_up_amplitude_ = walking_param_.hip_swing_up_amplitude;
  hip_swing_down_amplitude_ = walking_param_.hip_swing_down_amplitude;
  shoulder_swing_amplitude_ = walking_param_.shoulder_swing_amplitude;
}

void WalkingModule::updateMovementParam()
{
  // Step length
  step_length_x_ = walking_param_.x_step;
  step_length_y_ = walking_param_.y_step / 2;
  step_length_yaw_ = walking_param_.yaw_step;

  // Body Forward/Back Swing
  x_swing_amplitude_ = step_length_x_ * walking_param_.step_forward_back_ratio;

  // Body Right/Left Swing
  y_swing_amplitude_ = walking_param_.y_swing_amplitude;

  // Body Up/Down Swing
  z_swing_amplitude_ = walking_param_.z_swing_amplitude;

  // Foot Up/Down Swing
  foot_lift_height_ = walking_param_.foot_height;

  // Remember one previous stride to prevent a sudden change in stride
  previous_step_length_x_ = step_length_x_;
  previous_step_length_y_ = step_length_y_;
  previous_step_length_yaw_ = step_length_yaw_;
  previous_foot_lift_height_ = foot_lift_height_;
}

void WalkingModule::updatePoseParam()
{
  init_x_offset_ = walking_param_.init_x_offset;
  init_y_offset_ = walking_param_.init_y_offset;
  init_z_offset_ = walking_param_.init_z_offset;
  init_roll_offset_ = walking_param_.init_roll_offset;
  init_pitch_offset_ = walking_param_.init_pitch_offset;
  init_yaw_offset_ = walking_param_.init_yaw_offset;
  init_hip_pitch_offset_ = walking_param_.init_hip_pitch_offset;
}

void WalkingModule::startWalking()
{
  request_walk_ = true;
  is_walking_ = true;

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start walking");
}

void WalkingModule::stop()
{
  request_walk_ = false;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Stop walking");
}

bool WalkingModule::isRunning()
{
  return is_walking_ || (walking_state_ == WALK_INITIAL_POSE);
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
      ROS_INFO_STREAM_COND(debug_, "init_hip_pitch_offset: " << walking_param_.init_hip_pitch_offset * RADIAN2DEGREE);
      ROS_INFO_STREAM_COND(debug_, "period_time: " << walking_param_.period_time * 1000);
      ROS_INFO_STREAM_COND(debug_, "dsp_ratio: " << walking_param_.dsp_ratio);
      ROS_INFO_STREAM_COND(debug_, "step_forward_back_ratio: " << walking_param_.step_forward_back_ratio);
      ROS_INFO_STREAM_COND(debug_, "foot_height: " << walking_param_.foot_height);
      ROS_INFO_STREAM_COND(debug_, "y_swing_amplitude: " << walking_param_.y_swing_amplitude);
      ROS_INFO_STREAM_COND(debug_, "z_swing_amplitude: " << walking_param_.z_swing_amplitude);
      ROS_INFO_STREAM_COND(debug_, "hip_swing_up_amplitude: " << walking_param_.hip_swing_up_amplitude * RADIAN2DEGREE);
      ROS_INFO_STREAM_COND(debug_,
                           "hip_swing_down_amplitude: " << walking_param_.hip_swing_down_amplitude * RADIAN2DEGREE);
      ROS_INFO_STREAM_COND(debug_, "shoulder_swing_amplitude: " << walking_param_.shoulder_swing_amplitude);
      ROS_INFO_STREAM_COND(debug_, "balance_gyro_roll_gain: " << walking_param_.balance_gyro_roll_gain);
      ROS_INFO_STREAM_COND(debug_, "balance_gyro_pitch_gain: " << walking_param_.balance_gyro_pitch_gain);
      ROS_INFO_STREAM_COND(debug_, "balance_gyro_y_gain: " << walking_param_.balance_gyro_y_gain);
      ROS_INFO_STREAM_COND(debug_, "balance_gyro_x_gain: " << walking_param_.balance_gyro_x_gain);
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

  if (is_walking_)
  {
    time_ += time_unit;
    if (time_ >= walk_period_)
      time_ -= walk_period_;  // reset time
  }
}

void WalkingModule::processPhase(const double& time_unit)
{
  bool can_stop = false;

  // Update walk phase
  if (time_ >= -time_unit * 0.5 && time_ < time_unit * 0.5)
  {
    // middle of double support state
    can_stop = true;
    updateTimeParam();
    phase_ = PHASE0;
  }
  else if (time_ >= (phase1_time_ - time_unit * 0.5) && time_ < (phase1_time_ + time_unit * 0.5))
  {
    // the position of left foot is the highest
    updateMovementParam();
    phase_ = PHASE1;
  }
  else if (time_ >= (phase2_time_ - time_unit * 0.5) && time_ < (phase2_time_ + time_unit * 0.5))
  {
    // middle of double support state
    can_stop = true;
    updateTimeParam();
    phase_ = PHASE2;
  }
  else if (time_ >= (phase3_time_ - time_unit * 0.5) && time_ < (phase3_time_ + time_unit * 0.5))
  {
    // the position of right foot is the highest
    updateMovementParam();
    phase_ = PHASE3;
  }

  // Receive Stop Request
  if (can_stop && !request_walk_)
  {
    walking_param_.x_step = 0;
    walking_param_.y_step = 0;
    walking_param_.yaw_step = 0;
    // Finish Walk
    if (phase_ == PHASE0 || phase_ == PHASE2)
    {
      if (fabs(step_length_x_) < 0.001 && fabs(step_length_y_) < 0.001 && fabs(step_length_yaw_) < 0.001)
      {
        step_length_x_ = step_length_y_ = step_length_yaw_ = 0;
        previous_step_length_x_ = previous_step_length_y_ = previous_step_length_yaw_ = 0;
        previous_foot_lift_height_ = 0;
        is_walking_ = false;
      }
    }
  }
}

bool WalkingModule::updateLegTargetAngles(std::vector<double>& leg_joints)
{
  Pose3D body, r_foot, l_foot;
  double r_hip_roll_swing, l_hip_roll_swing;
  std::vector<double> r_target_pose(6, 0);
  std::vector<double> l_target_pose(6, 0);

  std::vector<double> r_leg_joints(6, 0);
  std::vector<double> l_leg_joints(6, 0);

  updatePoseParam();

  // Compute endpoints
  body.x_ = wSin(time_, walk_period_ * 0.5, M_PI, x_swing_amplitude_, 0);
  body.y_ = wSin(time_, walk_period_, 0, y_swing_amplitude_, 0);
  body.z_ = wSin(time_, walk_period_ * 0.5, M_PI * 1.5, z_swing_amplitude_, z_swing_amplitude_);
  body.roll_ = 0.0;
  body.pitch_ = 0.0;
  body.yaw_ = 0.0;
  l_hip_roll_swing = 0;
  r_hip_roll_swing = 0;

  double dsp_period = walk_period_ * dsp_ratio_ / 2.0;
  double ssp_period = walk_period_ * (1.0 - dsp_ratio_) / 2.0;

  if (time_ <= l_ssp_start_time_)
  {
    // Shift center of gravity to the left foot while keeping both feet attached
    l_foot.x_ = wSin(dsp_ratio_ * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     step_length_x_, 0);
    l_foot.y_ = wSin(dsp_ratio_ * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     step_length_y_, fabs(step_length_y_));
    l_foot.z_ =
        wSin(dsp_ratio_ * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_) / 2.0,
             M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
             foot_lift_height_ / 2.0, foot_lift_height_ / 2.0);
    l_foot.yaw_ = wSin(dsp_ratio_ * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                       M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                       step_length_yaw_, fabs(step_length_yaw_));
    r_foot.x_ = wSin(dsp_ratio_ * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     -step_length_x_, -0);
    r_foot.y_ = wSin(dsp_ratio_ * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     -step_length_y_, -fabs(step_length_y_));
    r_foot.z_ = wSin((2.0 + dsp_ratio_) * walk_period_ / 4.0, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
                     M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * (2.0 + dsp_ratio_) *
                                      walk_period_ / 4.0,
                     (foot_lift_height_ / 2.0), foot_lift_height_ / 2.0);
    r_foot.yaw_ = wSin(dsp_ratio_ * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                       M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                       -step_length_yaw_, -fabs(step_length_yaw_));
  }
  else if (time_ <= l_ssp_end_time_)
  {
    // Lift left leg and move left leg forward, move right leg backward
    l_foot.x_ = wSin(time_, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     step_length_x_, 0);
    l_foot.y_ = wSin(time_, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     step_length_y_, fabs(step_length_y_));
    l_foot.z_ =
        wSin(time_, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
             M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * dsp_ratio_ * walk_period_ / 4.0,
             (foot_lift_height_ / 2.0), (foot_lift_height_ / 2.0));
    l_foot.yaw_ = wSin(time_, walk_period_ * (1.0 - dsp_ratio_),
                       M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                       step_length_yaw_, fabs(step_length_yaw_));
    r_foot.x_ = wSin(time_, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     -step_length_x_, -0);
    r_foot.y_ = wSin(time_, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     -step_length_y_, -fabs(step_length_y_));
    r_foot.z_ = wSin((2.0 + dsp_ratio_) * walk_period_ / 4.0, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
                     M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * (2.0 + dsp_ratio_) *
                                      walk_period_ / 4.0,
                     (foot_lift_height_ / 2.0), (foot_lift_height_ / 2.0));
    r_foot.yaw_ = wSin(time_, walk_period_ * (1.0 - dsp_ratio_),
                       M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                       -step_length_yaw_, -fabs(step_length_yaw_));
  }
  else if (time_ <= r_ssp_start_time_)
  {
    // Shift center of gravity to the right foot while keeping both feet attached
    l_foot.x_ = wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     step_length_x_, 0);
    l_foot.y_ = wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     step_length_y_, fabs(step_length_y_));
    l_foot.z_ =
        wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
             M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * dsp_ratio_ * walk_period_ / 4.0,
             (foot_lift_height_ / 2.0), (foot_lift_height_ / 2.0));
    l_foot.yaw_ = wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                       M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                       step_length_yaw_, fabs(step_length_yaw_));
    r_foot.x_ = wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     -step_length_x_, -0);
    r_foot.y_ = wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                     M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                     -step_length_y_, -fabs(step_length_y_));
    r_foot.z_ = wSin((2.0 + dsp_ratio_) * walk_period_ / 4.0, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
                     M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * (2.0 + dsp_ratio_) *
                                      walk_period_ / 4.0,
                     (foot_lift_height_ / 2.0), (foot_lift_height_ / 2.0));
    r_foot.yaw_ = wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
                       M_PI / 2.0 + 2.0 * M_PI * dsp_ratio_ * walk_period_ / 4.0 / (walk_period_ * (1.0 - dsp_ratio_)),
                       -step_length_yaw_, -fabs(step_length_yaw_));
  }
  else if (time_ <= r_ssp_end_time_)
  {
    // Lift Right leg and move right leg forward, move left leg backward
    l_foot.x_ = wSin(
        time_, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        step_length_x_, 0);
    l_foot.y_ = wSin(
        time_, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        step_length_y_, fabs(step_length_y_));
    l_foot.z_ =
        wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
             M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * dsp_ratio_ * walk_period_ / 4.0,
             (foot_lift_height_ / 2.0), (foot_lift_height_ / 2.0));
    l_foot.yaw_ = wSin(
        time_, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        step_length_yaw_, fabs(step_length_yaw_));
    r_foot.x_ = wSin(
        time_, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        -step_length_x_, -0);
    r_foot.y_ = wSin(
        time_, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        -step_length_y_, -fabs(step_length_y_));
    r_foot.z_ = wSin(time_, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
                     M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * (2.0 + dsp_ratio_) *
                                      walk_period_ / 4.0,
                     (foot_lift_height_ / 2.0), (foot_lift_height_ / 2.0));
    r_foot.yaw_ = wSin(
        time_, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        -step_length_yaw_, -fabs(step_length_yaw_));
  }
  else
  {
    // Shift center of gravity to the left foot while keeping both feet attached
    l_foot.x_ = wSin(
        (4.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        step_length_x_, 0);
    l_foot.y_ = wSin(
        (4.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        step_length_y_, fabs(step_length_y_));
    l_foot.z_ =
        wSin((2.0 - dsp_ratio_) * walk_period_ / 4.0, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
             M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * dsp_ratio_ * walk_period_ / 4.0,
             (foot_lift_height_ / 2.0), (foot_lift_height_ / 2.0));
    l_foot.yaw_ = wSin(
        (4.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        step_length_yaw_, fabs(step_length_yaw_));
    r_foot.x_ = wSin(
        (4.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        -step_length_x_, -0);
    r_foot.y_ = wSin(
        (4.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        -step_length_y_, -fabs(step_length_y_));
    r_foot.z_ = wSin((4.0 - dsp_ratio_) * walk_period_ / 4.0, (walk_period_ * (1.0 - dsp_ratio_) / 2.0),
                     M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_) / 2.0) * (2.0 + dsp_ratio_) *
                                      walk_period_ / 4.0,
                     (foot_lift_height_ / 2.0), (foot_lift_height_ / 2.0));
    r_foot.yaw_ = wSin(
        (4.0 - dsp_ratio_) * walk_period_ / 4.0, walk_period_ * (1.0 - dsp_ratio_),
        M_PI / 2.0 + 2.0 * M_PI / (walk_period_ * (1.0 - dsp_ratio_)) * (2.0 + dsp_ratio_) * walk_period_ / 4.0 + M_PI,
        -step_length_yaw_, -fabs(step_length_yaw_));
  }

  l_foot.roll_ = 0;
  l_foot.pitch_ = 0;
  r_foot.roll_ = 0;
  r_foot.pitch_ = 0;

  // mm, rad
  // Right leg target point
  r_target_pose[0] = body.x_ + r_foot.x_ + init_x_offset_;
  r_target_pose[1] = body.y_ + r_foot.y_ - (init_y_offset_ + leg_default_separaion_) / 2.0;
  r_target_pose[2] = body.z_ + r_foot.z_ + init_z_offset_ - leg_default_length_;
  r_target_pose[3] = body.roll_ + r_foot.roll_ - init_roll_offset_ / 2.0;
  r_target_pose[4] = body.pitch_ + r_foot.pitch_ + init_pitch_offset_;
  r_target_pose[5] = body.yaw_ + r_foot.yaw_ - init_yaw_offset_ / 2.0;
  // Left leg target point
  l_target_pose[0] = body.x_ + l_foot.x_ + init_x_offset_;
  l_target_pose[1] = body.y_ + l_foot.y_ + (init_y_offset_ + leg_default_separaion_) / 2.0;
  l_target_pose[2] = body.z_ + l_foot.z_ + init_z_offset_ - leg_default_length_;
  l_target_pose[3] = body.roll_ + l_foot.roll_ + init_roll_offset_ / 2.0;
  l_target_pose[4] = body.pitch_ + l_foot.pitch_ + init_pitch_offset_;
  l_target_pose[5] = body.yaw_ + l_foot.yaw_ + init_yaw_offset_ / 2.0;

  // Compute body swing
  if (time_ <= l_ssp_end_time_)
  {
    body_swing_y_ = -l_target_pose[1];
    body_swing_z_ = l_target_pose[2];
  }
  else
  {
    body_swing_y_ = -r_target_pose[1];
    body_swing_z_ = r_target_pose[2];
  }
  body_swing_z_ -= leg_default_length_;

  // Right leg IK
  if (!kuroko_kinematics_->solveInverseKinematicsForRightLeg(r_leg_joints, r_target_pose))
  {
    printf("IK not Solved EPR : %f %f %f %f %f %f\n", r_target_pose[0], r_target_pose[1], r_target_pose[2],
           r_target_pose[3], r_target_pose[4], r_target_pose[5]);
    return false;
  }
  // Left leg IK
  if (!kuroko_kinematics_->solveInverseKinematicsForLeftLeg(l_leg_joints, l_target_pose))
  {
    printf("IK not Solved EPL : %f %f %f %f %f %f\n", l_target_pose[0], l_target_pose[1], l_target_pose[2],
           l_target_pose[3], l_target_pose[4], l_target_pose[5]);
    return false;
  }

  // Add offset angles [rad]
  // Hip Roll Offset
  r_leg_joints[0] += kuroko_kinematics_->getJointDirection("hip_r_roll") * r_hip_roll_swing;
  l_leg_joints[0] += kuroko_kinematics_->getJointDirection("hip_l_roll") * l_hip_roll_swing;
  // Hip Pitch Offset
  r_leg_joints[1] -= kuroko_kinematics_->getJointDirection("hip_r_pitch") * init_hip_pitch_offset_;
  l_leg_joints[1] -= kuroko_kinematics_->getJointDirection("hip_l_pitch") * init_hip_pitch_offset_;

  leg_joints.resize(12);
  for (int i = 0; i < 6; i++)
  {
    leg_joints[i] = r_leg_joints[i];
    leg_joints[i + 6] = l_leg_joints[i];
  }

  return true;
}

void WalkingModule::gyroFeedback(const double& roll_gyro_err, const double& pitch_gyro_err,
                                 std::vector<double>& balance_angle)
{
  if (!static_cast<bool>(walking_param_.balance_enable))
    return;

  // Roll joints
  balance_angle[joint_table_["hip_r_roll"]] =
      kuroko_kinematics_->getJointDirection("hip_r_roll") * roll_gyro_err *
      (walking_param_.balance_gyro_y_gain + walking_param_.balance_gyro_roll_gain);
  balance_angle[joint_table_["hip_l_roll"]] =
      kuroko_kinematics_->getJointDirection("hip_l_roll") * roll_gyro_err *
      (walking_param_.balance_gyro_y_gain + walking_param_.balance_gyro_roll_gain);
  balance_angle[joint_table_["ankle_r_roll"]] =
      -kuroko_kinematics_->getJointDirection("ankle_r_roll") * roll_gyro_err * walking_param_.balance_gyro_y_gain;
  balance_angle[joint_table_["ankle_l_roll"]] =
      -kuroko_kinematics_->getJointDirection("ankle_l_roll") * roll_gyro_err * walking_param_.balance_gyro_y_gain;

  // Pitch joints
  balance_angle[joint_table_["hip_r_pitch"]] =
      kuroko_kinematics_->getJointDirection("hip_r_pitch") * pitch_gyro_err *
      (walking_param_.balance_gyro_x_gain + walking_param_.balance_gyro_pitch_gain);
  balance_angle[joint_table_["hip_l_pitch"]] =
      kuroko_kinematics_->getJointDirection("hip_l_pitch") * pitch_gyro_err *
      (walking_param_.balance_gyro_x_gain + walking_param_.balance_gyro_pitch_gain);
  balance_angle[joint_table_["ankle_r_pitch"]] =
      -kuroko_kinematics_->getJointDirection("ankle_r_pitch") * roll_gyro_err * walking_param_.balance_gyro_x_gain;
  balance_angle[joint_table_["ankle_l_pitvh"]] =
      -kuroko_kinematics_->getJointDirection("ankle_l_pitch") * roll_gyro_err * walking_param_.balance_gyro_x_gain;
}

void WalkingModule::loadWalkingParam(const std::string& path)
{
  YAML::Node doc;
  ROS_INFO("WalkingModule::loadWalkingParam - Loading: %s", path.c_str());
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
  walking_param_.init_x_offset = doc["init_x_offset"].as<double>();
  walking_param_.init_y_offset = doc["init_y_offset"].as<double>();
  walking_param_.init_z_offset = doc["init_z_offset"].as<double>();
  walking_param_.init_roll_offset = doc["init_roll_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_pitch_offset = doc["init_pitch_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_yaw_offset = doc["init_yaw_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_hip_pitch_offset = doc["init_hip_pitch_offset"].as<double>() * DEGREE2RADIAN;
  // Cycle Time
  walking_param_.period_time = doc["period_time"].as<double>() * 0.001;  // ms -> s
  walking_param_.dsp_ratio = doc["dsp_ratio"].as<double>();
  walking_param_.step_forward_back_ratio = doc["step_forward_back_ratio"].as<double>();
  // Foot Height
  walking_param_.foot_height = doc["foot_height"].as<double>();
  // Swing parameters
  walking_param_.y_swing_amplitude = doc["y_swing_amplitude"].as<double>();
  walking_param_.z_swing_amplitude = doc["z_swing_amplitude"].as<double>();
  walking_param_.roll_swing_amplitude = doc["roll_swing_amplitude"].as<double>() * DEGREE2RADIAN;
  walking_param_.hip_swing_up_amplitude = doc["hip_swing_up_amplitude"].as<double>() * DEGREE2RADIAN;
  walking_param_.hip_swing_down_amplitude = doc["hip_swing_down_amplitude"].as<double>() * DEGREE2RADIAN;
  walking_param_.chest_swing_amplitude = doc["chest_swing_amplitude"].as<double>() * DEGREE2RADIAN;
  walking_param_.shoulder_swing_amplitude = doc["shoulder_swing_amplitude"].as<double>() * DEGREE2RADIAN;
  // Feedback parameters
  walking_param_.balance_gyro_roll_gain = doc["balance_gyro_roll_gain"].as<double>();
  walking_param_.balance_gyro_pitch_gain = doc["balance_gyro_pitch_gain"].as<double>();
  walking_param_.balance_gyro_y_gain = doc["balance_gyro_y_gain"].as<double>();
  walking_param_.balance_gyro_x_gain = doc["balance_gyro_x_gain"].as<double>();
}

void WalkingModule::saveWalkingParam(std::string& path)
{
  YAML::Emitter out_emitter;

  out_emitter << YAML::BeginMap;
  out_emitter << YAML::Key << "init_x_offset" << YAML::Value << walking_param_.init_x_offset;
  out_emitter << YAML::Key << "init_y_offset" << YAML::Value << walking_param_.init_y_offset;
  out_emitter << YAML::Key << "init_z_offset" << YAML::Value << walking_param_.init_z_offset;
  out_emitter << YAML::Key << "init_roll_offset" << YAML::Value << walking_param_.init_roll_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "init_pitch_offset" << YAML::Value << walking_param_.init_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "init_yaw_offset" << YAML::Value << walking_param_.init_yaw_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "init_hip_pitch_offset" << YAML::Value
              << walking_param_.init_hip_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "period_time" << YAML::Value << walking_param_.period_time * 1000;
  out_emitter << YAML::Key << "dsp_ratio" << YAML::Value << walking_param_.dsp_ratio;
  out_emitter << YAML::Key << "step_forward_back_ratio" << YAML::Value << walking_param_.step_forward_back_ratio;
  out_emitter << YAML::Key << "foot_height" << YAML::Value << walking_param_.foot_height;
  out_emitter << YAML::Key << "y_swing_amplitude" << YAML::Value << walking_param_.y_swing_amplitude;
  out_emitter << YAML::Key << "z_swing_amplitude" << YAML::Value << walking_param_.z_swing_amplitude;
  out_emitter << YAML::Key << "roll_swing_amplitude" << YAML::Value << walking_param_.roll_swing_amplitude;
  out_emitter << YAML::Key << "chest_swing_amplitude" << YAML::Value << walking_param_.chest_swing_amplitude;
  out_emitter << YAML::Key << "shoulder_swing_amplitude" << YAML::Value << walking_param_.shoulder_swing_amplitude;
  out_emitter << YAML::Key << "hip_swing_up_amplitude" << YAML::Value
              << walking_param_.hip_swing_up_amplitude * RADIAN2DEGREE;
  out_emitter << YAML::Key << "hip_swing_down_amplitude" << YAML::Value
              << walking_param_.hip_swing_down_amplitude * RADIAN2DEGREE;
  out_emitter << YAML::Key << "balance_gyro_roll_gain" << YAML::Value << walking_param_.balance_gyro_roll_gain;
  out_emitter << YAML::Key << "balance_gyro_pitch_gain" << YAML::Value << walking_param_.balance_gyro_pitch_gain;
  out_emitter << YAML::Key << "balance_gyro_y_gain" << YAML::Value << walking_param_.balance_gyro_y_gain;
  out_emitter << YAML::Key << "balance_gyro_x_gain" << YAML::Value << walking_param_.balance_gyro_x_gain;
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
