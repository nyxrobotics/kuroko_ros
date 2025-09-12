#include "roboone_auto.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <sensor_msgs/CameraInfo.h>
#include "ros/console.h"
#include "ros/duration.h"
#include "ros/time.h"

// コンストラクタ
RobooneAuto::RobooneAuto(ros::NodeHandle& nh)
  : atk_min_rect_size_(0.24)
  , class_sub_(nh, "/object_detection/output/class", 5)
  , label_sub_(nh, "/object_detection/output/labels", 5)
  , rect_sub_(nh, "/object_detection/output/rects", 5)
  , sync_(SyncPolicy(5), class_sub_, label_sub_, rect_sub_)
  , client_(nh.serviceClient<robotis_controller_msgs::SetModule>("/motion_control/set_present_ctrl_modules"))
{
  joy_sub_ = nh.subscribe("/gamepad/joy", 5, &RobooneAuto::joyCallback, this);
  imu_sub_ = nh.subscribe("/kuroko/sensors/imu/data", 5, &RobooneAuto::imuCallback, this);
  range_sub_ = nh.subscribe("/kuroko/sensors/range", 5, &RobooneAuto::rangeCallback, this);  // Dummy, not used
  camera_info_sub_ = nh.subscribe("/camera/resized/camera_info", 5, &RobooneAuto::cameraInfoCallback, this);

  sync_.registerCallback(boost::bind(&RobooneAuto::yoloCallback, this, _1, _2, _3));

  walking_command_pub_ = nh.advertise<std_msgs::String>("/motion_control/walking/command", 5);
  walking_params_pub_ = nh.advertise<kuroko_walking_module_msgs::WalkingParam>("/motion_control/walking/set_params", 5);
  action_page_pub_ = nh.advertise<std_msgs::Int32>("/motion_control/action/animation_num", 5);
  init_pose_pub_ = nh.advertise<std_msgs::String>("/motion_control/base/ini_pose", 5);

  current_state_ = "INITIAL";
  running_ = true;
  force_walk_ = false;

  // Initialize last_joy_ with default size
  last_joy_.axes.resize(2);
  last_joy_.buttons.resize(10);
  robot_detected_time_ = ros::Time(0);
  fall_detected_time_ = ros::Time(0);
  attacked_time_ = ros::Time(0);
  last_target_detected_direction_ = 1;
  last_attack_name_ = "";
  attack_rect_distance_ = 0.28;
  attack_count_ = 0;
  max_attack_count_ = 2;

  // Set max step sizes
  x_forward_step_max_ = 0.02;
  x_backward_step_max_ = -0.02;
  y_step_max_ = 0.02;
  yaw_step_max_ = 0.1;

  // Set walking params
  walk_param_.init_x_offset = 0.018;
  walk_param_.init_y_offset = 0.06;
  walk_param_.init_z_offset = 0.05;
  walk_param_.init_roll_offset = 0.1396;
  walk_param_.init_pitch_offset = 0.0;
  walk_param_.init_yaw_offset = 0.0;
  walk_param_.init_hip_pitch_offset = 0;
  walk_param_.init_pose_duration = 0.0;
  walk_param_.period_time = 0.43;
  walk_param_.dsp_ratio = 0.1;
  walk_param_.step_forward_back_ratio = 0.0;
  walk_param_.foot_height = 0.08;
  walk_param_.y_swing_amplitude = 0.016;
  walk_param_.z_swing_amplitude = 0.004;
  walk_param_.roll_swing_amplitude = -0.05236;
  walk_param_.roll_swing_phase = 0.3491;
  walk_param_.hip_swing_up_amplitude = 0.03491;
  walk_param_.hip_swing_down_amplitude = -0.03491;
  walk_param_.shoulder_swing_amplitude = 0;
  walk_param_.chest_swing_amplitude = 0;
  walk_param_.balance_enable = true;
  walk_param_.balance_gyro_x_gain = 0.008;
  walk_param_.balance_gyro_y_gain = -0.004;
  walk_param_.balance_gyro_zx_gain = 0.004;
  walk_param_.balance_gyro_zy_gain = 0.008;
  walk_param_.balance_gyro_roll_gain = -0.01;
  walk_param_.balance_gyro_pitch_gain = 0.04;
  walk_param_.balance_euler_x_gain = 0.04;
  walk_param_.balance_euler_y_gain = -0.02;
  walk_param_.balance_euler_zx_gain = 0.004;
  walk_param_.balance_euler_zy_gain = 0.008;
  walk_param_.balance_euler_roll_gain = -0.01;
  walk_param_.balance_euler_pitch_gain = 0.04;
  walk_param_.p_gain = 0;
  walk_param_.i_gain = 0;
  walk_param_.d_gain = 0;
  walk_param_.x_step = 0;
  walk_param_.y_step = 0;
  walk_param_.yaw_step = 0;

  idle_param_ = walk_param_;
  idle_param_.init_pose_duration = 0.1;
  idle_param_.balance_enable = false;

  squat_param_ = walk_param_;
  squat_param_.init_x_offset = 0;
  squat_param_.init_z_offset = 0.12;
  squat_param_.balance_gyro_x_gain = 0.004;
  squat_param_.balance_gyro_y_gain = 0.0;
  squat_param_.balance_gyro_zx_gain = 0.004;
  squat_param_.balance_gyro_zy_gain = 0.0;
  squat_param_.balance_gyro_roll_gain = 0.0;
  squat_param_.balance_gyro_pitch_gain = 0.02;
  squat_param_.balance_euler_x_gain = 0.06;
  squat_param_.balance_euler_y_gain = 0.0;
  squat_param_.balance_euler_zx_gain = 0.01;
  squat_param_.balance_euler_zy_gain = 0.0;
  squat_param_.balance_euler_roll_gain = 0.0;
  squat_param_.balance_euler_pitch_gain = 0.8;

  jump_param_ = squat_param_;
  jump_param_.init_z_offset = 0.08;
  jump_param_.balance_gyro_x_gain = 0.01;
  jump_param_.balance_gyro_y_gain = 0.0;
  jump_param_.balance_gyro_zx_gain = -0.01;
  jump_param_.balance_gyro_zy_gain = 0.0;
  jump_param_.balance_gyro_roll_gain = 0.0;
  jump_param_.balance_gyro_pitch_gain = 0.1;
  jump_param_.balance_euler_x_gain = 0.2;
  jump_param_.balance_euler_y_gain = 0.0;
  jump_param_.balance_euler_zx_gain = -0.04;
  jump_param_.balance_euler_zy_gain = 0.0;
  jump_param_.balance_euler_roll_gain = 0.0;
  jump_param_.balance_euler_pitch_gain = 1.0;

  stable_detect_angle_ = 0.12;
  squat_detect_angle_ = 0.16;
  jump_detect_angle_ = 3.32;
  fall_detect_angle_ = 0.64;

  stable_detect_duration_ = 0.8;
  squat_detect_duration_ = 0.1;
  jump_detect_duration_ = 0.1;
  fall_detect_duration_ = 0.4;

  jump_duration_ = 0.2;

  walk_start_time_ = ros::Time(0);
  walk_stop_duration_ = walk_param_.period_time * 1.0;
  min_walk_duration_ = walk_param_.period_time * 1.0;

  squat_start_time_ = ros::Time(0);
  min_squat_duration_ = 0.2;
  max_squat_duration_ = 2.8;
  squat_duration_ = 0;

  jump_start_time_ = ros::Time(0);
  jump_duration_ = 0.1;

  action_start_time_ = ros::Time(0);

  run_start_time_ = ros::Time(0);
  run_startup_duration_ = 0.2;

  walk_status_ = "stop";
  current_module_ = "";

  action_id_map_["disable"] = -2;
  action_id_map_["enable"] = -1;
  action_id_map_["crazy_catch"] = 0;
  action_id_map_["crazy_kick"] = 1;
  action_id_map_["crouch_down"] = 2;
  action_id_map_["crouch_up"] = 3;
  action_id_map_["getup_front"] = 4;
  action_id_map_["getup_rear"] = 5;
  action_id_map_["l_crazy_spin"] = 6;
  action_id_map_["l_grip_front"] = 7;
  action_id_map_["l_hook_front"] = 8;
  action_id_map_["l_punch_high"] = 9;
  action_id_map_["l_punch_low"] = 10;
  action_id_map_["r_crazy_spin"] = 11;
  action_id_map_["r_grip_front"] = 12;
  action_id_map_["r_hook_front"] = 13;
  action_id_map_["r_punch_high"] = 14;
  action_id_map_["r_punch_low"] = 15;

  action_duration_map_["disable"] = 0;
  action_duration_map_["enable"] = 0;
  action_duration_map_["crazy_catch"] = 7.5;
  action_duration_map_["crazy_kick"] = 1.68 + 0.5;
  action_duration_map_["crouch_down"] = 0.1;
  action_duration_map_["crouch_up"] = 0;
  action_duration_map_["getup_front"] = 3.1 + 0.5;
  action_duration_map_["getup_rear"] = 4.9 + 0.5;
  action_duration_map_["l_crazy_spin"] = 3.94;
  action_duration_map_["l_grip_front"] = 1.94;
  action_duration_map_["l_hook_front"] = 1.3;
  action_duration_map_["l_punch_high"] = 1.51;
  action_duration_map_["l_punch_low"] = 1.68;
  action_duration_map_["r_crazy_spin"] = 3.94;
  action_duration_map_["r_grip_front"] = 1.94;
  action_duration_map_["r_hook_front"] = 1.3;
  action_duration_map_["r_punch_high"] = 1.51;
  action_duration_map_["r_punch_low"] = 1.68;

  attack_actions_.clear();
  AttackData attack_data;
  // Crazy attacks
  attack_data.normal_max_count = 1;
  attack_data.ultimate_max_count = -1;
  attack_data.current_count = 0;
  attack_data.force_aim = true;
  attack_data.name = "crazy_catch";
  attack_data.min_distance = 0.15;
  attack_data.max_distance = 0.25;
  attack_actions_.push_back(attack_data);
  attack_data.name = "crazy_kick";
  attack_data.min_distance = 0.8;
  attack_data.max_distance = 0.9;
  attack_actions_.push_back(attack_data);
  attack_data.name = "l_crazy_spin";
  attack_data.normal_max_count = 0;
  attack_data.min_distance = 0.75;
  attack_data.max_distance = 0.85;
  attack_actions_.push_back(attack_data);
  attack_data.name = "r_crazy_spin";
  attack_data.normal_max_count = 0;
  attack_data.min_distance = 0.75;
  attack_data.max_distance = 0.85;
  attack_actions_.push_back(attack_data);
  // Normal attacks
  attack_data.normal_max_count = -1;
  attack_data.ultimate_max_count = 0;
  attack_data.current_count = 0;
  attack_data.force_aim = false;
  attack_data.name = "l_grip_front";
  attack_data.min_distance = 0.2;
  attack_data.max_distance = 0.3;
  attack_actions_.push_back(attack_data);
  attack_data.name = "l_hook_front";
  attack_data.min_distance = 0.23;
  attack_data.max_distance = 0.33;
  attack_actions_.push_back(attack_data);
  attack_data.name = "l_punch_high";
  attack_data.min_distance = 0.23;
  attack_data.max_distance = 0.33;
  attack_actions_.push_back(attack_data);
  attack_data.name = "l_punch_low";
  attack_data.min_distance = 0.17;
  attack_data.max_distance = 0.27;
  attack_actions_.push_back(attack_data);
  attack_data.name = "r_grip_front";
  attack_data.min_distance = 0.2;
  attack_data.max_distance = 0.3;
  attack_actions_.push_back(attack_data);
  attack_data.name = "r_hook_front";
  attack_data.min_distance = 0.23;
  attack_data.max_distance = 0.33;
  attack_actions_.push_back(attack_data);
  attack_data.name = "r_punch_high";
  attack_data.min_distance = 0.23;
  attack_data.max_distance = 0.33;
  attack_actions_.push_back(attack_data);
  attack_data.name = "r_punch_low";
  attack_data.min_distance = 0.17;
  attack_data.max_distance = 0.27;
  attack_actions_.push_back(attack_data);

  ultimate_mode_ = false;
  force_aim_ = false;

  state_thread_ = std::thread(&RobooneAuto::stateThread, this);
  ROS_INFO("RobooneAuto initialized.");
}

// デストラクタ
RobooneAuto::~RobooneAuto()
{
  running_ = false;
  if (state_thread_.joinable())
  {
    state_thread_.join();
  }
}

// Function to set the control module
bool RobooneAuto::setCtrlModule(const std::string& module_name)
{
  robotis_controller_msgs::SetModule srv;
  srv.request.module_name = module_name;

  if (current_module_ == module_name)
  {
    // ROS_INFO("Control module is already set to %s", module_name.c_str());
    return true;
  }

  if (client_.call(srv))
  {
    ROS_INFO("Successfully set control module to %s", module_name.c_str());
    walk_status_ = "stop";
    current_module_ = module_name;
    ros::Duration(0.04).sleep();  // Wait for module to stabilize
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service to set control module to %s", module_name.c_str());
    current_module_ = "";
    return false;
  }
}

// Start walking
void RobooneAuto::startWalking()
{
  if (walk_status_ == "start")
    return;
  setCtrlModule("walking_module");
  walk_status_ = "start";
  ROS_INFO("Starting Walking...");
  std_msgs::String msg;
  msg.data = "start";
  walking_command_pub_.publish(msg);
  walk_start_time_ = ros::Time::now();
}

// Stop walking
void RobooneAuto::stopWalking()
{
  if (walk_status_ == "stop" || walk_status_ == "abort")
    return;
  if (current_module_ != "walking_module")
    return;
  walk_status_ = "stop";
  ROS_INFO("Stopping Walking...");
  std_msgs::String msg;
  msg.data = "stop";
  walking_command_pub_.publish(msg);
  ros::Duration(walk_stop_duration_).sleep();
}

// Abort walking
void RobooneAuto::abortWalking()
{
  if (walk_status_ == "abort")
    return;
  if (current_module_ != "walking_module")
    return;
  walk_status_ = "abort";
  ROS_INFO("Aborting Walking...");
  std_msgs::String msg;
  msg.data = "abort";
  walking_command_pub_.publish(msg);
}

// Set walking parameters with specified initial values
void RobooneAuto::setWalkSteps(double x_step, double y_step, double yaw_step)
{
  kuroko_walking_module_msgs::WalkingParam params = walk_param_;
  double x_scale = fabs(x_step / ((x_step > 0) ? (x_forward_step_max_) : (x_backward_step_max_)));
  double y_scale = fabs(y_step / y_step_max_);
  double yaw_scale = fabs(yaw_step / yaw_step_max_);
  double normalization_factor = sqrt(x_scale * x_scale + y_scale * y_scale + yaw_scale * yaw_scale);

  if (normalization_factor > 1.0)
  {
    x_step /= normalization_factor;
    y_step /= normalization_factor;
    yaw_step /= normalization_factor;
  }

  // Move amplitudes set dynamically
  params.x_step = x_step;
  params.y_step = y_step;
  params.yaw_step = yaw_step;

  // Publish the walking parameters
  walking_params_pub_.publish(params);
}

void RobooneAuto::setIdle()
{
  kuroko_walking_module_msgs::WalkingParam params = idle_param_;
  params.x_step = 0;
  params.y_step = 0;
  params.yaw_step = 0;

  // Publish the walking parameters
  walking_params_pub_.publish(params);
}

void RobooneAuto::setSquat()
{
  kuroko_walking_module_msgs::WalkingParam params = squat_param_;
  params.x_step = 0;
  params.y_step = 0;
  params.yaw_step = 0;

  // Publish the walking parameters
  walking_params_pub_.publish(params);
}

void RobooneAuto::setFrontJump()
{
  kuroko_walking_module_msgs::WalkingParam params = jump_param_;
  params.x_step = 0;
  params.y_step = 0;
  params.yaw_step = 0;

  // Publish the walking parameters
  walking_params_pub_.publish(params);
}

// Execute an action by name
void RobooneAuto::executeAction(std::string action_name)
{
  int action_id = 0;
  double action_duration = 0;

  auto it_id = action_id_map_.find(action_name);
  if (it_id != action_id_map_.end())
  {
    action_id = it_id->second;
  }
  else
  {
    ROS_ERROR_STREAM("Action name '" << action_name << "' not found in action ID map.");
    action_name_ = "";
    return;
  }

  auto it_duration = action_duration_map_.find(action_name);
  if (it_duration != action_duration_map_.end())
  {
    action_duration = it_duration->second;
  }
  else
  {
    ROS_ERROR_STREAM("Action name '" << action_name << "' not found in action duration map.");
    action_name_ = "";
    return;
  }
  ROS_INFO_STREAM("Executing action: " << action_name << " with ID: " << action_id
                                       << " for duration: " << action_duration);
  std_msgs::Int32 msg;
  msg.data = action_id;

  setWalkSteps(0, 0, 0);
  action_page_pub_.publish(msg);
  action_name_ = action_name;
  action_start_time_ = ros::Time::now();
  action_duration_ = action_duration;
}

// 状態管理スレッド
void RobooneAuto::stateThread()
{
  while (running_ && ros::ok())
  {
    manageState();
  }
}

// 状態管理処理
void RobooneAuto::manageState()
{
  Eigen::Quaterniond imy_orientation(last_imu_.orientation.w, last_imu_.orientation.x, last_imu_.orientation.y,
                                     last_imu_.orientation.z);
  Eigen::Vector3d imu_rpy = imuQuaternionToRollPitchYaw(imy_orientation);
  bool over_fall_angle = fabs(imu_rpy[1]) > fall_detect_angle_;
  bool over_jump_angle = fabs(imu_rpy[1]) > jump_detect_angle_;
  bool over_squat_angle = fabs(imu_rpy[1]) > squat_detect_angle_;
  bool within_stable_angle = fabs(imu_rpy[1]) < stable_detect_angle_;

  // ボタン検知
  if (current_state_ == "INITIAL" && last_joy_.buttons[2])
  {
    // Triangle
    force_walk_ = false;
    ROS_INFO("Transitioning to RUN state from INITIAL.");
    action_name_ = "";
    transitionToRun();
    return;
  }
  else if (current_state_ != "FREE" && last_joy_.buttons[1])
  {
    // Circle
    force_walk_ = false;
    ROS_INFO("Transitioning to FREE state from current state: %s", current_state_.c_str());
    action_name_ = "";
    transitionToFree();
    return;
  }
  else if (current_state_ != "INITIAL" && last_joy_.buttons[0])
  {
    // Cross
    force_walk_ = false;
    ROS_INFO("Transitioning to INITIAL state from current state: %s", current_state_.c_str());
    action_name_ = "";
    transitionToInit();
    return;
  }
  // Ultimate mode
  if (last_joy_.axes[7] < -0.9)
  {
    // Up
    if (ultimate_mode_)
      ROS_INFO("Ultimate mode deactivated. Normal attacks only.");
    ultimate_mode_ = false;
  }
  else if (last_joy_.axes[7] > 0.9)
  {
    // Down
    if (!ultimate_mode_)
      ROS_INFO("Ultimate mode activated. Crazy attacks enabled.");
    ultimate_mode_ = true;
  }
  if (last_joy_.axes[6] > 0.9)
  {
    // Left
    // Resset current attack counts
    ROS_INFO_THROTTLE(1.0, "Resetting attack counts.");
    for (auto& attack : attack_actions_)
    {
      attack.current_count = 0;
    }
  }

  // Sleep
  if (current_state_ == "FREE" || current_state_ == "INITIAL")
  {
    action_name_ = "";
    ros::Duration(0.1).sleep();
    return;
  }
  if (action_name_ != "")
  {
    if ((ros::Time::now() - action_start_time_).toSec() > action_duration_)
    {
      ROS_INFO_STREAM("Action " << action_name_ << " completed"
                                << " after " << action_duration_ << " seconds.");
      action_start_time_ = ros::Time::now();
      action_name_ = "";
      if (over_jump_angle)
        setFrontJump();
      else if (over_squat_angle)
        setSquat();
      else
        setWalkSteps(0, 0, 0);
      setCtrlModule("walking_module");
      last_imu_.orientation.x = 0.0;
      last_imu_.orientation.y = 0.0;
      last_imu_.orientation.z = 0.0;
      last_imu_.orientation.w = 1.0;
      over_fall_angle = false;
      over_squat_angle = false;
      within_stable_angle = true;
      walk_start_time_ = ros::Time::now();
      attacked_time_ = ros::Time(0);
      transitionToHold();
    }
    else
    {
      double remaining_time = action_duration_ - (ros::Time::now() - action_start_time_).toSec();
      if (remaining_time < 0.1)
        ros::Duration(remaining_time).sleep();
      else
        ros::Duration(0.1).sleep();
      return;  // Action is still ongoing, skip state management
    }
  }

  // 状態遷移
  if (current_state_ == "RUN")
  {
    if (over_squat_angle)
    {
      // 転倒検知
      transitionToHold();
    }
    else
    {
      handleRun();
    }
  }
  else if (current_state_ == "HOLD")
  {
    if (over_fall_angle)
    {
      transitionToFall();
    }
    else if (over_squat_angle && force_walk_ == false)
    {
      transitionToSquat();
    }
    else if (within_stable_angle)
    {
      ROS_INFO("IMU stabilized. Returning to RUN state.");
      transitionToRun();
    }
  }
  else if (current_state_ == "SQUAT")
  {
    force_walk_ = true;
    squat_duration_ = (ros::Time::now() - squat_start_time_).toSec();
    if (squat_duration_ > min_squat_duration_)
    {
      if (over_fall_angle)
      {
        ROS_INFO("Over fall angle detected in SQUAT state. Transitioning to FALL.");
        transitionToFall();
      }
      else if (over_jump_angle)
      {
        ROS_INFO("Over jump angle detected in SQUAT state. Transitioning to JUMP.");
        transitionToJump();
      }
      else if (within_stable_angle && squat_duration_ > stable_detect_duration_)
      {
        ROS_INFO("Stable angle restored. Transitioning to HOLD.");
        transitionToHold();
      }
    }
    if (squat_duration_ > max_squat_duration_)
    {
      ROS_INFO("Max squat duration exceeded. Transitioning to HOLD.");
      transitionToHold();
    }
  }
  else if (current_state_ == "JUMP")
  {
    force_walk_ = true;
    double jump_duration = (ros::Time::now() - jump_start_time_).toSec();
    if (jump_duration > jump_duration_)
    {
      if (over_fall_angle)
      {
        ROS_INFO("Over fall angle detected in JUMP state. Transitioning to FALL.");
        transitionToFall();
      }
      else if (within_stable_angle && jump_duration > stable_detect_duration_)
      {
        ROS_INFO("Stable angle restored. Transitioning to HOLD.");
        transitionToHold();
      }
      else if (!over_squat_angle)
      {
        ROS_INFO("Under squat angle detected in JUMP state. Transitioning to SQUAT.");
        transitionToSquat();
      }
    }
  }
  else if (current_state_ == "FALL")
  {
    force_walk_ = false;
    if (within_stable_angle)
    {
      ROS_INFO("Stable angle restored. Transitioning to HOLD.");
      transitionToHold();
      fall_detected_time_ = ros::Time::now() + ros::Duration(1.0);
    }
    else if (!over_squat_angle)
    {
      transitionToSquat();
    }
    else if ((ros::Time::now() - fall_detected_time_).toSec() > fall_detect_duration_)
    {
      ROS_INFO("Handling FALL state.");
      handleFall();
      transitionToHold();
    }
  }
}

// 初期姿勢への遷移
void RobooneAuto::transitionToInit()
{
  if (current_state_ == "INITIAL")
    return;
  current_state_ = "INITIAL";
  setIdle();
  enableAllJoints();
  ROS_INFO("Transitioning to INITIAL state.");
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub_.publish(init_msg);
  setCtrlModule("initial_pose_module");
  setCtrlModule("action_module");
  setCtrlModule("walking_module");
}

// 自律移動への遷移
void RobooneAuto::transitionToRun()
{
  if (current_state_ == "RUN")
    return;
  current_state_ = "RUN";
  ROS_INFO("Transitioning to RUN state.");
  setCtrlModule("walking_module");
  setWalkSteps(0, 0, 0);
  abortWalking();
  robot_detected_time_ = ros::Time(0);
  last_rects_.rects.clear();
  attacked_time_ = ros::Time(0);
  walk_start_time_ = ros::Time::now();
  run_start_time_ = ros::Time::now();
}

// 歩行一時停止状態への遷移
void RobooneAuto::transitionToHold()
{
  if (current_state_ == "HOLD")
    return;
  current_state_ = "HOLD";
  ROS_INFO("Transitioning to HOLD state due to excessive tilt.");
  setCtrlModule("walking_module");
  setWalkSteps(0, 0, 0);
  stopWalking();
}

// しゃがみ状態への遷移
void RobooneAuto::transitionToSquat()
{
  if (current_state_ == "SQUAT")
    return;
  current_state_ = "SQUAT";
  ROS_INFO("Transitioning to SQUAT state.");
  setCtrlModule("walking_module");
  setSquat();
  abortWalking();
  squat_start_time_ = ros::Time::now();
}

// ジャンプ状態への遷移
void RobooneAuto::transitionToJump()
{
  if (current_state_ == "JUMP")
    return;
  current_state_ = "JUMP";
  ROS_INFO("Transitioning to JUMP state.");
  setCtrlModule("walking_module");
  setFrontJump();
  abortWalking();
  jump_start_time_ = ros::Time::now();
}

// 転倒状態への遷移
void RobooneAuto::transitionToFall()
{
  if (current_state_ == "FALL")
    return;
  if (current_state_ != "SQUAT")
  {
    transitionToSquat();
  }
  fall_detected_time_ = ros::Time::now();
  current_state_ = "FALL";
  ROS_INFO("Transitioning to FALL state.");
}

// 脱力状態への遷移
void RobooneAuto::transitionToFree()
{
  freeAllJoints();
  current_state_ = "FREE";
}

// 転倒状態の処理
void RobooneAuto::handleFall()
{
  Eigen::Quaterniond imy_orientation(last_imu_.orientation.w, last_imu_.orientation.x, last_imu_.orientation.y,
                                     last_imu_.orientation.z);
  Eigen::Vector3d imu_rpy = imuQuaternionToRollPitchYaw(imy_orientation);
  double pitch_angle = imu_rpy[1];
  ROS_INFO("Handling FALL state with IMU RPY: roll=%f, pitch=%f", imu_rpy[0], imu_rpy[1]);
  setCtrlModule("action_module");
  if (imu_rpy[1] > 0)
  {
    executeAction("getup_front");  // 前起き上がりモーション
  }
  else
  {
    executeAction("getup_rear");  // 後起き上がりモーション
  }
  setWalkSteps(0, 0, 0);
  current_state_ = "HOLD";
}

// 攻撃処理
void RobooneAuto::handleRun()
{
  if (last_camera_info_.height == 0 || last_camera_info_.width == 0)
  {
    ROS_WARN("Camera info is missing.");
    return;
  }
  else if (!action_name_.empty())
  {
    return;
  }
  else if (ros::Time::now() - run_start_time_ < ros::Duration(run_startup_duration_))
  {
    return;
  }

  bool roboone_found = false;
  int roboone_count = 0;
  jsk_recognition_msgs::Rect largest_rect;
  jsk_recognition_msgs::Rect largest_conbined_rect;
  int largest_idx = -1;
  largest_rect.width = 0;
  largest_rect.height = 0;

  // robooneラベルを持つrectを探し、その中で一番大きいものを見つける
  if (ros::Time::now() - last_class_.header.stamp < ros::Duration(2.0) &&
      ros::Time::now() - last_rects_.header.stamp < ros::Duration(2.0))
  {
    for (size_t i = 0; i < last_rects_.rects.size(); ++i)
    {
      if (last_class_.label_names[i] == "roboone")
      {
        if (last_rects_.rects[i].width > 10 && last_rects_.rects[i].height > 10)
        {
          roboone_count++;
          if (largest_rect.width * largest_rect.height < last_rects_.rects[i].width * last_rects_.rects[i].height)
          {
            roboone_found = true;
            robot_detected_time_ = ros::Time::now();
            largest_rect = last_rects_.rects[i];
            robot_detected_rect_ = largest_rect;
            largest_idx = i;
          }
        }
      }
    }
  }

  // 一番大きいrectに重なる部分があるrectを結合する
  if (roboone_found)
  {
    largest_conbined_rect = largest_rect;
    for (size_t i = 0; i < last_rects_.rects.size(); ++i)
    {
      if (i != largest_idx && last_class_.label_names[i] == "roboone")
      {
        int x1 = std::max(largest_conbined_rect.x, last_rects_.rects[i].x);
        int y1 = std::max(largest_conbined_rect.y, last_rects_.rects[i].y);
        int x2 = std::min(largest_conbined_rect.x + largest_conbined_rect.width,
                          last_rects_.rects[i].x + last_rects_.rects[i].width);
        int y2 = std::min(largest_conbined_rect.y + largest_conbined_rect.height,
                          last_rects_.rects[i].y + last_rects_.rects[i].height);
        if (x1 < x2 && y1 < y2)
        {
          // 重なりあり
          int new_x = std::min(largest_conbined_rect.x, last_rects_.rects[i].x);
          int new_y = std::min(largest_conbined_rect.y, last_rects_.rects[i].y);
          int new_w = std::max(largest_conbined_rect.x + largest_conbined_rect.width,
                               last_rects_.rects[i].x + last_rects_.rects[i].width) -
                      new_x;
          int new_h = std::max(largest_conbined_rect.y + largest_conbined_rect.height,
                               last_rects_.rects[i].y + last_rects_.rects[i].height) -
                      new_y;
          largest_conbined_rect.x = new_x;
          largest_conbined_rect.y = new_y;
          largest_conbined_rect.width = new_w;
          largest_conbined_rect.height = new_h;
        }
      }
    }
    robot_detected_rect_ = largest_conbined_rect;
  }
  // ROS_INFO("Roboone detected: %d, robot_detected_rect=(x=%d, y=%d, w=%d, h=%d)", roboone_found, robot_detected_rect_.x,
  //          robot_detected_rect_.y, robot_detected_rect_.width, robot_detected_rect_.height);

  // 中心の上下左右10ピクセルがlargest_conbined_rectに内包されている場合は距離センサが有効
  bool range_available = false;
  int range_available_area_pixels = 10;
  if (ros::Time::now() - last_range_.header.stamp < ros::Duration(2.0) &&
      ros::Time::now() - robot_detected_time_ < ros::Duration(2.0) &&
      last_camera_info_.height / 2 - range_available_area_pixels > robot_detected_rect_.y &&
      last_camera_info_.height / 2 + range_available_area_pixels <
          robot_detected_rect_.y + robot_detected_rect_.height &&
      last_camera_info_.width / 2 - range_available_area_pixels > robot_detected_rect_.x &&
      last_camera_info_.width / 2 + range_available_area_pixels < robot_detected_rect_.x + robot_detected_rect_.width)
  {
    if (last_range_.range < 3.6)
    {
      range_available = true;
    }
  }
  last_rects_.rects.clear();

  // Walk force flag
  if (attack_count_ > max_attack_count_)
  {
    walk_start_time_ = ros::Time::now();
    squat_start_time_ = ros::Time::now();
    squat_duration_ = 0;
    attack_count_ = 0;
  }
  if (ros::Time::now() - walk_start_time_ > ros::Duration(min_walk_duration_) &&
      ros::Time::now() - squat_start_time_ > ros::Duration(min_walk_duration_ + squat_duration_) &&
      ros::Time::now() - robot_detected_time_ < ros::Duration(2.0))
  {
    force_walk_ = false;
  }

  // 相手ロボットの位置を算出
  double rect_area = (robot_detected_rect_.width * robot_detected_rect_.height) /
                     double(last_camera_info_.width * last_camera_info_.height);
  double rect_center_x = robot_detected_rect_.x + robot_detected_rect_.width / 2.0;
  double rect_center_y = robot_detected_rect_.y + robot_detected_rect_.height / 2.0;
  double image_center_x = last_camera_info_.width / 2.0;
  double image_center_y = last_camera_info_.height / 2.0;
  double x_offset = (rect_center_x - image_center_x) / image_center_x;
  double y_offset = (rect_center_y - image_center_y) / image_center_y;
  if (roboone_found)
    last_target_detected_direction_ = x_offset > 0 ? -1 : 1;
  double target_distance = 1000.0;
  if (range_available)
    target_distance = last_range_.range;
  else if (rect_area > 0.0001)
    target_distance = attack_rect_distance_ * sqrt(atk_min_rect_size_ / rect_area);
  int8_t is_left = (x_offset < 0) ? 1 : 0;

  // 攻撃判定
  std::string selected_attack_name = "";
  if (!force_walk_)
  {
    selected_attack_name = decideAttack(target_distance, range_available, is_left);
    if (selected_attack_name != "front" && selected_attack_name != "back" && selected_attack_name != "stop" &&
        selected_attack_name != "")
    {
      // 攻撃実行
      stopWalking();
      setCtrlModule("action_module");
      executeAction(selected_attack_name);
      attacked_time_ = ros::Time::now() + ros::Duration(action_duration_);
      attack_count_++;
      ROS_INFO("Attack triggered: %s. range_available=%d, target_distance=%f", selected_attack_name.c_str(),
               range_available, target_distance);
      return;
    }
    // TODO: When walking forward or backward, defer processing to the next step.
  }

  // 歩行処理
  double x_step = 0.0;
  double y_step = 0.0;
  double yaw_step = 0.0;
  if ((!roboone_found && (ros::Time::now() - robot_detected_time_).toSec() > 2.0) ||
      robot_detected_rect_.y > last_camera_info_.height * 0.8)
  {
    // ターゲットが見つからないときはその場旋回
    ROS_WARN_THROTTLE(3.0, "No roboone label found. Rotate in place.");
    yaw_step = last_target_detected_direction_ * fabs(yaw_step_max_);
    setCtrlModule("walking_module");
    setWalkSteps(0.0, 0.0, yaw_step);
    startWalking();
    return;
  }
  else if (robot_detected_rect_.y > last_camera_info_.height * 0.55)
  {
    // 相手ロボット転倒時
    if (rect_area > atk_min_rect_size_ * 0.5 || (range_available && last_range_.range > attack_rect_distance_ * 1.5))
    {
      // 相手が転倒していて至近距離の場合、後退
      ROS_INFO_THROTTLE(3.0, "The target is down and close. Move back.");
      double target_direction = (x_offset > 0) ? -1.0 : 1.0;  // 右なら-1、左なら1
      double target_angle_factor = -x_offset * 2.0;
      if (target_angle_factor > 1.0)
        target_angle_factor = 1.0;
      else if (target_angle_factor < -1.0)
        target_angle_factor = -1.0;
      yaw_step = target_angle_factor * fabs(yaw_step_max_);
      x_step = -fabs(x_backward_step_max_ * 0.5) * (1.0 - fabs(target_angle_factor));
      setWalkSteps(x_step, 0.0, yaw_step);
      startWalking();
    }
    else
    {
      // 離れた場所で相手が倒れている場合、その場旋回のみ
      ROS_INFO_THROTTLE(3.0, "Target is in fall state, rotating in place.");
      double target_angle_factor = -x_offset;
      yaw_step = target_angle_factor * fabs(yaw_step_max_);  // 最大15度の旋回
      x_step = 0.0;
      if (rect_area > atk_min_rect_size_ * 0.6)
      {
        x_step = -fabs(x_backward_step_max_);
      }
      if (!force_walk_ && walk_status_ == "start" && fabs(yaw_step) < (5.0 * M_PI / 180.0) && fabs(x_step) < 0.01 &&
          (ros::Time::now() - walk_start_time_).toSec() > 1.0)
      {
        setWalkSteps(0.0, 0.0, 0.0);
        stopWalking();
      }
      else if (!force_walk_ && walk_status_ == "stop" && fabs(yaw_step) < (10.0 * M_PI / 180.0) &&
               fabs(x_step) < 0.01 && (ros::Time::now() - walk_start_time_).toSec() > 1.0)
      {
        setWalkSteps(0.0, 0.0, 0.0);
        stopWalking();
      }
      else
      {
        setWalkSteps(x_step, 0.0, yaw_step);
        startWalking();
      }
    }
    return;
  }
  else
  {
    // 通常の歩行処理
    ROS_INFO_THROTTLE(3.0, "Target detected but too small(area: %f), delay: %f, rect area: %f", rect_area,
                      (ros::Time::now() - robot_detected_time_).toSec(), atk_min_rect_size_);
    setCtrlModule("walking_module");
    if ((ros::Time::now() - attacked_time_).toSec() < 2.0)
    {
      // 攻撃後の2秒間は後退のみ許可
      setWalkSteps(-fabs(x_backward_step_max_), 0.0, 0.0);
      startWalking();
    }
    else if ((ros::Time::now() - attacked_time_).toSec() < 8.0)
    {
      // 攻撃後の6秒間旋回のみ許可（前後左右移動は0）
      // 中央からのずれに基づいて旋回角を計算
      double target_direction = (x_offset > 0) ? -1.0 : 1.0;  // 右なら-1、左なら1
      double target_angle_factor = -x_offset;
      yaw_step = target_angle_factor * fabs(yaw_step_max_);  // 最大15度の旋回
      ROS_INFO_THROTTLE(3.0, "Calculated angle move for rotation only (radians): %f", yaw_step);
      // 前後左右の移動は0で、旋回のみ許可
      if (fabs(yaw_step) < (5.0 * M_PI / 180.0))
      {
        setWalkSteps(0.0, 0.0, 0.0);
        stopWalking();
      }
      else
      {
        setWalkSteps(0.0, 0.0, yaw_step);
        startWalking();
      }
    }
    else
    {
      // 相手の方に向かって歩行処理
      // 中央からのずれに基づいて旋回角を計算
      double target_direction = (x_offset > 0) ? -1.0 : 1.0;  // 右なら-1、左なら1
      double target_angle_factor = -x_offset;
      if (target_angle_factor > 0.5)
        target_angle_factor = 1.0;
      else if (target_angle_factor < -0.5)
        target_angle_factor = -1.0;
      yaw_step = target_angle_factor * fabs(yaw_step_max_);
      if (selected_attack_name == "back")
        x_step = -fabs(x_backward_step_max_) * (1.0 - fabs(target_angle_factor));
      else if (selected_attack_name == "front")
        x_step = fabs(x_forward_step_max_) * (1.0 - fabs(target_angle_factor));
      else
        x_step = 0.0;
      setWalkSteps(x_step, 0.0, yaw_step);
      startWalking();
    }
  }
}

// Free all joints by loading action module and playing motion ID -2
void RobooneAuto::freeAllJoints()
{
  setCtrlModule("action_module");  // Load the action module
  ROS_INFO("Executing action -2 to free all joints.");
  executeAction("disable");  // Play motion ID -2 to free the joints
}

// Enable all joints by loading action module and playing motion ID -1
void RobooneAuto::enableAllJoints()
{
  setCtrlModule("action_module");  // Load the action module
  ROS_INFO("Executing action -1 to enable all joints.");
  executeAction("enable");  // Play motion ID -1 to enable the joints
}

Eigen::Vector3d RobooneAuto::imuQuaternionToRollPitchYaw(const Eigen::Quaterniond& q)
{
  // ジンバルロックを防ぐため、ベクトルの相対的な変位を使用して角度を求める
  // 解はXYZオイラー角と異なる
  Eigen::Vector3d x_origin = Eigen::Vector3d::UnitX();  // Front
  Eigen::Vector3d y_origin = Eigen::Vector3d::UnitY();  // Left
  Eigen::Vector3d z_origin = Eigen::Vector3d::UnitZ();  // Up

  Eigen::Vector3d x_robot = q * x_origin;
  Eigen::Vector3d y_robot = q * y_origin;
  Eigen::Vector3d z_robot = q * z_origin;

  double roll = 0, pitch = 0, yaw = 0;
  // pitch: 原点のxy平面から見てロボットのx軸がどれだけ傾いているか
  pitch = std::atan2(-x_robot.z(), std::sqrt(x_robot.x() * x_robot.x() + x_robot.y() * x_robot.y()));
  if (z_robot.z() < 0.0)
    pitch = M_PI - pitch;  // 逆立ち補正
  // roll: 原点のxy平面から見てロボットのy軸がどれだけ傾いているか
  roll = std::atan2(y_robot.z(), std::sqrt(y_robot.x() * y_robot.x() + y_robot.y() * y_robot.y()));
  // yaw: 原点のz軸周りにロボットがどれだけ回転しているか
  if (fabs(x_robot.z()) < fabs(y_robot.z()))
  {
    // x軸の傾きがy軸の傾きより小さい場合、x軸を使用してyawを計算
    yaw = std::atan2(x_robot.y(), x_robot.x());
    if (z_robot.z() < 0.0)
      yaw = yaw + M_PI;  // 逆立ち補正
  }
  else
  {
    // y軸の傾きがx軸の傾きより小さい場合、y軸を使用してyawを計算
    yaw = std::atan2(y_robot.y(), y_robot.x()) - M_PI / 2.0;
  }

  // 最後に-M_PIから+M_PIに変換する
  roll = wrapToPi(roll);
  pitch = wrapToPi(pitch);
  yaw = wrapToPi(yaw);
  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond RobooneAuto::imuRollPitchYawToQuaternion(const Eigen::Vector3d& rpy)
{
  // rpy = (roll, pitch, yaw) as returned by imuQuaternionToRollPitchYaw
  const double roll = rpy.x();
  const double pitch = rpy.y();
  const double yaw = rpy.z();

  // 1) Build robot-frame axis directions consistent with the forward mapping.
  //    From the definitions used in imuQuaternionToRollPitchYaw:
  //    - pitch controls the tilt of the robot x-axis: x_robot.z = -sin(pitch),
  //      and its XY projection magnitude is cos(pitch) with heading = yaw.
  //    - roll  controls the tilt of the robot y-axis: y_robot.z =  sin(roll),
  //      and its XY projection magnitude is cos(roll) with heading = yaw + 90 deg.
  Eigen::Vector3d x_robot(std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), -std::sin(pitch));

  Eigen::Vector3d y_robot(std::cos(yaw + M_PI * 0.5) * std::cos(roll), std::sin(yaw + M_PI * 0.5) * std::cos(roll),
                          std::sin(roll));

  // 2) Orthonormalize to be safe (ensure a right-handed, orthonormal basis).
  //    z = x × y, then re-derive y = z × x to guarantee orthogonality.
  Eigen::Vector3d x = x_robot.normalized();
  Eigen::Vector3d y = y_robot.normalized();
  Eigen::Vector3d z = x.cross(y);
  double nz = z.norm();
  if (nz < 1e-12)
  {
    // In the unlikely event x and y became nearly colinear numerically,
    // pick an arbitrary perpendicular to x for stabilization.
    Eigen::Vector3d tmp = (std::abs(x.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    z = x.cross(tmp);
    nz = z.norm();
  }
  z.normalize();
  y = z.cross(x).normalized();

  // 3) Build rotation matrix whose columns are the images of the origin axes:
  //    q * [1,0,0] = x, q * [0,1,0] = y, q * [0,0,1] = z.
  Eigen::Matrix3d r_matrix;
  r_matrix.col(0) = x;
  r_matrix.col(1) = y;
  r_matrix.col(2) = z;

  // 4) Convert to quaternion.
  Eigen::Quaterniond q(r_matrix);
  return q.normalized();
}

double RobooneAuto::wrapToPi(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
    angle += 2.0 * M_PI;
  return angle - M_PI;
}

// Joyコールバック
void RobooneAuto::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // ROS_INFO("Joy data received: axes[0]: %f, axes[1]: %f, buttons[0]: %d, buttons[1]: %d", joy->axes[0], joy->axes[1],
  //          joy->buttons[0], joy->buttons[1]);
  last_joy_ = *joy;
  last_joy_.header.stamp = ros::Time::now();
}

// IMUコールバック
void RobooneAuto::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  // ROS_INFO("IMU data received: orientation (x: %f, y: %f, z: %f, w: %f)", imu->orientation.x, imu->orientation.y,
  //          imu->orientation.z, imu->orientation.w);
  last_imu_ = *imu;
  last_imu_.header.stamp = ros::Time::now();
}

void RobooneAuto::rangeCallback(const sensor_msgs::Range::ConstPtr& range)
{
  // ROS_INFO("Range data received: range: %f", range->range);
  if (range->range > 3.6)
    return;  // 異常値は無視
  last_range_ = *range;
  last_range_.header.stamp = ros::Time::now();
}

// カメラインフォコールバック
void RobooneAuto::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
  // ROS_INFO("Camera info received: height: %d, width: %d", camera_info->height, camera_info->width);
  last_camera_info_ = *camera_info;
}

// YOLOコールバック
void RobooneAuto::yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                               const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                               const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg)
{
  // ROS_INFO("YOLO data received: class size: %ld, label size: %ld, rects size: %ld", class_msg->label_names.size(),
  //          label_msg->labels.size(), rect_msg->rects.size());
  last_class_ = *class_msg;
  last_rects_ = *rect_msg;
  last_labels_ = *label_msg;
  last_rects_time_ = ros::Time::now();
}

std::string RobooneAuto::decideAttack(double target_distance, bool is_aimed, bool is_left)
{
  std::string action_name = "l_grip_front";
  if (force_aim_ && !is_aimed)
    return "back";

  // Get min and max attack distances
  double max_distance = 0.0;
  double min_distance = 1000.0;
  for (const auto& attack : attack_actions_)
  {
    // Skip if max count reached
    if (ultimate_mode_ && attack.ultimate_max_count >= 0 && attack.current_count >= attack.ultimate_max_count)
      continue;
    if (!ultimate_mode_ && attack.normal_max_count >= 0 && attack.current_count >= attack.normal_max_count)
      continue;
    // Skip if same as last attack
    if (isSameAttack(attack.name, last_attack_name_))
      continue;
    // Skip if left/right mismatch
    if (is_left && getActionDirection(attack.name) == -1)
      continue;
    else if (!is_left && getActionDirection(attack.name) == 1)
      continue;
    if (attack.force_aim && !is_aimed)
      continue;
    if (attack.min_distance < min_distance)
      min_distance = attack.min_distance;
    if (attack.max_distance > max_distance)
      max_distance = attack.max_distance;
  }
  if (target_distance < min_distance)
    return "back";
  if (target_distance > max_distance)
    return "front";

  // Get available attacks
  std::vector<std::string> available_attacks;
  std::vector<int8_t> attack_usage_count;
  std::string effective_attack = "l_grip_front";
  double effective_offset = 1000.0;
  for (const auto& attack : attack_actions_)
  {
    // Skip if max count reached
    if (ultimate_mode_ && attack.ultimate_max_count >= 0 && attack.current_count >= attack.ultimate_max_count)
      continue;
    if (!ultimate_mode_ && attack.normal_max_count >= 0 && attack.current_count >= attack.normal_max_count)
      continue;
    // Skip if same as last attack
    if (isSameAttack(attack.name, last_attack_name_))
      continue;
    // Skip if left/right mismatch
    if (is_left && getActionDirection(attack.name) == -1)
      continue;
    else if (!is_left && getActionDirection(attack.name) == 1)
      continue;
    if (attack.force_aim && !is_aimed)
      continue;
    // Check if within effective range
    double offset = fabs((attack.min_distance + attack.max_distance) / 2.0 - target_distance);
    if (offset < effective_offset)
    {
      effective_offset = offset;
      effective_attack = attack.name;
    }
    if (attack.min_distance < target_distance && target_distance < attack.max_distance)
    {
      available_attacks.push_back(attack.name);
      attack_usage_count.push_back(attack.current_count);
    }
  }
  // Execute the attack with the lowest number of uses from available_attacks
  if (!available_attacks.empty())
  {
    int min_count = 1000;
    int min_index = 0;
    for (size_t i = 0; i < available_attacks.size(); ++i)
    {
      if (attack_usage_count[i] < min_count)
      {
        min_count = attack_usage_count[i];
        min_index = i;
      }
    }
    action_name = available_attacks[min_index];
    last_attack_name_ = action_name;
    // Increment the usage count
    for (auto& attack : attack_actions_)
    {
      if (attack.name == action_name)
      {
        attack.current_count++;
        break;
      }
    }
  }
  else if (effective_offset < 0.1 && !force_aim_)
  {
    action_name = effective_attack;
    last_attack_name_ = action_name;
    // Increment the usage count
    for (auto& attack : attack_actions_)
    {
      if (attack.name == action_name)
      {
        attack.current_count++;
        break;
      }
    }
  }
  else
  {
    return "back";
  }
  if (is_aimed)
    force_aim_ = false;
  else
    force_aim_ = true;
  return action_name;
}

bool RobooneAuto::isSameAttack(const std::string& action_name, const std::string& last_action_name)
{
  if (action_name == last_action_name)
    return true;
  // Motions differing only in left and right are considered same
  const bool a_hand =
      (action_name.size() >= 2) && (action_name[1] == '_') && (action_name[0] == 'l' || action_name[0] == 'r');
  const bool b_hand = (last_action_name.size() >= 2) && (last_action_name[1] == '_') &&
                      (last_action_name[0] == 'l' || last_action_name[0] == 'r');
  if (a_hand && b_hand)
  {
    return action_name.substr(2) == last_action_name.substr(2);
  }
  return false;
}

int8_t RobooneAuto::getActionDirection(const std::string& action_name)
{
  if (action_name.size() >= 2 && action_name[1] == '_' && (action_name[0] == 'l' || action_name[0] == 'r'))
  {
    return (action_name[0] == 'l') ? 1 : -1;
  }
  return 0;
}
