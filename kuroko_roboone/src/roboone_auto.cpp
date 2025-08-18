#include "roboone_auto.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <sensor_msgs/CameraInfo.h>
#include "ros/console.h"
#include "ros/duration.h"

// コンストラクタ
RobooneAuto::RobooneAuto(ros::NodeHandle& nh)
  : atk_rects_size_(0.22)
  , class_sub_(nh, "/object_detection/output/class", 1)
  , label_sub_(nh, "/object_detection/output/labels", 1)
  , rect_sub_(nh, "/object_detection/output/rects", 1)
  , sync_(SyncPolicy(10), class_sub_, label_sub_, rect_sub_)
  , client_(nh.serviceClient<robotis_controller_msgs::SetModule>("/motion_control/set_present_ctrl_modules"))
{
  joy_sub_ = nh.subscribe("/gamepad/joy", 1, &RobooneAuto::joyCallback, this);
  imu_sub_ = nh.subscribe("/kuroko/sensors/imu/data", 1, &RobooneAuto::imuCallback, this);
  camera_info_sub_ = nh.subscribe("/camera/resized/camera_info", 1, &RobooneAuto::cameraInfoCallback, this);

  sync_.registerCallback(boost::bind(&RobooneAuto::yoloCallback, this, _1, _2, _3));

  walking_command_pub_ = nh.advertise<std_msgs::String>("/motion_control/walking/command", 1);
  walking_params_pub_ = nh.advertise<kuroko_walking_module_msgs::WalkingParam>("/motion_control/walking/set_params", 1);
  action_page_pub_ = nh.advertise<std_msgs::Int32>("/motion_control/action/animation_num", 1);

  current_state_ = "IDLE";
  running_ = true;
  // Initialize last_joy_ with default size
  last_joy_.axes.resize(2);
  last_joy_.buttons.resize(10);
  robot_detected_time_ = ros::Time(0);
  fall_detected_time_ = ros::Time(0);
  attacked_time_ = ros::Time(0);
  last_target_detected_direction_ = 1;
  last_attack_name_ = "";
  // Set walking params
  walk_param_.init_x_offset = 0.018;
  walk_param_.init_y_offset = 0.06;
  walk_param_.init_z_offset = 0.05;
  walk_param_.init_roll_offset = 0.1396;
  walk_param_.init_pitch_offset = 0.0;
  walk_param_.init_yaw_offset = 0.0;
  walk_param_.init_hip_pitch_offset = 0;
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
  jump_param_.init_z_offset = 0.13;
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
  jump_param_.balance_euler_pitch_gain = 0.1;

  stable_angle_threshold_ = 0.08;
  squat_angle_threshold_ = 0.16;
  jump_angle_threshold_ = 0.32;
  fall_angle_threshold_ = 0.64;

  stable_duration_ = 0.3;
  squat_duration_ = 1.0;
  jump_duration_ = 0.2;
  fall_duration_ = 2.0;
  walk_stop_duration_ = walk_param_.period_time * 2.0;
  min_walk_duration_ = walk_param_.period_time * 2.0;
  walk_start_time_ = ros::Time::now();
  force_walk_ = true;

  x_forward_step_max_ = 0.03;
  x_backward_step_max_ = -0.03;
  y_step_max_ = 0.02;
  yaw_step_max_ = 0.1;

  action_start_time_ = ros::Time(0);

  walk_status_ = "stop";
  current_module_ = "";

  action_id_map_["crouch_down"] = 0;
  action_id_map_["crouch_up"] = 1;
  action_id_map_["getup_front"] = 2;
  action_id_map_["getup_rear"] = 3;
  action_id_map_["l_grip_front"] = 4;
  action_id_map_["l_hook_front"] = 5;
  action_id_map_["l_punch_high"] = 6;
  action_id_map_["l_punch_low"] = 7;
  action_id_map_["r_grip_front"] = 8;
  action_id_map_["r_hook_front"] = 9;
  action_id_map_["r_punch_high"] = 10;
  action_id_map_["r_punch_low"] = 11;
  action_id_map_["disable"] = -2;
  action_id_map_["enable"] = -1;

  action_duration_map_["crouch_down"] = 0.1;
  action_duration_map_["crouch_up"] = 0.3;
  action_duration_map_["getup_front"] = 2.9;
  action_duration_map_["getup_rear"] = 5.1;
  action_duration_map_["l_grip_front"] = 2.69;
  action_duration_map_["l_hook_front"] = 1.4;
  action_duration_map_["l_punch_high"] = 1.81;
  action_duration_map_["l_punch_low"] = 1.68;
  action_duration_map_["r_grip_front"] = 2.69;
  action_duration_map_["r_hook_front"] = 1.4;
  action_duration_map_["r_punch_high"] = 1.81;
  action_duration_map_["r_punch_low"] = 1.68;
  action_duration_map_["disable"] = 0.1;
  action_duration_map_["enable"] = 0.1;

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

void RobooneAuto::setSquatSteps(double x_step, double y_step, double yaw_step)
{
  kuroko_walking_module_msgs::WalkingParam params = squat_param_;
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

void RobooneAuto::setJumpSteps(double x_step, double y_step, double yaw_step)
{
  kuroko_walking_module_msgs::WalkingParam params = jump_param_;
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
  bool over_fall_angle = fabs(imu_rpy[1]) > fall_angle_threshold_;
  bool over_squat_angle = fabs(imu_rpy[1]) > squat_angle_threshold_;
  bool within_stable_angle = fabs(imu_rpy[1]) < stable_angle_threshold_;

  // ボタン検知
  if (current_state_ == "INITIAL_POSE" && last_joy_.buttons[2])
  {
    ROS_INFO("Transitioning to WALKING state from INITIAL_POSE.");
    action_name_ = "";
    transitionToAutoMoveState();
    return;
  }
  else if (current_state_ != "IDLE" && last_joy_.buttons[1])
  {
    ROS_INFO("Transitioning to IDLE state from current state: %s", current_state_.c_str());
    action_name_ = "";
    transitionToIdleState();
    return;
  }
  else if (current_state_ != "INITIAL_POSE" && last_joy_.buttons[0])
  {
    ROS_INFO("Transitioning to INITIAL_POSE state from current state: %s", current_state_.c_str());
    action_name_ = "";
    transitionToInitPose();
    return;
  }

  // Sleep
  if (current_state_ == "IDLE" || current_state_ == "INITIAL_POSE")
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
      setWalkSteps(0, 0, 0);
      setCtrlModule("walking_module");
      action_name_ = "";
      last_imu_.orientation.x = 0.0;
      last_imu_.orientation.y = 0.0;
      last_imu_.orientation.z = 0.0;
      last_imu_.orientation.w = 1.0;
      over_fall_angle = false;
      over_squat_angle = false;
      within_stable_angle = true;
      walk_start_time_ = ros::Time::now();
      attacked_time_ = ros::Time(0);
      force_walk_ = true;
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

  // 転倒検知
  if (over_fall_angle)
  {
    transitionToFallState();
  }
  else if (over_squat_angle)
  {
    transitionToSquatState();
  }

  // 状態遷移
  if (current_state_ == "WALKING")
  {
    handleAttack();
  }
  else if (current_state_ == "PAUSE_WALKING")
  {
    if (over_fall_angle)
    {
      transitionToFallState();
    }
    else if (over_squat_angle)
    {
      transitionToSquatState();
    }
    else if (within_stable_angle)
    {
      ROS_INFO("IMU stabilized. Returning to WALKING state.");
      transitionToAutoMoveState();
    }
    else
    {
      ROS_INFO("curent time: %f, last imu time: %f", ros::Time::now().toSec(), last_imu_time_.toSec());
    }
  }
  else if (current_state_ == "SQUAT")
  {
    if (within_stable_angle)
    {
      ROS_INFO("Stable angle restored. Transitioning to PAUSE_WALKING.");
      transitionToPauseWalkingState();
    }
    else if (over_fall_angle)
    {
      ROS_INFO("Over fall angle detected in SQUAT state. Transitioning to FALL.");
      transitionToFallState();
    }
  }
  else if (current_state_ == "FALL")
  {
    if (within_stable_angle)
    {
      ROS_INFO("Stable angle restored. Transitioning to PAUSE_WALKING.");
      transitionToPauseWalkingState();
      fall_detected_time_ = ros::Time::now() + ros::Duration(1.0);
    }
    else if (!over_squat_angle)
    {
      transitionToSquatState();
    }
    else if ((ros::Time::now() - fall_detected_time_).toSec() > 1.5)
    {
      ROS_INFO("Handling FALL state.");
      handleFall();
      transitionToPauseWalkingState();
    }
  }
}

// 初期姿勢への遷移
void RobooneAuto::transitionToInitPose()
{
  if (current_state_ == "INITIAL_POSE")
    return;
  current_state_ = "INITIAL_POSE";
  enableAllJoints();
  ROS_INFO("Transitioning to INITIAL_POSE state.");
  setCtrlModule("initial_pose_module");
  setCtrlModule("action_module");
  setCtrlModule("walking_module");
}

// 自律移動への遷移
void RobooneAuto::transitionToAutoMoveState()
{
  if (current_state_ == "WALKING")
    return;
  current_state_ = "WALKING";
  ROS_INFO("Transitioning to WALKING state.");
  setCtrlModule("walking_module");
  setWalkSteps(0, 0, 0);
  startWalking();
  robot_detected_time_ = ros::Time(0);
  last_rects_.rects.clear();
  attacked_time_ = ros::Time(0);
  walk_start_time_ = ros::Time::now();
  force_walk_ = true;
}

// 歩行一時停止状態への遷移
void RobooneAuto::transitionToPauseWalkingState()
{
  if (current_state_ == "PAUSE_WALKING")
    return;
  current_state_ = "PAUSE_WALKING";
  ROS_INFO("Transitioning to PAUSE_WALKING state due to excessive tilt.");
  setCtrlModule("walking_module");
  setWalkSteps(0, 0, 0);
  stopWalking();
}

// こらえ状態への遷移
void RobooneAuto::transitionToSquatState()
{
  if (current_state_ == "SQUAT")
    return;
  current_state_ = "SQUAT";
  ROS_INFO("Transitioning to SQUAT state.");
  setCtrlModule("walking_module");
  setSquatSteps(0, 0, 0);
  ros::Duration(0.04).sleep();
  abortWalking();
  attacked_time_ = ros::Time(0);
}

// 転倒状態への遷移
void RobooneAuto::transitionToFallState()
{
  if (current_state_ == "FALL")
    return;
  if (current_state_ != "SQUAT")
  {
    transitionToSquatState();
  }
  fall_detected_time_ = ros::Time::now();
  current_state_ = "FALL";
  ROS_INFO("Transitioning to FALL state.");
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
  current_state_ = "PAUSE_WALKING";
  robot_detected_time_ = ros::Time(0);
  last_rects_.rects.clear();
}

// 脱力状態への遷移
void RobooneAuto::transitionToIdleState()
{
  freeAllJoints();
  current_state_ = "IDLE";
}

// 攻撃処理
void RobooneAuto::handleAttack()
{
  if (last_camera_info_.height == 0 || last_camera_info_.width == 0)
  {
    ROS_WARN("Camera info is missing.");
    return;
  }
  bool roboone_found = false;
  int roboone_count = 0;
  jsk_recognition_msgs::Rect largest_rect;

  // robooneラベルを持つrectを探し、その中で一番大きいものを見つける
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
          robot_detected_time_ = ros::Time::now();  // robooneが見つかった時刻を記録
          largest_rect = last_rects_.rects[i];
          robot_detected_rect_ = largest_rect;
        }
      }
    }
  }
  last_rects_.rects.clear();

  if (roboone_found || (ros::Time::now() - robot_detected_time_).toSec() < 2.0)
  {
    double rect_area = (robot_detected_rect_.width * robot_detected_rect_.height) /
                       double(last_camera_info_.width * last_camera_info_.height);
    // ROS_INFO("Largest roboone rect found with area: %f", rect_area);
    double rect_center_x = robot_detected_rect_.x + robot_detected_rect_.width / 2.0;
    double rect_center_y = robot_detected_rect_.y + robot_detected_rect_.height / 2.0;
    double image_center_x = last_camera_info_.width / 2.0;
    double image_center_y = last_camera_info_.height / 2.0;
    double x_offset = (rect_center_x - image_center_x) / image_center_x;
    double y_offset = (rect_center_y - image_center_y) / image_center_y;
    last_target_detected_direction_ = x_offset > 0 ? -1 : 1;
    if (ros::Time::now() - walk_start_time_ > ros::Duration(min_walk_duration_) &&
        ros::Time::now() - attacked_time_ > ros::Duration(min_walk_duration_) && force_walk_)
    {
      force_walk_ = false;
    }

    if ((robot_detected_rect_.y > last_camera_info_.height * 0.6 && rect_area > atk_rects_size_ * 0.3 &&
         rect_area < atk_rects_size_) ||
        roboone_count > 2)
    {
      // 相手ロボットが画面のした半分にしか入っていたいときはなにかおかしいので後退
      ROS_INFO_THROTTLE(1.0, "Target is too low, retreating.");
      double target_direction = (x_offset > 0) ? -1.0 : 1.0;  // 右なら-1、左なら1
      double target_angle_factor = -x_offset;
      if (target_angle_factor > 0.5)
        target_angle_factor = 1.0;
      else if (target_angle_factor < -0.5)
        target_angle_factor = -1.0;
      double yaw_step = target_angle_factor * fabs(yaw_step_max_);
      double x_step = -fabs(x_backward_step_max_) * (1.0 - fabs(target_angle_factor));
      setWalkSteps(x_step, 0.0, yaw_step);
      startWalking();
    }
    else if (robot_detected_rect_.y > last_camera_info_.height * 0.4 &&
             robot_detected_rect_.width > robot_detected_rect_.height * 1.8)
    {
      // 相手が倒れている場合、その場旋回のみ
      ROS_INFO_THROTTLE(1.0, "Target is in fall state, rotating in place.");
      double target_angle_factor = -x_offset;
      double yaw_step = target_angle_factor * fabs(yaw_step_max_);  // 最大15度の旋回
      double x_step = 0.0;
      if (rect_area > atk_rects_size_ * 0.6)
      {
        x_step = -fabs(x_backward_step_max_);
      }
      if (fabs(yaw_step) > (3.0 * M_PI / 180.0) || fabs(x_step) > 0.01)
      {
        setWalkSteps(x_step, 0.0, yaw_step);
        startWalking();
      }
      else
      {
        setWalkSteps(0.0, 0.0, 0.0);
        stopWalking();
      }
    }
    else if ((rect_area > atk_rects_size_ ||
              robot_detected_rect_.width / double(last_camera_info_.width) > 2.0 * sqrt(atk_rects_size_) ||
              robot_detected_rect_.height / double(last_camera_info_.height) > 2.0 * sqrt(atk_rects_size_)))
    {
      if (force_walk_)
      {
        double target_direction = (x_offset > 0) ? -1.0 : 1.0;
        double target_angle_factor = -x_offset;
        if (target_angle_factor > 0.5)
          target_angle_factor = 1.0;
        else if (target_angle_factor < -0.5)
          target_angle_factor = -1.0;
        double yaw_step = target_angle_factor * fabs(yaw_step_max_);
        double x_step = -fabs(x_backward_step_max_) * (1.0 - fabs(target_angle_factor));
        setWalkSteps(x_step, 0.0, yaw_step);
        startWalking();
      }
      else
      {
        ROS_INFO("Attack triggered! Rect area is larger than threshold and detected within 5 seconds.");
        std::string action_name = "l_grip_front";
        if (x_offset > 0)
        {
          ROS_INFO("Target is on the right side, executing action based on vertical position.");
          if (robot_detected_rect_.width * 1.2 < robot_detected_rect_.height)
            action_name = "r_hook_front";
          else if (robot_detected_rect_.y < last_camera_info_.height * 0.4)
            action_name = "r_grip_front";
          else if (robot_detected_rect_.y < last_camera_info_.height * 0.6)
            action_name = "r_punch_low";
          else
            action_name = "r_punch_high";
        }
        else
        {
          ROS_INFO("Target is on the left side, executing action based on vertical position.");
          if (robot_detected_rect_.width * 1.2 < robot_detected_rect_.height)
            action_name = "l_hook_front";
          else if (robot_detected_rect_.y < last_camera_info_.height * 0.4)
            action_name = "l_grip_front";
          else if (robot_detected_rect_.y < last_camera_info_.height * 0.6)
            action_name = "l_punch_low";
          else
            action_name = "l_punch_high";
        }

        if (last_attack_name_ == action_name)
        {
          if (action_name == "r_grip_front")
            action_name = "r_punch_low";
          else if (action_name == "r_punch_low")
            action_name = "r_hook_front";
          else if (action_name == "r_hook_front")
            action_name = "r_punch_high";
          else if (action_name == "r_punch_high")
            action_name = "r_grip_front";

          else if (action_name == "l_grip_front")
            action_name = "l_punch_low";
          else if (action_name == "l_punch_low")
            action_name = "l_hook_front";
          else if (action_name == "l_hook_front")
            action_name = "l_punch_high";
          else if (action_name == "l_punch_high")
            action_name = "l_grip_front";
        }
        last_attack_name_ = action_name;
        // 歩行を停止し攻撃を開始
        stopWalking();
        setCtrlModule("action_module");
        executeAction(action_name);
        attacked_time_ = ros::Time::now() + ros::Duration(action_duration_);
      }
    }
    else
    {
      ROS_INFO_THROTTLE(1.0, "Target detected but too small(area: %f), delay: %f, rect area: %f", rect_area,
                        (ros::Time::now() - robot_detected_time_).toSec(), atk_rects_size_);
      setCtrlModule("walking_module");
      if ((ros::Time::now() - attacked_time_).toSec() < 0.2)
      {
        // 攻撃後の1.0秒間は転倒復帰のみ許可
        abortWalking();
      }
      else if ((ros::Time::now() - attacked_time_).toSec() < 2.2)
      {
        // 攻撃後の3秒間は後退のみ許可
        setWalkSteps(-fabs(x_backward_step_max_), 0.0, 0.0);
        startWalking();
      }
      else if ((ros::Time::now() - attacked_time_).toSec() < 8.0)
      {
        // 攻撃後の6秒間旋回のみ許可（前後左右移動は0）
        // 中央からのずれに基づいて旋回角を計算
        double target_direction = (x_offset > 0) ? -1.0 : 1.0;  // 右なら-1、左なら1
        double target_angle_factor = -x_offset;
        double yaw_step = target_angle_factor * fabs(yaw_step_max_);  // 最大15度の旋回
        ROS_INFO_THROTTLE(1.0, "Calculated angle move for rotation only (radians): %f", yaw_step);
        // 前後左右の移動は0で、旋回のみ許可
        if (fabs(yaw_step) < (3.0 * M_PI / 180.0) && !force_walk_)
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
        double yaw_step = target_angle_factor * fabs(yaw_step_max_);                    // 最大10度の旋回
        double x_step = fabs(x_forward_step_max_) * (1.0 - fabs(target_angle_factor));  // 最大0.04mの前進
        setWalkSteps(x_step, 0.0, yaw_step);
        startWalking();
      }
    }
  }
  else
  {
    setCtrlModule("walking_module");
    ROS_WARN_THROTTLE(1.0, "No roboone label found.");
    double yaw_step = last_target_detected_direction_ * fabs(yaw_step_max_);  // 最大15度の旋回
    setWalkSteps(0.0, 0.0, yaw_step);
    ROS_INFO_THROTTLE(1.0, "Rotating in place with yaw_step (radians): %f", yaw_step);
    startWalking();
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
  last_joy_time_ = ros::Time::now();
}

// IMUコールバック
void RobooneAuto::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  // ROS_INFO("IMU data received: orientation (x: %f, y: %f, z: %f, w: %f)", imu->orientation.x, imu->orientation.y,
  //          imu->orientation.z, imu->orientation.w);
  last_imu_ = *imu;
  last_imu_time_ = ros::Time::now();
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
