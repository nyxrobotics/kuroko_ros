#include "roboone_auto.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <sensor_msgs/CameraInfo.h>

// コンストラクタ
RobooneAuto::RobooneAuto(ros::NodeHandle& nh)
  : atk_rects_size_(0.2)
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
  state_thread_ = std::thread(&RobooneAuto::stateThread, this);
  // Initialize last_joy_ with default size
  last_joy_.axes.resize(2);
  last_joy_.buttons.resize(10);
  robot_detected_time_ = ros::Time(0);
  fall_detected_time_ = ros::Time(0);
  attacked_time_ = ros::Time(0);
  last_target_detected_direction_ = 1;
  last_attack_id_ = 0;
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

  hold_param_ = walk_param_;
  hold_param_.init_x_offset = 0;
  hold_param_.init_z_offset = 0.12;
  hold_param_.balance_gyro_x_gain = 0.004;
  hold_param_.balance_gyro_y_gain = 0.0;
  hold_param_.balance_gyro_zx_gain = 0.004;
  hold_param_.balance_gyro_zy_gain = 0.0;
  hold_param_.balance_gyro_roll_gain = 0.0;
  hold_param_.balance_gyro_pitch_gain = 0.02;
  hold_param_.balance_euler_x_gain = 0.06;
  hold_param_.balance_euler_y_gain = 0.0;
  hold_param_.balance_euler_zx_gain = 0.01;
  hold_param_.balance_euler_zy_gain = 0.0;
  hold_param_.balance_euler_roll_gain = 0.0;
  hold_param_.balance_euler_pitch_gain = 0.8;

  jump_param_ = hold_param_;
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

  fall_angle_threshold_ = 0.26;
  hold_angle_threshold_ = 0.16;
  stable_angle_threshold_ = 0.08;

  x_forward_step_max_ = 0.03;
  x_backward_step_max_ = -0.03;
  y_step_max_ = 0.02;
  yaw_step_max_ = 0.2;

  action_start_time_ = ros::Time(0);

  walk_status_ = "stop";
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

  if (client_.call(srv))
  {
    ROS_INFO("Successfully set control module to %s", module_name.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service to set control module to %s", module_name.c_str());
    return false;
  }
}

// Start walking
void RobooneAuto::startWalking()
{
  if (walk_status_ == "start")
    return;
  walk_status_ = "start";
  ROS_INFO("Starting Walking...");
  std_msgs::String msg;
  msg.data = "start";
  walking_command_pub_.publish(msg);
}

// Stop walking
void RobooneAuto::stopWalking()
{
  if (walk_status_ == "stop" || walk_status_ == "abort")
    return;
  walk_status_ = "stop";
  ROS_INFO("Stopping Walking...");
  std_msgs::String msg;
  msg.data = "stop";
  walking_command_pub_.publish(msg);
}

// Abort walking
void RobooneAuto::abortWalking()
{
  if (walk_status_ == "abort")
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

void RobooneAuto::setHoldSteps(double x_step, double y_step, double yaw_step)
{
  kuroko_walking_module_msgs::WalkingParam params = hold_param_;
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

// Execute an action by ID
void RobooneAuto::executeAction(int action_id)
{
  ROS_INFO_STREAM("Executing action with ID: " << action_id);
  std_msgs::Int32 msg;
  msg.data = action_id;
  action_page_pub_.publish(msg);
}

// 状態管理スレッド
void RobooneAuto::stateThread()
{
  ros::Rate rate(10);
  while (running_)
  {
    manageState();
    rate.sleep();
  }
}

// 状態管理処理
void RobooneAuto::manageState()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  Eigen::Quaterniond imy_orientation(last_imu_.orientation.x, last_imu_.orientation.y, last_imu_.orientation.z,
                                     last_imu_.orientation.w);
  Eigen::Vector3d imu_rpy = quaterionToYpr(imy_orientation);
  bool over_fall_angle = fabs(imu_rpy[1]) > fall_angle_threshold_;
  bool over_hold_angle = fabs(imu_rpy[1]) > hold_angle_threshold_;
  bool within_stable_angle = fabs(imu_rpy[1]) < stable_angle_threshold_;

  // ボタン検知
  if (current_state_ == "INITIAL_POSE" && last_joy_.buttons[2])
  {
    transitionToAutoMoveState();
  }
  else if (current_state_ != "IDLE" && last_joy_.buttons[1])
  {
    transitionToIdleState();
  }
  else if (current_state_ != "INITIAL_POSE" && last_joy_.buttons[0])
  {
    transitionToInitPose();
  }

  // 転倒検知
  bool is_atk = (ros::Time::now() - attacked_time_).toSec() < 0.4;
  if (!is_atk && current_state_ != "IDLE" && current_state_ != "INITIAL_POSE")
  {
    if (over_fall_angle)
    {
      transitionToFallState();
    }
    else if (over_hold_angle)
    {
      transitionToHoldState();
    }
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
    else if (over_hold_angle)
    {
      transitionToHoldState();
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
  else if (current_state_ == "HOLD")
  {
    if (within_stable_angle)
    {
      ROS_INFO("Stable angle restored. Transitioning to PAUSE_WALKING.");
      setWalkSteps(0, 0, 0);
      ros::Duration(0.1).sleep();
      abortWalking();
      ros::Duration(0.1).sleep();
      transitionToPauseWalkingState();
    }
    else if (over_fall_angle)
    {
      ROS_INFO("Over fall angle detected in HOLD state. Transitioning to FALL.");
      transitionToFallState();
    }
  }
  else if (current_state_ == "FALL")
  {
    if (within_stable_angle)
    {
      ROS_INFO("Stable angle restored. Transitioning to PAUSE_WALKING.");
      setWalkSteps(0, 0, 0);
      ros::Duration(0.1).sleep();
      abortWalking();
      ros::Duration(0.1).sleep();
      transitionToPauseWalkingState();
      fall_detected_time_ = ros::Time::now() + ros::Duration(1.0);
    }
    else if (!over_hold_angle)
    {
      transitionToHoldState();
    }
    else if ((ros::Time::now() - fall_detected_time_).toSec() > 1.5)
    {
      ROS_INFO("Handling FALL state.");
      setWalkSteps(0, 0, 0);
      ros::Duration(0.1).sleep();
      abortWalking();
      ros::Duration(0.2).sleep();
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
  ros::Duration(0.1).sleep();
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
  startWalking();
  ros::Duration(0.1).sleep();
}

// 歩行一時停止状態への遷移
void RobooneAuto::transitionToPauseWalkingState()
{
  if (current_state_ == "PAUSE_WALKING")
    return;
  current_state_ = "PAUSE_WALKING";
  ROS_INFO("Transitioning to PAUSE_WALKING state due to excessive tilt.");
  setWalkSteps(0, 0, 0);
  ros::Duration(0.1).sleep();
  stopWalking();  // 歩行を停止
  ros::Duration(0.1).sleep();
}

// こらえ状態への遷移
void RobooneAuto::transitionToHoldState()
{
  if (ros::Time::now() - action_start_time_ < ros::Duration(5.0))
  {
    setWalkSteps(0, 0, 0);
    ros::Duration(0.1).sleep();
    abortWalking();  // 歩行を停止
    ros::Duration(0.1).sleep();
    current_state_ = "PAUSE_WALKING";
    return;
  }
  if (current_state_ == "HOLD")
    return;
  current_state_ = "HOLD";
  ROS_INFO("Transitioning to HOLD state.");
  setHoldSteps(0, 0, 0);
  ros::Duration(0.1).sleep();
  abortWalking();
  ros::Duration(0.1).sleep();
  attacked_time_ = ros::Time::now();      // しゃがんだ後は歩行が必須
  action_start_time_ = ros::Time::now();  // しゃがんだ後は歩行が必須
}

// 転倒状態への遷移
void RobooneAuto::transitionToFallState()
{
  if (current_state_ == "FALL")
    return;
  fall_detected_time_ = ros::Time::now();
  if (current_state_ != "HOLD")
  {
    setHoldSteps(0, 0, 0);
    ros::Duration(0.1).sleep();
    abortWalking();
  }
  current_state_ = "FALL";
  ROS_INFO("Transitioning to FALL state.");
}

// 転倒状態の処理
void RobooneAuto::handleFall()
{
  Eigen::Quaterniond imy_orientation(last_imu_.orientation.x, last_imu_.orientation.y, last_imu_.orientation.z,
                                     last_imu_.orientation.w);
  Eigen::Vector2d imu_rp = quaternionToRollPitch(imy_orientation);
  double pitch_angle = imu_rp[1];
  ROS_INFO("Handling FALL state with IMU RPY: roll=%f, pitch=%f", imu_rp[0], imu_rp[1]);
  setWalkSteps(0, 0, 0);
  abortWalking();
  setCtrlModule("action_module");
  ros::Duration(0.1).sleep();
  if (imu_rp[1] > 0)
  {
    executeAction(2);  // 前起き上がりモーション
    ros::Duration(4.0).sleep();
  }
  else
  {
    executeAction(3);  // 後起き上がりモーション
    ros::Duration(6.0).sleep();
  }
  // モーション再生完了後、一時停止状態に遷移、1秒待機
  setCtrlModule("walking_module");
  current_state_ = "PAUSE_WALKING";
  fall_detected_time_ = ros::Time::now() + ros::Duration(1.0);
  robot_detected_time_ = ros::Time(0);
  last_rects_.rects.clear();
  action_start_time_ = ros::Time::now();
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
    if ((robot_detected_rect_.y > last_camera_info_.height * 0.5 && rect_area > atk_rects_size_ * 0.3 &&
         rect_area < atk_rects_size_ && (ros::Time::now() - attacked_time_).toSec() > 3.0) ||
        roboone_count > 2)
    {
      // 相手ロボットが画面のした半分にしか入っていたいときはなにかおかしいので後退
      ROS_INFO_THROTTLE(1.0, "Target is too low, retreating.");
      double x_step = -0.04;
      setWalkSteps(x_step, 0.0, 0.0);
      startWalking();
    }
    else if (robot_detected_rect_.y > last_camera_info_.height * 0.2 &&
             robot_detected_rect_.width > robot_detected_rect_.height * 1.2 &&
             (ros::Time::now() - attacked_time_).toSec() > 3.0)
    {
      // 相手が倒れている場合、その場旋回のみ
      ROS_INFO_THROTTLE(1.0, "Target is in fall state, rotating in place.");
      double target_angle_factor = -x_offset;
      double yaw_step = target_angle_factor * (10.0 * M_PI / 180.0);  // 最大15度の旋回
      if (fabs(yaw_step) > (3.0 * M_PI / 180.0))
      {
        setWalkSteps(0.0, 0.0, yaw_step);
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
              robot_detected_rect_.height / double(last_camera_info_.height) > 2.0 * sqrt(atk_rects_size_)) &&
             (ros::Time::now() - attacked_time_).toSec() > 5.0)
    {
      ROS_INFO("Attack triggered! Rect area is larger than threshold and detected within 5 seconds.");
      // 歩行を停止し攻撃を開始
      stopWalking();
      int action_id = 4;
      if (x_offset > 0)
      {
        ROS_INFO("Target is on the right side, executing action based on vertical position.");
        if (robot_detected_rect_.width * 1.2 < robot_detected_rect_.height)
          action_id = 9;  // r_hook_front
        else if (robot_detected_rect_.y < last_camera_info_.height * 0.4)
          action_id = 8;  // r_grip_front
        else if (robot_detected_rect_.y < last_camera_info_.height * 0.6)
          action_id = 11;  // r_punch_low
        else
          action_id = 10;  // r_punch_high
      }
      else
      {
        ROS_INFO("Target is on the left side, executing action based on vertical position.");
        if (robot_detected_rect_.width * 1.2 < robot_detected_rect_.height)
          action_id = 5;  // l_hook_front
        else if (robot_detected_rect_.y < last_camera_info_.height * 0.4)
          action_id = 4;  // l_grip_front
        else if (robot_detected_rect_.y < last_camera_info_.height * 0.6)
          action_id = 7;  // l_punch_low
        else
          action_id = 6;  // l_punch_high
      }

      if (last_attack_id_ == action_id)
      {
        if (action_id == 8)
          action_id = 11;
        else if (action_id == 11)
          action_id = 9;
        else if (action_id == 9)
          action_id = 10;
        else if (action_id == 10)
          action_id = 8;

        else if (action_id == 4)
          action_id = 7;
        else if (action_id == 7)
          action_id = 5;
        else if (action_id == 5)
          action_id = 6;
        else if (action_id == 6)
          action_id = 4;
      }
      last_attack_id_ = action_id;
      // 攻撃実行時刻を記録
      attacked_time_ = ros::Time::now();
      setCtrlModule("action_module");
      ros::Duration(0.1).sleep();
      executeAction(action_id);
      ros::Duration(0.1).sleep();
      setCtrlModule("walking_module");
      ros::Duration(0.1).sleep();
    }
    else
    {
      ROS_INFO_THROTTLE(1.0, "Target detected but too small(area: %f), delay: %f, rect area: %f", rect_area,
                        (ros::Time::now() - robot_detected_time_).toSec(), atk_rects_size_);
      if ((ros::Time::now() - attacked_time_).toSec() < 1.0)
      {
        // 攻撃後の1.0秒間は転倒復帰のみ許可
        stopWalking();
      }
      else if ((ros::Time::now() - attacked_time_).toSec() < 4.0)
      {
        // 攻撃後の3秒間は後退のみ許可
        setWalkSteps(-0.04, 0.0, 0.0);
        startWalking();
      }
      else if ((ros::Time::now() - attacked_time_).toSec() < 10.0)
      {
        // 攻撃後の6秒間旋回のみ許可（前後左右移動は0）
        // 中央からのずれに基づいて旋回角を計算
        double target_direction = (x_offset > 0) ? -1.0 : 1.0;  // 右なら-1、左なら1
        double target_angle_factor = -x_offset;
        double yaw_step = target_angle_factor * (10.0 * M_PI / 180.0);  // 最大15度の旋回
        ROS_INFO_THROTTLE(1.0, "Calculated angle move for rotation only (radians): %f", yaw_step);
        // 前後左右の移動は0で、旋回のみ許可
        if (fabs(yaw_step) < (3.0 * M_PI / 180.0) && (ros::Time::now() - attacked_time_).toSec() > 6.0)
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
        double yaw_step = target_angle_factor * (10.0 * M_PI / 180.0);  // 最大10度の旋回
        // ROS_INFO("target x: %f, y: %f, width: %d, height: %d", rect_center_x, rect_center_y, largest_rect.width,
        //          largest_rect.height);
        // ROS_INFO("target x offset (pixels): %f, angle move (radians): %f", x_offset, yaw_step);
        double x_step = 0.04 * (1.0 - fabs(target_angle_factor));  // 最大0.04mの前進
        setWalkSteps(x_step, 0.0, yaw_step);
        // ROS_INFO("Moving towards target with x_step: %f, yaw_step: %f", x_step, yaw_step);
        startWalking();
      }
    }
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "No roboone label found.");
    double yaw_step = last_target_detected_direction_ * (10.0 * M_PI / 180.0);  // 最大15度の旋回
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
  executeAction(-2);  // Play motion ID -2 to free the joints
}

// Enable all joints by loading action module and playing motion ID -1
void RobooneAuto::enableAllJoints()
{
  setCtrlModule("action_module");  // Load the action module
  ROS_INFO("Executing action -1 to enable all joints.");
  executeAction(-1);  // Play motion ID -1 to enable the joints
}

// Utility function to convert quaternion to pitch (radians)
Eigen::Vector3d RobooneAuto::quaterionToRpy(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d angles;  // roll pitch yaw
  double x = q.x(), y = q.y(), z = q.z(), w = q.w();

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  angles[2] = std::atan2(siny_cosp, cosy_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
    angles[1] = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles[1] = std::asin(sinp);

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  angles[0] = std::atan2(sinr_cosp, cosr_cosp);

  return angles;
}

Eigen::Vector3d RobooneAuto::quaterionToYpr(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d angles;  // yaw pitch roll
  double x = q.x(), y = q.y(), z = q.z(), w = q.w();

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  angles[2] = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
    angles[1] = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles[1] = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  angles[0] = std::atan2(siny_cosp, cosy_cosp);
  return angles;
}

Eigen::Vector2d RobooneAuto::quaternionToRollPitch(const Eigen::Quaterniond& q)
{
  // 各基準ベクトルを「回転前の姿勢」と見なす
  Eigen::Vector3d ref_x = Eigen::Vector3d::UnitX();  // 前方
  Eigen::Vector3d ref_y = Eigen::Vector3d::UnitY();  // 左
  Eigen::Vector3d ref_z = Eigen::Vector3d::UnitZ();  // 上

  // 回転後の座標系から見た、これら基準ベクトルの向き
  // → 回転前ベクトルを逆回転する（＝q.inverse()で回転後座標系に投影）
  Eigen::Vector3d x_local = q.inverse() * ref_x;
  Eigen::Vector3d y_local = q.inverse() * ref_y;
  Eigen::Vector3d z_local = q.inverse() * ref_z;

  // roll（横方向の傾き）: z_local が y-z 平面でどれだけ傾いているか
  double roll = -std::atan2(z_local.y(), z_local.z());

  // pitch（前後方向の傾き）: z_local が x-z 平面でどれだけ傾いているか
  double pitch = -std::atan2(-z_local.x(), std::sqrt(z_local.y() * z_local.y() + z_local.z() * z_local.z()));

  // yaw（方位）: x_local が x-y 平面でどれだけ回転しているか
  double yaw = -std::atan2(x_local.y(), x_local.x());

  return Eigen::Vector2d(roll, pitch);
}

// Joyコールバック
void RobooneAuto::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // ROS_INFO("Joy data received: axes[0]: %f, axes[1]: %f, buttons[0]: %d, buttons[1]: %d", joy->axes[0], joy->axes[1],
  //          joy->buttons[0], joy->buttons[1]);
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_joy_ = *joy;
  last_joy_time_ = ros::Time::now();
}

// IMUコールバック
void RobooneAuto::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  // ROS_INFO("IMU data received: orientation (x: %f, y: %f, z: %f, w: %f)", imu->orientation.x, imu->orientation.y,
  //          imu->orientation.z, imu->orientation.w);
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_imu_ = *imu;
  last_imu_time_ = ros::Time::now();
}

// カメラインフォコールバック
void RobooneAuto::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
  // ROS_INFO("Camera info received: height: %d, width: %d", camera_info->height, camera_info->width);
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_camera_info_ = *camera_info;
}

// YOLOコールバック
void RobooneAuto::yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                               const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                               const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg)
{
  // ROS_INFO("YOLO data received: class size: %ld, label size: %ld, rects size: %ld", class_msg->label_names.size(),
  //          label_msg->labels.size(), rect_msg->rects.size());
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_class_ = *class_msg;
  last_rects_ = *rect_msg;
  last_labels_ = *label_msg;
  last_rects_time_ = ros::Time::now();
}
