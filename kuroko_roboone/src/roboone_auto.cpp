#include "roboone_auto.h"
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
  ROS_INFO("Starting Walking...");
  std_msgs::String msg;
  msg.data = "start";
  walking_command_pub_.publish(msg);
}

// Stop walking
void RobooneAuto::stopWalking()
{
  ROS_INFO("Stopping Walking...");
  std_msgs::String msg;
  msg.data = "stop";
  walking_command_pub_.publish(msg);
}

// Abort walking
void RobooneAuto::abortWalking()
{
  ROS_INFO("Aborting Walking...");
  std_msgs::String msg;
  msg.data = "abort";
  walking_command_pub_.publish(msg);
}

// Set walking parameters with specified initial values
void RobooneAuto::setWalkingParams(double x_step, double y_step, double yaw_step)
{
  kuroko_walking_module_msgs::WalkingParam params;
  double normalization_factor = std::abs(yaw_step / 0.26) + std::abs(x_step / 0.02) + std::abs(y_step / 0.015);

  if (normalization_factor > 1.0)
  {
    x_step /= normalization_factor;
    y_step /= normalization_factor;
    yaw_step /= normalization_factor;
  }

  // Initial values from the provided topic output
  params.init_x_offset = 0.018;
  params.init_y_offset = 0.06;
  params.init_z_offset = 0.05;
  params.init_roll_offset = 0.1396;
  params.init_pitch_offset = 0.0;
  params.init_yaw_offset = 0.0;
  params.init_hip_pitch_offset = 0;

  params.period_time = 0.43;
  params.dsp_ratio = 0.1;
  params.step_forward_back_ratio = 0.0;

  // Move amplitudes set dynamically
  params.x_step = x_step;
  params.y_step = y_step;
  params.yaw_step = yaw_step;

  // Fixed initial values for other fields
  params.foot_height = 0.08;
  params.y_swing_amplitude = 0.016;
  params.z_swing_amplitude = 0.004;
  params.roll_swing_amplitude = -0.05236;
  params.roll_swing_phase = 0.3491;
  params.hip_swing_up_amplitude = 0.03491;
  params.hip_swing_down_amplitude = -0.03491;

  params.shoulder_swing_amplitude = 0;
  params.chest_swing_amplitude = 0;

  params.balance_enable = true;
  params.balance_gyro_x_gain = 0.008;
  params.balance_gyro_y_gain = -0.004;
  params.balance_gyro_zx_gain = 0.004;
  params.balance_gyro_zy_gain = 0.008;
  params.balance_gyro_roll_gain = -0.01;
  params.balance_gyro_pitch_gain = 0.04;
  params.balance_euler_x_gain = 0.04;
  params.balance_euler_y_gain = -0.02;
  params.balance_euler_zx_gain = 0.004;
  params.balance_euler_zy_gain = 0.008;
  params.balance_euler_roll_gain = -0.01;
  params.balance_euler_pitch_gain = 0.04;

  // PID gains
  params.p_gain = 0;
  params.i_gain = 0;
  params.d_gain = 0;

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
  else if (current_state_ == "WALKING")
  {
    // 自律移動中にIMUの傾きが30度を超えた場合、歩行を停止して「歩行一時停止状態」に遷移
    double pitch = quaternionToPitch(last_imu_.orientation);
    double roll = quaternionToRoll(last_imu_.orientation);

    if (std::abs(pitch) > 0.52 || std::abs(roll) > 0.52)
    {
      transitionToPauseWalkingState();
    }
    else
    {
      // 攻撃処理の判定
      handleAttack();
    }
  }
  else if (current_state_ == "PAUSE_WALKING")
  {
    // 歩行一時停止状態でIMUの傾きが±15度以内に戻ったら再び自律移動状態へ遷移
    double pitch = quaternionToPitch(last_imu_.orientation);
    double roll = quaternionToRoll(last_imu_.orientation);

    if (std::abs(pitch) < 0.26 && std::abs(roll) < 0.26)
    {
      ROS_INFO("IMU stabilized. Returning to WALKING state.");
      startWalking();
      current_state_ = "WALKING";
    }
    else if ((ros::Time::now() - fall_detected_time_).toSec() > 1.5)
    {
      // 歩行一時停止状態が1.5秒以上続いた場合は「転倒状態」へ遷移
      transitionToFallState();
    }
    else
    {
      ROS_INFO("curent time: %f, last imu time: %f", ros::Time::now().toSec(), last_imu_time_.toSec());
    }
  }
  else if (current_state_ == "FALL")
  {
    // 転倒状態の処理
    ROS_INFO("Handling FALL state.");
    handleFall();
  }
}

// 初期姿勢への遷移
void RobooneAuto::transitionToInitPose()
{
  enableAllJoints();
  ros::Duration(0.1).sleep();
  ROS_INFO("Transitioning to INITIAL_POSE state.");
  setCtrlModule("initial_pose_module");
  setCtrlModule("action_module");
  setCtrlModule("walking_module");
  current_state_ = "INITIAL_POSE";
}

// 自律移動への遷移
void RobooneAuto::transitionToAutoMoveState()
{
  ROS_INFO("Transitioning to AUTO_MOVE state.");
  current_state_ = "WALKING";
  startWalking();
}

// 歩行一時停止状態への遷移
void RobooneAuto::transitionToPauseWalkingState()
{
  ROS_INFO("Transitioning to PAUSE_WALKING state due to excessive tilt.");
  abortWalking();  // 歩行を停止
  current_state_ = "PAUSE_WALKING";
  fall_detected_time_ = ros::Time::now();
}

// 転倒状態への遷移
void RobooneAuto::transitionToFallState()
{
  ROS_INFO("Transitioning to FALL state.");
  setCtrlModule("action_module");
  ros::Duration(0.1).sleep();
  current_state_ = "FALL";
}

// 転倒状態の処理
void RobooneAuto::handleFall()
{
  double pitch = quaternionToPitch(last_imu_.orientation);
  if (pitch > 0)
  {
    executeAction(2);  // 前起き上がりモーション
  }
  else
  {
    executeAction(3);  // 後起き上がりモーション
  }

  ros::Duration(1.0).sleep();
  // モーション再生完了後、一時停止状態に遷移、1秒待機
  setCtrlModule("walking_module");
  current_state_ = "PAUSE_WALKING";
  fall_detected_time_ = ros::Time::now() + ros::Duration(1.0);
  robot_detected_time_ = ros::Time(0);
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
  double pitch = quaternionToPitch(last_imu_.orientation);
  double roll = quaternionToRoll(last_imu_.orientation);
  if (std::abs(pitch) > 0.52 || std::abs(roll) > 0.52)
  {
    transitionToPauseWalkingState();
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

  if (roboone_found || (ros::Time::now() - robot_detected_time_).toSec() < 2.0)
  {
    double rect_area = (robot_detected_rect_.width * robot_detected_rect_.height) /
                       double(last_camera_info_.width * last_camera_info_.height);
    ROS_INFO("Largest roboone rect found with area: %f", rect_area);
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
      ROS_INFO("Target is too low, retreating.");
      double x_step = -0.03;
      setWalkingParams(x_step, 0.0, 0.0);
      startWalking();
    }
    else if (robot_detected_rect_.y > last_camera_info_.height * 0.2 &&
             robot_detected_rect_.width > robot_detected_rect_.height &&
             (ros::Time::now() - attacked_time_).toSec() > 3.0)
    {
      // 相手が倒れている場合、その場旋回のみ
      ROS_INFO("Target is in fall state, rotating in place.");
      double target_angle_factor = -x_offset;
      double yaw_step = target_angle_factor * (10.0 * M_PI / 180.0);  // 最大15度の旋回
      if (fabs(yaw_step) > (3.0 * M_PI / 180.0))
      {
        setWalkingParams(0.0, 0.0, yaw_step);
        startWalking();
      }
      else
      {
        setWalkingParams(0.0, 0.0, 0.0);
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
      setWalkingParams(0.0, 0.0, 0.0);
      stopWalking();
      ros::Duration(0.2).sleep();
      setCtrlModule("action_module");
      ros::Duration(0.2).sleep();
      executeAction(action_id);
      ros::Duration(1.0).sleep();
      setCtrlModule("walking_module");
      // 攻撃実行時刻を記録
      attacked_time_ = ros::Time::now();
    }
    else
    {
      ROS_INFO("Target detected but too small for attack or detected over 2 seconds ago (area: %f)", rect_area);
      if ((ros::Time::now() - attacked_time_).toSec() < 0.5)
      {
        // 攻撃後の1秒間は転倒復帰のみ許可
        stopWalking();
      }
      else if ((ros::Time::now() - attacked_time_).toSec() < 2.0)
      {
        // 攻撃後の2秒間は後退のみ許可
        setWalkingParams(-0.03, 0.0, 0.0);
        startWalking();
      }
      else if ((ros::Time::now() - attacked_time_).toSec() < 8.0)
      {
        // 攻撃後の6秒間旋回のみ許可（前後左右移動は0）
        // 中央からのずれに基づいて旋回角を計算
        double target_direction = (x_offset > 0) ? -1.0 : 1.0;  // 右なら-1、左なら1
        double target_angle_factor = -x_offset;
        double yaw_step = target_angle_factor * (10.0 * M_PI / 180.0);  // 最大15度の旋回
        ROS_INFO("Calculated angle move for rotation only (radians): %f", yaw_step);
        // 前後左右の移動は0で、旋回のみ許可
        if (fabs(yaw_step) > (3.0 * M_PI / 180.0))
        {
          setWalkingParams(0.0, 0.0, yaw_step);
          startWalking();
        }
        else
        {
          setWalkingParams(0.0, 0.0, 0.0);
          stopWalking();
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
        ROS_INFO("target x: %f, y: %f, width: %d, height: %d", rect_center_x, rect_center_y, largest_rect.width,
                 largest_rect.height);
        ROS_INFO("target x offset (pixels): %f, angle move (radians): %f", x_offset, yaw_step);
        double x_step = 0.04 * (1.0 - fabs(target_angle_factor));  // 最大0.04mの前進
        setWalkingParams(x_step, 0.0, yaw_step);
        ROS_INFO("Moving towards target with x_step: %f, yaw_step: %f", x_step, yaw_step);
        startWalking();
      }
    }
  }
  else
  {
    ROS_WARN("No roboone label found.");
    double yaw_step = last_target_detected_direction_ * (10.0 * M_PI / 180.0);  // 最大15度の旋回
    setWalkingParams(0.0, 0.0, yaw_step);
    ROS_INFO("Rotating in place with yaw_step (radians): %f", yaw_step);
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
double RobooneAuto::quaternionToPitch(const geometry_msgs::Quaternion& q)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return pitch;
}
double RobooneAuto::quaternionToRoll(const geometry_msgs::Quaternion& q)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return roll;
}

// Utility function to convert quaternion to yaw (radians)
double RobooneAuto::quaternionToYaw(const geometry_msgs::Quaternion& q)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

// Joyコールバック
void RobooneAuto::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_joy_ = *joy;
  last_joy_time_ = ros::Time::now();
}

// IMUコールバック
void RobooneAuto::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_imu_ = *imu;
  last_imu_time_ = ros::Time::now();
}

// カメラインフォコールバック
void RobooneAuto::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_camera_info_ = *camera_info;
}

// YOLOコールバック
void RobooneAuto::yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                               const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                               const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_class_ = *class_msg;
  last_rects_ = *rect_msg;
  last_labels_ = *label_msg;

  last_rects_time_ = ros::Time::now();
}
