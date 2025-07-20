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
// Set walking parameters with specified initial values
void RobooneAuto::setWalkingParams(double x_step, double y_move, double angle_move)
{
  kuroko_walking_module_msgs::WalkingParam params;
  double normalization_factor = std::abs(angle_move / 0.26) + std::abs(x_step / 0.02) + std::abs(y_move / 0.015);

  if (normalization_factor > 1.0)
  {
    x_step /= normalization_factor;
    y_move /= normalization_factor;
    angle_move /= normalization_factor;
  }

  // Initial values from the provided topic output
  params.init_x_offset = 0.0;
  params.init_y_offset = 0.06;
  params.init_z_offset = 0.08;
  params.init_roll_offset = -0.0349;
  params.init_pitch_offset = 0.0;
  params.init_yaw_offset = 0.0;
  params.period_time = 0.47;
  params.dsp_ratio = 0.35;
  params.step_fb_ratio = 0.0;

  // Move amplitudes set dynamically
  params.x_step = x_step;
  params.y_step = y_move;
  params.yaw_step = angle_move;

  // Fixed initial values for other fields
  params.z_step = 0.12;
  params.move_aim_on = false;
  params.balance_enable = false;
  params.balance_hip_roll_gain = 0.3499999940395355;
  params.balance_knee_gain = 0.30000001192092896;
  params.balance_ankle_roll_gain = 0.699999988079071;
  params.balance_ankle_pitch_gain = 0.8999999761581421;
  params.y_swing_amplitude = 0.016;
  params.z_swing_amplitude = 0.003;
  params.shoulder_swing_amplitude = 1.5;
  params.hip_swing_amplitude = 0.008726646192371845;
  params.init_hip_pitch_offset = 0;

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
    else if ((ros::Time::now() - fall_detected_time_).toSec() > 3.0)
    {
      // 歩行一時停止状態が3秒以上続いた場合は「転倒状態」へ遷移
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
  stopWalking();  // 歩行を停止
  current_state_ = "PAUSE_WALKING";
  fall_detected_time_ = ros::Time::now();
}

// 転倒状態への遷移
void RobooneAuto::transitionToFallState()
{
  ROS_INFO("Transitioning to FALL state.");
  setCtrlModule("action_module");
  ros::Duration(1.0).sleep();
  current_state_ = "FALL";
}

// 転倒状態の処理
void RobooneAuto::handleFall()
{
  double pitch = quaternionToPitch(last_imu_.orientation);
  if (pitch > 0)
  {
    executeAction(0);  // 前起き上がりモーション
  }
  else
  {
    executeAction(1);  // 後起き上がりモーション
  }

  ros::Duration(1.0).sleep();
  // モーション再生完了後、一時停止状態に遷移、3秒待機
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

  bool roboone_found = false;
  jsk_recognition_msgs::Rect largest_rect;

  // robooneラベルを持つrectを探し、その中で一番大きいものを見つける
  for (size_t i = 0; i < last_rects_.rects.size(); ++i)
  {
    if (last_class_.label_names[i] == "roboone")
    {
      roboone_found = true;
      robot_detected_time_ = ros::Time::now();  // robooneが見つかった時刻を記録
      if (largest_rect.width * largest_rect.height < last_rects_.rects[i].width * last_rects_.rects[i].height)
      {
        largest_rect = last_rects_.rects[i];
      }
    }
  }

  if ((ros::Time::now() - robot_detected_time_).toSec() < 2.0)
  {
    double rect_area =
        (largest_rect.width * largest_rect.height) / double(last_camera_info_.width * last_camera_info_.height);
    ROS_INFO("Largest roboone rect found with area: %f", rect_area);
    double rect_center_x = largest_rect.x + largest_rect.width / 2.0;
    double image_center_x = last_camera_info_.width / 2.0;
    double x_offset = (rect_center_x - image_center_x) / image_center_x;
    last_target_detected_direction_ = x_offset > 0 ? -1 : 1;
    if (rect_area > atk_rects_size_ && (ros::Time::now() - attacked_time_).toSec() > 5.0)
    {
      // 相手が倒れている場合、attacked_time_を9秒前にリセット
      if (largest_rect.y > last_camera_info_.height * 0.2)
      {
        ROS_INFO("Target is in fall state, resetting attacked_time_ to 9 seconds ago.");
        attacked_time_ = ros::Time::now() - ros::Duration(9.0);
      }
      else
      {
        ROS_INFO("Attack triggered! Rect area is larger than threshold and detected within 5 seconds.");
        // 歩行を停止し攻撃を開始
        stopWalking();
        executeAction(2);
        attacked_time_ = ros::Time::now();  // 攻撃実行時刻を記録
      }
    }
    else
    {
      ROS_INFO("Target detected but too small for attack or detected over 2 seconds ago (area: %f)", rect_area);

      // 攻撃後の1秒間は歩行停止
      if ((ros::Time::now() - attacked_time_).toSec() < 1.0)
      {
        stopWalking();
        robot_detected_time_ = ros::Time(0);
        setWalkingParams(0.0, 0.0, 0.0);
      }
      // 攻撃後の2秒間は後退のみ許可
      else if ((ros::Time::now() - attacked_time_).toSec() < 3.0)
      {
        setWalkingParams(-0.02, 0.0, 0.0);
        startWalking();
      }
      // 攻撃後の6秒間旋回のみ許可（前後左右移動は0）
      else if ((ros::Time::now() - attacked_time_).toSec() < 8.0)
      {
        // 中央からのずれに基づいて旋回角を計算
        double angle_move = -x_offset * (10.0 * M_PI / 180.0);  // 最大15度の旋回
        ROS_INFO("Calculated angle move for rotation only (radians): %f", angle_move);

        // 前後左右の移動は0で、旋回のみ許可
        setWalkingParams(0.0, 0.0, angle_move);
        ROS_INFO("Rotating in place with angle_move (radians): %f", angle_move);
        startWalking();
      }
      else
      {
        // 相手の方に向かって歩行処理
        // 中央からのずれに基づいて旋回角を計算
        double angle_move = -x_offset * (10.0 * M_PI / 180.0);  // 最大15度の旋回
        ROS_INFO("Calculated angle move (radians): %f", angle_move);

        // 0.04m前進しつつ旋回
        setWalkingParams(0.04, 0.0, angle_move);
        ROS_INFO("Moving towards target with x_step: 0.02, angle_move (radians): %f", angle_move);
        startWalking();
      }
    }
  }
  else
  {
    ROS_WARN("No roboone label found.");
    double angle_move = last_target_detected_direction_ * (10.0 * M_PI / 180.0);  // 最大15度の旋回
    setWalkingParams(0.0, 0.0, angle_move);
    ROS_INFO("Rotating in place with angle_move (radians): %f", angle_move);
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
