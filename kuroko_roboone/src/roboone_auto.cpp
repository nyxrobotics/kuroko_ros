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
  walking_params_pub_ = nh.advertise<op3_walking_module_msgs::WalkingParam>("/motion_control/walking/set_params", 1);
  action_page_pub_ = nh.advertise<std_msgs::Int32>("/motion_control/action/page_num", 1);

  current_state_ = "IDLE";
  running_ = true;
  state_thread_ = std::thread(&RobooneAuto::stateThread, this);
  // Initialize last_joy_ with default size
  last_joy_.axes.resize(2);
  last_joy_.buttons.resize(10);
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
void RobooneAuto::setWalkingParams(double x_move, double y_move, double angle_move)
{
  op3_walking_module_msgs::WalkingParam params;
  double normalization_factor = sqrt(pow(angle_move / 0.26, 2) + pow(x_move / 0.02, 2) + pow(y_move / 0.015, 2));

  if (normalization_factor > 1.0)
  {
    x_move /= normalization_factor;
    y_move /= normalization_factor;
    angle_move /= normalization_factor;
  }

  // Initial values from the provided topic output
  params.init_x_offset = 0.019999999552965164;
  params.init_y_offset = 0.03999999910593033;
  params.init_z_offset = 0.06499999761581421;
  params.init_roll_offset = -0.0872664600610733;
  params.init_pitch_offset = 0.0;
  params.init_yaw_offset = 0.0;
  params.period_time = 0.4699999988079071;
  params.dsp_ratio = 0.4000000059604645;
  params.step_fb_ratio = 0.0;

  // Move amplitudes set dynamically
  params.x_move_amplitude = x_move;
  params.y_move_amplitude = y_move;
  params.angle_move_amplitude = angle_move;

  // Fixed initial values for other fields
  params.z_move_amplitude = 0.14000000059604645;
  params.move_aim_on = false;
  params.balance_enable = false;
  params.balance_hip_roll_gain = 0.3499999940395355;
  params.balance_knee_gain = 0.30000001192092896;
  params.balance_ankle_roll_gain = 0.699999988079071;
  params.balance_ankle_pitch_gain = 0.8999999761581421;
  params.y_swap_amplitude = 0.02800000086426735;
  params.z_swap_amplitude = 0.006000000052154064;
  params.arm_swing_gain = 1.5;
  params.pelvis_offset = 0.008726646192371845;
  params.hip_pitch_offset = 0.0872664600610733;

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

  if (current_state_ == "INIT_POSE" && last_joy_.buttons[2])
  {
    transitionToAutoMoveState();
  }
  else if (current_state_ != "IDLE" && last_joy_.buttons[1])
  {
    transitionToIdleState();
  }
  else if (current_state_ != "INIT_POSE" && last_joy_.buttons[0])
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
  ROS_INFO("Transitioning to INIT_POSE state.");
  setCtrlModule("initial_pose_module");
  setCtrlModule("action_module");
  setCtrlModule("walking_module");
  current_state_ = "INIT_POSE";
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
  current_state_ = "FALL";
}

// 転倒状態の処理
void RobooneAuto::handleFall()
{
  setCtrlModule("action_module");
  double pitch = quaternionToPitch(last_imu_.orientation);

  if (pitch > 0)
  {
    executeAction(0);  // 前起き上がりモーション
  }
  else
  {
    executeAction(1);  // 後起き上がりモーション
  }
  // Sleep 7 seconds
  ros::Duration(7.0).sleep();

  // モーション再生完了後、walking_moduleをロードして再び自律移動状態に遷移
  setCtrlModule("walking_module");
  current_state_ = "PAUSE_WALKING";
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
  if (last_rects_.rects.empty() || last_camera_info_.height == 0 || last_camera_info_.width == 0)
  {
    ROS_WARN("No recognized objects or camera info is missing.");
    return;
  }

  auto it = std::max_element(last_rects_.rects.begin(), last_rects_.rects.end(),
                             [](const jsk_recognition_msgs::Rect& a, const jsk_recognition_msgs::Rect& b) {
                               return a.width * a.height < b.width * b.height;
                             });

  if (it != last_rects_.rects.end())
  {
    double rect_area = (it->width * it->height) / double(last_camera_info_.width * last_camera_info_.height);
    ROS_INFO("Largest rect found with area: %f", rect_area);

    if (rect_area > atk_rects_size_)
    {
      ROS_INFO("Attack triggered! Rect area is larger than threshold.");

      // 歩行を停止し攻撃を開始
      stopWalking();
      executeAction(2);

      // 攻撃後の処理（相手に向かって移動）
      double rect_center_x = it->x + it->width / 2.0;
      double image_center_x = last_camera_info_.width / 2.0;
      double x_offset = (rect_center_x - image_center_x) / image_center_x;

      // カメラ中央からの相対位置に基づいて旋回量を計算 (ラジアンに変換)
      double angle_move = -x_offset * (15.0 * M_PI / 180.0);  // 最大15度の旋回
      ROS_INFO("Calculated angle move (radians): %f", angle_move);

      // 前進は0.02m、左右移動はなし
      setWalkingParams(0.02, 0.0, angle_move);  // 0.02m前進しながら旋回

      ROS_INFO("Moving towards target with x_move: 0.02, angle_move (radians): %f", angle_move);
      startWalking();
    }
    else
    {
      ROS_INFO("Target detected but too small for attack (area: %f)", rect_area);

      // 相手の方に向かって歩行処理
      double rect_center_x = it->x + it->width / 2.0;
      double image_center_x = last_camera_info_.width / 2.0;
      double x_offset = (rect_center_x - image_center_x) / image_center_x;

      // 中央からのずれに基づいて旋回角を計算
      double angle_move = -x_offset * (15.0 * M_PI / 180.0);  // 最大15度の旋回
      ROS_INFO("Calculated angle move (radians): %f", angle_move);

      // 0.02m前進しつつ旋回
      setWalkingParams(0.02, 0.0, angle_move);
      ROS_INFO("Moving towards target with x_move: 0.02, angle_move (radians): %f", angle_move);
      startWalking();
    }
  }
  else
  {
    ROS_WARN("No valid rects found for movement.");
  }
}

// Free all joints and switch to "none" control module
void RobooneAuto::freeAllJoints()
{
  setCtrlModule("none");
  std_msgs::String msg;
  msg.data = "free";
  walking_command_pub_.publish(msg);
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
  last_rects_ = *rect_msg;
  last_rects_time_ = ros::Time::now();
}
