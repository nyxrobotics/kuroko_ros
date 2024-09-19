#include "roboone_auto.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <op3_walking_module_msgs/WalkingParam.h>
#include <ros/ros.h>
#include <robotis_controller_msgs/SetModule.h>

// Constructor
RobooneAuto::RobooneAuto(ros::NodeHandle& nh)
  : atk_rects_size_(0.5)
  , class_sub_(nh, "/object_detection/output/class", 1)
  , label_sub_(nh, "/object_detection/output/labels", 1)
  , rect_sub_(nh, "/object_detection/output/rects", 1)
  , sync_(SyncPolicy(10), class_sub_, label_sub_, rect_sub_)
  , client_(nh.serviceClient<robotis_controller_msgs::SetModule>(
        "/motion_control/set_present_ctrl_modules"))  // クライアントをメンバ変数として初期化
{
  joy_sub_ = nh.subscribe("/gamepad/joy", 1, &RobooneAuto::joyCallback, this);
  imu_sub_ = nh.subscribe("/kuroko/sensors/imu/data", 1, &RobooneAuto::imuCallback, this);
  sync_.registerCallback(boost::bind(&RobooneAuto::yoloCallback, this, _1, _2, _3));

  // Publishers for walking and motion control
  walking_command_pub_ = nh.advertise<std_msgs::String>("/motion_control/walking/command", 1);
  walking_params_pub_ = nh.advertise<op3_walking_module_msgs::WalkingParam>("/motion_control/walking/set_params", 1);
  action_page_pub_ = nh.advertise<std_msgs::Int32>("/motion_control/action/page_num", 1);

  // Initialize joint names from joint_names.yaml
  joint_names_ = { "chest",         "shoulder_r_pitch", "shoulder_r_roll", "elbow_r_front",
                   "elbow_r_rear",  "shoulder_l_pitch", "shoulder_l_roll", "elbow_l_front",
                   "elbow_l_rear",  "hip_r_roll",       "hip_r_pitch",     "thigh_r_active",
                   "shin_r_active", "ankle_r_roll",     "ankle_r_yaw",     "hip_l_roll",
                   "hip_l_pitch",   "thigh_l_active",   "shin_l_active",   "ankle_l_roll",
                   "ankle_l_yaw" };

  // Initialize state variables
  current_state_ = IDLE;
  previous_state_ = IDLE;
  next_state_ = IDLE;

  // Initialize last_joy_ with default size
  last_joy_.axes.resize(2);
  last_joy_.buttons.resize(10);

  // Initialize thread control flag
  running_ = true;

  // Start the state management thread
  state_thread_ = std::thread(&RobooneAuto::stateThread, this);

  ROS_INFO("Modules initialized.");
}

// Destructor
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

// State management thread
void RobooneAuto::stateThread()
{
  ros::Rate rate(10);  // 10 Hz loop
  while (running_)
  {
    manageState();
    rate.sleep();
  }
}

// Manage state transitions
void RobooneAuto::manageState()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (current_state_ != INIT_POSE && last_joy_.buttons[0])
  {
    transitionToInitPose();
  }
  else if (current_state_ != IDLE && last_joy_.buttons[1])
  {
    current_state_ = IDLE;
    freeAllJoints();
  }
  else if (current_state_ == INIT_POSE && last_joy_.buttons[2])
  {
    transitionToWalking();
  }

  if (current_state_ == WALKING)
  {
    double pitch = quaternionToPitch(last_imu_.orientation);
    if (std::abs(pitch) > 30.0 && (ros::Time::now() - last_imu_time_).toSec() >= 2.0)
    {
      transitionToFallState();
    }
    handleAttack();
  }
}

// Transition to initial pose by loading "initial_pose_module", "action_module", and "walking_module"
void RobooneAuto::transitionToInitPose()
{
  ROS_INFO("Transitioning to INIT_POSE state.");

  // Load initial_pose_module
  if (!setCtrlModule("initial_pose_module"))
  {
    ROS_ERROR("Failed to set control module to initial_pose_module.");
    return;
  }
  // Load action_module
  if (!setCtrlModule("action_module"))
  {
    ROS_ERROR("Failed to set control module to action_module.");
    return;
  }
  current_state_ = INIT_POSE;
}

// Transition to walking by loading "walking_module"
void RobooneAuto::transitionToWalking()
{
  ROS_INFO("Transitioning to WALKING (Autonomous Movement) state.");

  // Load walking_module
  if (setCtrlModule("walking_module"))
  {
    startWalking();
    current_state_ = WALKING;
  }
  else
  {
    ROS_ERROR("Failed to set control module to walking_module.");
  }
}

// Transition to fall state by loading "action_module", executing fall recovery, and then loading "walking_module"
void RobooneAuto::transitionToFallState()
{
  ROS_INFO("Transitioning to FALL state.");

  // Load action_module
  if (!setCtrlModule("action_module"))
  {
    ROS_ERROR("Failed to set control module to action_module.");
    return;
  }

  // Execute fall recovery motion based on pitch direction
  if (quaternionToPitch(last_imu_.orientation) > 0)
  {
    executeAction(0);  // Execute front fall recovery
  }
  else
  {
    executeAction(1);  // Execute back fall recovery
  }

  // Load initial_pose_module after fall recovery
  if (!setCtrlModule("initial_pose_module"))
  {
    ROS_ERROR("Failed to set control module to initial_pose_module.");
    return;
  }

  current_state_ = FALL;
}

// Handle attack logic
void RobooneAuto::handleAttack()
{
  if (last_rects_.rects.empty())
  {
    return;  // No recognized objects
  }

  auto it = std::max_element(last_rects_.rects.begin(), last_rects_.rects.end(),
                             [](const jsk_recognition_msgs::Rect& a, const jsk_recognition_msgs::Rect& b) {
                               return a.width * a.height < b.width * b.height;
                             });

  if (it != last_rects_.rects.end())
  {
    double area = (it->width * it->height) / (960 * 600);
    if (area > atk_rects_size_)
    {
      stopWalking();
      executeAction(2);  // Execute attack action
      startWalking();
    }
  }
}

// Free all joints by loading "none" control module
void RobooneAuto::freeAllJoints()
{
  ROS_INFO("Freeing all joints and switching to none module.");
  if (setCtrlModule("none"))
  {
    std_msgs::String msg;
    msg.data = "free";
    walking_command_pub_.publish(msg);
  }
  else
  {
    ROS_ERROR("Failed to set control module to none.");
  }
}

// Utility functions to convert quaternion to yaw/pitch
double RobooneAuto::quaternionToYaw(const geometry_msgs::Quaternion& q)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

double RobooneAuto::quaternionToPitch(const geometry_msgs::Quaternion& q)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return pitch;
}

// Joy callback function
void RobooneAuto::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_joy_ = *joy;
  last_joy_time_ = ros::Time::now();
}

// IMU callback function
void RobooneAuto::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_imu_ = *imu;
  last_imu_time_ = ros::Time::now();
}

// YOLO callback function
void RobooneAuto::yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                               const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                               const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_rects_ = *rect_msg;
  last_rects_time_ = ros::Time::now();
}
