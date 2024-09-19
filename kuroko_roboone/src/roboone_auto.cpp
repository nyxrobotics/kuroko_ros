#include "roboone_auto.h"

RobooneAuto::RobooneAuto(ros::NodeHandle& nh)
  : class_sub_(nh, "/object_detection/output/class", 1)
  , label_sub_(nh, "/object_detection/output/labels", 1)
  , rect_sub_(nh, "/object_detection/output/rects", 1)
  , sync_(SyncPolicy(10), class_sub_, label_sub_, rect_sub_)
{
  joy_sub_ = nh.subscribe("/gamepad/joy", 1, &RobooneAuto::joyCallback, this);
  imu_sub_ = nh.subscribe("/kuroko/sensors/imu/data", 1, &RobooneAuto::imuCallback, this);
  sync_.registerCallback(boost::bind(&RobooneAuto::yoloCallback, this, _1, _2, _3));

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

RobooneAuto::~RobooneAuto()
{
  // Stop the state management thread
  running_ = false;
  if (state_thread_.joinable())
  {
    state_thread_.join();
  }
}

void RobooneAuto::init()
{
  // Initialize parameters and modules
  ROS_INFO("Modules initialized.");
}

void RobooneAuto::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_joy_.axes.resize(joy->axes.size());
  last_joy_.buttons.resize(joy->buttons.size());
  last_joy_ = *joy;
  last_joy_time_ = ros::Time::now();
}

void RobooneAuto::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_imu_ = *imu;
  last_imu_time_ = ros::Time::now();
}

void RobooneAuto::yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                               const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                               const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_rects_ = *rect_msg;
  last_rects_time_ = ros::Time::now();
}

void RobooneAuto::stateThread()
{
  ros::Rate rate(10);  // 10 Hz loop
  while (running_)
  {
    manageState();
    rate.sleep();
  }
}

void RobooneAuto::manageState()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Handle button presses and state transitions
  if (current_state_ != INIT_POSE && last_joy_.buttons[0])
  {  // Circle button pressed, not in INIT_POSE
    transitionToInitPose();
  }
  else if (current_state_ != IDLE && last_joy_.buttons[1])
  {  // X button pressed, not in IDLE
    ROS_INFO("Transitioning to IDLE (Relaxed) state.");
    current_state_ = IDLE;
    // Logic to free all joints
  }
  else if (current_state_ == INIT_POSE && last_joy_.buttons[2])
  {  // Triangle button pressed in INIT_POSE
    transitionToWalking();
  }

  // Transition to fall state if in Autonomous Walking State and IMU indicates fall
  if (current_state_ == WALKING)
  {
    double pitch = quaternionToPitch(last_imu_.orientation);
    if (std::abs(pitch) > 30.0)
    {
      if ((ros::Time::now() - last_imu_time_).toSec() >= 2.0)
      {  // Check if the pitch has been exceeded for 2 seconds
        transitionToFallState();
      }
    }
  }
}

void RobooneAuto::transitionToInitPose()
{
  ROS_INFO("Transitioning to INIT_POSE state.");
  current_state_ = INIT_POSE;

  // Load initialpose, action, and walking modules
  // (Add the necessary module loading logic here)
}

void RobooneAuto::transitionToWalking()
{
  ROS_INFO("Transitioning to WALKING (Autonomous Movement) state.");
  current_state_ = WALKING;

  // Walking logic here
}

void RobooneAuto::transitionToFallState()
{
  ROS_INFO("Transitioning to FALL state.");
  current_state_ = FALL;

  // Handle fall recovery
}

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
