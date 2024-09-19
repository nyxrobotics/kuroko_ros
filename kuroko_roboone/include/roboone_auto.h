#ifndef ROBOONE_AUTO_H
#define ROBOONE_AUTO_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <jsk_recognition_msgs/ClassificationResult.h>
#include <jsk_recognition_msgs/LabelArray.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <mutex>

class RobooneAuto
{
public:
  RobooneAuto(ros::NodeHandle& nh);
  ~RobooneAuto();  // Destructor to join thread

  void init();

private:
  // Callback functions
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  void yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                    const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                    const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg);

  // State management function to run in a separate thread
  void stateThread();

  // State transitions
  void manageState();
  void transitionToInitPose();
  void transitionToWalking();
  void transitionToFallState();

  // Utility functions
  double quaternionToYaw(const geometry_msgs::Quaternion& q);
  double quaternionToPitch(const geometry_msgs::Quaternion& q);

  // ROS subscribers and publishers
  ros::Subscriber joy_sub_;
  ros::Subscriber imu_sub_;
  typedef message_filters::sync_policies::ExactTime<jsk_recognition_msgs::ClassificationResult,
                                                    jsk_recognition_msgs::LabelArray, jsk_recognition_msgs::RectArray>
      SyncPolicy;
  message_filters::Subscriber<jsk_recognition_msgs::ClassificationResult> class_sub_;
  message_filters::Subscriber<jsk_recognition_msgs::LabelArray> label_sub_;
  message_filters::Subscriber<jsk_recognition_msgs::RectArray> rect_sub_;
  message_filters::Synchronizer<SyncPolicy> sync_;

  // State and data management variables
  sensor_msgs::Joy last_joy_;
  sensor_msgs::Imu last_imu_;
  jsk_recognition_msgs::RectArray last_rects_;
  ros::Time last_joy_time_;
  ros::Time last_imu_time_;
  ros::Time last_rects_time_;

  // State management thread and mutex
  std::thread state_thread_;
  std::mutex state_mutex_;
  bool running_;  // Flag to control the state management thread

  enum State
  {
    IDLE,
    INIT_POSE,
    WALKING,
    FALL
  } current_state_,
      previous_state_, next_state_;
};

#endif  // ROBOONE_AUTO_H
