#ifndef ROBOONE_AUTO_H_
#define ROBOONE_AUTO_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <jsk_recognition_msgs/ClassificationResult.h>
#include <jsk_recognition_msgs/LabelArray.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <op3_walking_module_msgs/WalkingParam.h>
#include <tf/transform_datatypes.h>
#include <thread>
#include <mutex>
#include <robotis_controller_msgs/SetModule.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class RobooneAuto
{
public:
  RobooneAuto(ros::NodeHandle& nh);
  ~RobooneAuto();

  // Public functions for controlling walking and actions
  void startWalking();
  void stopWalking();
  void setWalkingParams(double x_move, double y_move, double angle_move);
  void executeAction(int action_id);

private:
  // Internal state management functions
  void stateThread();
  void manageState();
  void transitionToInitPose();
  void transitionToWalking();
  void transitionToFallState();
  void handleAttack();
  bool setCtrlModule(const std::string& module_name);
  void freeAllJoints();  // 関数宣言を追加

  // Utility functions for handling sensor data
  double quaternionToYaw(const geometry_msgs::Quaternion& q);
  double quaternionToPitch(const geometry_msgs::Quaternion& q);

  // Callback functions for subscribed topics
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  void yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                    const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                    const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg);

  // ROS publishers and subscribers
  ros::Subscriber joy_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher walking_command_pub_;
  ros::Publisher walking_params_pub_;
  ros::Publisher action_page_pub_;

  // ROS service client for setting control module
  ros::ServiceClient client_;

  // Message filters and synchronization policy for YOLO output
  message_filters::Subscriber<jsk_recognition_msgs::ClassificationResult> class_sub_;
  message_filters::Subscriber<jsk_recognition_msgs::LabelArray> label_sub_;
  message_filters::Subscriber<jsk_recognition_msgs::RectArray> rect_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
      jsk_recognition_msgs::ClassificationResult, jsk_recognition_msgs::LabelArray, jsk_recognition_msgs::RectArray>
      SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

  // State variables
  enum State
  {
    IDLE,
    INIT_POSE,
    WALKING,
    FALL
  };

  State current_state_;
  State previous_state_;
  State next_state_;

  // Sensor data and synchronization variables
  sensor_msgs::Joy last_joy_;
  sensor_msgs::Imu last_imu_;
  jsk_recognition_msgs::RectArray last_rects_;
  ros::Time last_joy_time_;
  ros::Time last_imu_time_;
  ros::Time last_rects_time_;

  std::vector<std::string> joint_names_;
  std::mutex state_mutex_;
  std::thread state_thread_;
  bool running_;
  double atk_rects_size_;  // Attack threshold for object detection size
};

#endif  // ROBOONE_AUTO_H_
