#ifndef ROBOONE_AUTO_H_
#define ROBOONE_AUTO_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <op3_walking_module_msgs/WalkingParam.h>
#include <jsk_recognition_msgs/ClassificationResult.h>
#include <jsk_recognition_msgs/LabelArray.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <robotis_controller_msgs/SetModule.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <mutex>
#include <thread>
#include <tf/transform_datatypes.h>
#include <vector>

class RobooneAuto
{
public:
  RobooneAuto(ros::NodeHandle& nh);
  ~RobooneAuto();

  void startWalking();
  void stopWalking();
  void setWalkingParams(double x_move, double y_move, double angle_move);
  void executeAction(int action_id);
  void manageState();  // 状態管理関数
  bool setCtrlModule(const std::string& module_name);
  void freeAllJoints();

private:
  void stateThread();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
  void yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                    const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                    const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg);
  void handleAttack();               // 攻撃処理
  void handlePostAttack();           // 攻撃後の処理
  void allowRotationOnly();          // 旋回のみ許可
  void transitionToInitPose();       // 初期姿勢への遷移
  void transitionToWalking();        // 自律移動への遷移
  void transitionToFallState();      // 転倒状態への遷移
  void transitionToIdleState();      // 脱力状態への遷移
  void transitionToAutoMoveState();  // 自律移動状態への遷移

  double quaternionToYaw(const geometry_msgs::Quaternion& q);
  double quaternionToPitch(const geometry_msgs::Quaternion& q);

  ros::Subscriber joy_sub_, imu_sub_, camera_info_sub_;
  ros::Publisher walking_command_pub_, walking_params_pub_, action_page_pub_;
  ros::ServiceClient client_;

  message_filters::Subscriber<jsk_recognition_msgs::ClassificationResult> class_sub_;
  message_filters::Subscriber<jsk_recognition_msgs::LabelArray> label_sub_;
  message_filters::Subscriber<jsk_recognition_msgs::RectArray> rect_sub_;
  typedef message_filters::sync_policies::ExactTime<jsk_recognition_msgs::ClassificationResult,
                                                    jsk_recognition_msgs::LabelArray, jsk_recognition_msgs::RectArray>
      SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

  std::vector<std::string> joint_names_;
  std::mutex state_mutex_;
  std::thread state_thread_;
  bool running_;

  // 内部状態変数
  std::string current_state_, previous_state_, next_state_;
  ros::Time action_start_time_;
  sensor_msgs::Joy last_joy_;
  sensor_msgs::Imu last_imu_;
  jsk_recognition_msgs::RectArray last_rects_;
  sensor_msgs::CameraInfo last_camera_info_;  // 最新のカメラインフォを保持する変数
  ros::Time last_rects_time_, last_imu_time_, last_joy_time_;

  double atk_rects_size_;
};

#endif  // ROBOONE_AUTO_H_
