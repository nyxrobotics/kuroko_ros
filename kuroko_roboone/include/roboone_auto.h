#ifndef ROBOONE_AUTO_H_
#define ROBOONE_AUTO_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <kuroko_walking_module_msgs/WalkingParam.h>
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
#include <eigen3/Eigen/Eigen>
#include <vector>

class RobooneAuto
{
public:
  RobooneAuto(ros::NodeHandle& nh);
  ~RobooneAuto();

  void startWalking();
  void stopWalking();
  void abortWalking();
  void setWalkSteps(double x_step, double y_step, double yaw_step);
  void setSquatSteps(double x_step, double y_step, double yaw_step);
  void setJumpSteps(double x_step, double y_step, double yaw_step);
  void executeAction(std::string action_name);
  void manageState();  // 状態管理関数
  bool setCtrlModule(const std::string& module_name);
  void freeAllJoints();
  void enableAllJoints();

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
  void transitionToPauseWalkingState();
  void transitionToSquatState();
  void handleFall();

  Eigen::Vector3d imuQuaternionToRollPitchYaw(const Eigen::Quaterniond& q);
  Eigen::Quaterniond imuRollPitchYawToQuaternion(const Eigen::Vector3d& rpy);
  double wrapToPi(double angle);

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
  std::thread state_thread_;
  bool running_;

  // 内部状態変数
  std::string current_state_, previous_state_, next_state_, current_module_;
  sensor_msgs::Joy last_joy_;
  sensor_msgs::Imu last_imu_;
  jsk_recognition_msgs::ClassificationResult last_class_;
  jsk_recognition_msgs::RectArray last_rects_;
  jsk_recognition_msgs::LabelArray last_labels_;
  sensor_msgs::CameraInfo last_camera_info_;  // 最新のカメラインフォを保持する変数
  ros::Time last_rects_time_, last_imu_time_, last_joy_time_, fall_detected_time_, robot_detected_time_, attacked_time_;
  char last_target_detected_direction_;
  jsk_recognition_msgs::Rect robot_detected_rect_;

  // Manage Attrack
  double atk_rects_size_;
  // Manage walk
  double x_forward_step_max_, x_backward_step_max_;
  double y_step_max_, yaw_step_max_;
  std::string walk_status_;
  // Manage balance interruption
  kuroko_walking_module_msgs::WalkingParam walk_param_;
  kuroko_walking_module_msgs::WalkingParam idle_param_;
  kuroko_walking_module_msgs::WalkingParam squat_param_;
  kuroko_walking_module_msgs::WalkingParam jump_param_;
  double stable_angle_threshold_;  // 安定状態の角度閾値
  double squat_angle_threshold_;   // ホールド状態の角度閾値
  double jump_angle_threshold_;    // ジャンピングホールド状態の角度閾値
  double fall_angle_threshold_;    // 転倒判定の角度閾値
  double stable_duration_;         // 安定状態に復帰するための必要時間
  double squat_duration_;          // ホールド状態の最低持続時間
  double jump_duration_;           // ジャンプ状態の最低持続時間
  double fall_duration_;           // 転倒判定の待機時間
  // Manage balance interruption
  std::map<std::string, int> action_id_map_;
  std::map<std::string, double> action_duration_map_;
  std::string last_attack_name_;
  std::string action_name_;
  ros::Time action_start_time_;
  double action_duration_;
  ros::Time squat_start_time_;
  ros::Time jump_start_time_;
  ros::Time walk_start_time_;
  double max_squat_duration_;
  double min_walk_duration_;
  double walk_stop_duration_;
  bool force_walk_;
};

#endif  // ROBOONE_AUTO_H_
