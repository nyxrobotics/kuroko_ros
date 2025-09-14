#ifndef ROBOONE_AUTO_H_
#define ROBOONE_AUTO_H_

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
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

struct AttackData
{
  std::string name;
  double min_distance;
  double max_distance;
  int8_t normal_max_count;
  int8_t ultimate_max_count;
  int8_t current_count;
  bool force_aim;
};

class RobooneAuto
{
public:
  RobooneAuto(ros::NodeHandle& nh);
  ~RobooneAuto();

  void startWalking();
  void stopWalking();
  void abortWalking();
  void setWalkSteps(double x_step, double y_step, double yaw_step);
  void setIdle();
  void setSquat();
  void setFrontJump();
  void setRearJump();
  void executeAction(std::string action_name);
  void manageState();  // 状態管理関数
  bool setCtrlModule(const std::string& module_name);
  void freeAllJoints();
  void enableAllJoints();
  void initialPose();

private:
  void stateThread();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  void rangeCallback(const sensor_msgs::Range::ConstPtr& range);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
  void yoloCallback(const jsk_recognition_msgs::ClassificationResult::ConstPtr& class_msg,
                    const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg,
                    const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg);
  void transitionToInit();  // 初期姿勢への遷移
  void transitionToRun();   // 自律移動への遷移
  void transitionToFall();  // 転倒状態への遷移
  void transitionToFree();  // 脱力状態への遷移
  void transitionToHold();
  void transitionToSquat();
  void transitionToJump();
  void handleFall();
  void handleRun();
  std::string decideAttack(double target_distance, bool is_aimed, bool is_left);
  bool isSameAttack(const std::string& action_name, const std::string& last_action_name);
  int8_t getActionDirection(const std::string& action_name);

  Eigen::Vector3d imuQuaternionToRollPitchYaw(const Eigen::Quaterniond& q);
  Eigen::Quaterniond imuRollPitchYawToQuaternion(const Eigen::Vector3d& rpy);
  double wrapToPi(double angle);

  ros::Subscriber joy_sub_, imu_sub_, camera_info_sub_, range_sub_;
  ros::Publisher walking_command_pub_, walking_params_pub_, action_page_pub_, init_pose_pub_;
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

  // Camera params
  double camera_height_;
  double camera_fov_h_, camera_fov_v_;
  ros::Time camera_start_time_;
  double camera_startup_duration_;
  double rects_timeout_duration_;

  // 内部状態変数
  std::string current_state_, previous_state_, next_state_, current_module_;
  sensor_msgs::Joy last_joy_;
  sensor_msgs::Imu last_imu_;
  sensor_msgs::Range last_range_;
  jsk_recognition_msgs::ClassificationResult last_class_;
  jsk_recognition_msgs::RectArray last_rects_;
  jsk_recognition_msgs::LabelArray last_labels_;
  sensor_msgs::CameraInfo last_camera_info_;  // 最新のカメラインフォを保持する変数
  ros::Time last_rects_time_, fall_detected_time_, robot_detected_time_, attacked_time_;
  char last_target_detected_direction_;
  jsk_recognition_msgs::Rect robot_detected_rect_;

  // Manage walk
  double x_forward_step_max_, x_backward_step_max_;
  double y_step_max_, yaw_step_max_;
  std::string walk_status_;

  // Manage balance interruption
  double stable_detect_angle_;     // 安定状態に復帰する角度
  double stable_detect_duration_;  // 安定状態に復帰するための必要時間
  double squat_detect_angle_;      // しゃがみ状態に移行する角度
  double squat_detect_duration_;   // しゃがみ判定の待機時間
  double jump_detect_angle_;       // ジャンプ状態に移行する角度
  double jump_detect_duration_;    // ジャンプ判定の待機時間
  double fall_detect_angle_;       // 転倒状態に移行する角度
  double fall_detect_duration_;    // 転倒判定の待機時間

  // Walk
  kuroko_walking_module_msgs::WalkingParam walk_param_;
  kuroko_walking_module_msgs::WalkingParam idle_param_;
  ros::Time walk_start_time_;
  double walk_stop_duration_;
  double min_walk_duration_;
  bool force_walk_;
  bool force_front_walk_;
  ros::Time front_walk_start_time_;
  ros::Time back_walk_start_time_;
  double min_front_walk_duration_;
  double max_back_walk_duration_;
  int8_t walk_direction_;  // -1: back, 0: stop, 1: front

  // Squat
  kuroko_walking_module_msgs::WalkingParam squat_param_;
  ros::Time squat_start_time_;
  double min_squat_duration_;  // しゃがみ状態の最低持続時間
  double max_squat_duration_;
  double squat_duration_;

  // Front jump
  kuroko_walking_module_msgs::WalkingParam jump_param_;
  ros::Time jump_start_time_;
  double jump_duration_;  // ジャンプ状態の持続時間

  // Manage action
  std::map<std::string, int> action_id_map_;
  std::map<std::string, double> action_duration_map_;
  std::vector<AttackData> attack_actions_;
  std::string last_attack_name_;
  std::string action_name_;
  ros::Time action_start_time_;
  double action_duration_;
  bool ultimate_mode_;
  bool force_aim_;

  // Manage Attrack
  double atk_min_rect_size_;
  double attack_rect_distance_;
  int attack_count_;
  int max_attack_count_;

  // Manage startup
  ros::Time run_start_time_;
  double run_startup_duration_;
};

#endif  // ROBOONE_AUTO_H_
