#ifndef KUROKO_ACTION_MODULE_H_
#define KUROKO_ACTION_MODULE_H_

#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#define _USE_MATH_DEFINES
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_action_module_msgs/StartAction.h"
#include "animation_files.h"
#include "kuroko_walking_module_msgs/GetFloat.h"

namespace motion_control
{
class ActionModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<ActionModule>
{
public:
  ActionModule();
  ~ActionModule() override;

  // ROS Framework Functions
  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, double> sensors) override;

  void stop() override;
  bool isRunning() override;

  void onModuleEnable() override;
  void onModuleDisable() override;

private:
  robotis_framework::Robot* robot_ptr_;
  bool debug_messages_ = false;
  std::string current_animation_name_;
  int current_block_id_;
  double time_in_frame_ = 0.0;
  bool is_running_ = false;
  bool is_running_leg_ = false;

  boost::thread queue_thread_;
  ros::Publisher status_msg_pub_;
  ros::Publisher done_msg_pub_;
  ros::Publisher sync_write_pub_;

  int control_cycle_msec_;
  bool action_module_initialized_;
  trajectory_msgs::JointTrajectory current_trajectory_;
  size_t trajectory_index_ = 0;
  ros::Time trajectory_start_time_;
  bool start_playing_requested_ = false;
  bool stop_playing_requested_ = false;

  std::map<std::string, int> joint_name_to_dxl_id_;
  std::map<int, std::string> dxl_id_to_joint_name_;
  std::map<std::string, bool> action_joints_enable_;
  std::vector<std::string> animation_joint_names_;
  std::vector<std::string> leg_joint_names_;

  animation_system::Workspace workspace_;

  // ROS Topic Callback Functions
  bool isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                op3_action_module_msgs::IsRunning::Response& res);

  bool getRemainingTimeServiceCallback(kuroko_walking_module_msgs::GetFloat::Request& req,
                                       kuroko_walking_module_msgs::GetFloat::Response& res);

  bool getLegRemainingTimeServiceCallback(kuroko_walking_module_msgs::GetFloat::Request& req,
                                          kuroko_walking_module_msgs::GetFloat::Response& res);
  void animationNumberCallback(const std_msgs::Int32::ConstPtr& msg);
  void startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg);

  // User functions
  void queueThread();
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);
  void getJointNames();
  void torqueOnAll();
  void torqueOffAll();
  void initialPose();
  trajectory_msgs::JointTrajectory createJointTrajectory(const std::vector<animation_system::FrameData>& frames,
                                                         const double control_cycle_msec);
  std::vector<animation_system::FrameData> getFrameVector(const animation_system::AnimationData& animation_data);
};

}  // namespace motion_control

#endif  // KUROKO_ACTION_MODULE_H_
