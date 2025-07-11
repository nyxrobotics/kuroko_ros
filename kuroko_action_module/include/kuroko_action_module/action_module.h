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
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_action_module_msgs/StartAction.h"
#include "animation_files.h"

namespace motion_control
{
class ActionModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<ActionModule>
{
public:
  ActionModule();
  virtual ~ActionModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, double> sensors) override;

  void onModuleEnable() override;
  void onModuleDisable() override;
  void stop() override;
  bool isRunning() override;
  void torqueOnAll();
  void torqueOffAll();

private:
  std::string current_animation_name_;
  std::string current_block_id_;
  double time_in_frame_ = 0.0;
  bool is_running_ = false;

  boost::thread queue_thread_;
  ros::Publisher status_msg_pub_;
  ros::Publisher done_msg_pub_;
  ros::Publisher sync_write_pub_;

  int control_cycle_msec_;
  bool enable_;
  bool action_module_enabled_;

  std::map<std::string, int> joint_name_to_dxl_id_;
  std::map<int, std::string> dxl_id_to_joint_name_;
  std::map<std::string, robotis_framework::DynamixelState*> result_;
  std::map<std::string, robotis_framework::DynamixelState*> action_result_;
  std::map<std::string, bool> action_joints_enable_;
  std::vector<std::string> animation_joint_names_;

  animation_system::Workspace workspace_;

  void queueThread();
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);

  bool isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                op3_action_module_msgs::IsRunning::Response& res);
  void motionNumberCallback(const std_msgs::Int32::ConstPtr& msg);
  void startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg);

  void processAnimationStep();
  void executeFrame(const animation_system::FrameData& frame);
  void getJointNames();
};

}  // namespace motion_control

#endif  // KUROKO_ACTION_MODULE_H_
