#ifndef KUROKO_ACTION_MODULE_H_
#define KUROKO_ACTION_MODULE_H_

// Check if the C++ standard is 17 or later
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
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_action_module_msgs/StartAction.h"

namespace motion_control
{
/**************************************
 * Section             /----\
 *                    /|    |\
 *        /+---------/ |    | \
 *       / |        |  |    |  \
 * -----/  |        |  |    |   \----
 *      PRE  MAIN   PRE MAIN POST PAUSE
 ***************************************/
enum class MotionSection
{
  PRE_SECTION,
  MAIN_SECTION,
  POST_SECTION,
  PAUSE_SECTION
};

enum class FinishType
{
  ZERO_FINISH,
  NONE_ZERO_FINISH
};

class ActionModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<ActionModule>
{
public:
  ActionModule();
  virtual ~ActionModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, double> sensors) override;

  void stop();
  bool isRunning();
  void brake();

  void loadAllMotions(const std::string& directory);
  void playMotionByName(const std::string& motion_name);

private:
  int control_cycle_msec_;
  bool enable_;

  MotionSection current_section_;
  FinishType finish_type_;

  std::string module_name_;
  bool playing_;
  bool first_driving_start_;
  bool playing_finished_;
  int page_step_count_;
  int play_page_idx_;
  bool stop_playing_;
  bool action_module_enabled_;
  bool previous_running_;
  bool present_running_;
  std::string current_motion_;  // Added to track the current motion

  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;
  std::map<std::string, robotis_framework::DynamixelState*> action_result_;
  std::map<std::string, robotis_framework::DynamixelState*> result_;
  std::map<std::string, bool> action_joints_enable_;

  ros::Publisher status_msg_pub_;
  ros::Publisher done_msg_pub_;

  boost::thread queue_thread_;

  void queueThread();
  bool isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                op3_action_module_msgs::IsRunning::Response& res);
  void pageNumberCallback(const std_msgs::Int32::ConstPtr& msg);
  void startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);
  void processMotionStep();

  void loadConfigJointNames(const std::string& file_name);
  void loadYAMLFile(const std::string& file_name, const std::string& motion_name);

  std::vector<std::string> config_joint_names_;
  std::map<std::string, std::vector<std::vector<double>>> positions_map_;
  std::map<std::string, std::vector<std::vector<double>>> velocities_map_;
  std::map<std::string, std::vector<std::vector<double>>> accelerations_map_;
  std::map<std::string, std::vector<std::vector<double>>> efforts_map_;
  std::map<std::string, std::vector<double>> time_from_start_map_;
};

}  // namespace motion_control

#endif /* KUROKO_ACTION_MODULE_H_ */
