#ifndef UPPER_BODY_ACTION_MODULE_H
#define UPPER_BODY_ACTION_MODULE_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <yaml-cpp/yaml.h>
#include "upper_body_action_file_define.h"

namespace motion_control
{
class UpperBodyActionModule
{
public:
  UpperBodyActionModule();
  ~UpperBodyActionModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot);
  void queueThread();
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls, std::map<std::string, double> sensors);
  bool isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                op3_action_module_msgs::IsRunning::Response& res);
  void pageNumberCallback(const std_msgs::Int32::ConstPtr& msg);
  void startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg);

  void stop();
  bool isRunning();
  void onModuleEnable();
  void onModuleDisable();

private:
  int control_cycle_msec_;
  FILE* action_file_;

  bool enable_;
  bool playing_;
  bool action_module_enabled_;
  bool previous_running_;
  bool present_running_;
  bool first_driving_start_;
  bool playing_finished_;
  bool stop_playing_;

  int page_step_count_;
  int play_page_idx_;

  robotis_framework::Robot* robot_;

  trajectory_msgs::JointTrajectory motion_data_;

  bool loadFile(std::string file_name);
  bool loadMotionFromYAML(const std::string& yaml_file);
  bool start(int page_number);
  bool start(int page_number, action_file_define::Page* page);
  bool start(std::string page_name);
  void actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel*> dxls);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDoneMsg(std::string msg);
  std::string convertIntToString(int n);

  bool parseYAMLFile(const std::string& yaml_file, trajectory_msgs::JointTrajectory& motion_data);
};
}  // namespace motion_control

#endif  // UPPER_BODY_ACTION_MODULE_H
