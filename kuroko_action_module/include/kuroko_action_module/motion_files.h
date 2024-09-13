
#ifndef MOTION_FILES_H_
#define MOTION_FILES_H_

#include <trajectory_msgs/JointTrajectory.h>

namespace motion_control
{
class MotionSection
{
public:
  std::string section_name;
  std::vector<std::string> next_sections;
  trajectory_msgs::JointTrajectory joint_trajectory;
  trajectory_msgs::JointTrajectory getSortedJointTrajectory(const std::vector<std::string> joint_names_in);
};
class MotionFile
{
public:
  std::string motion_name;
  std::vector<MotionSection> motion_sections;
  MotionSection getMotionSection(const std::string& section_name);
};
class MotionFiles
{
public:
  std::vector<MotionFile> motion_files;
  MotionFile getMotionFile(const std::string& motion_name);
  bool hasMotion(const std::string& motion_name);
};
class MotionStatus
{
public:
  bool is_running;
  bool start_requested;
  bool stop_requested;
  bool abort_requested;
  std::string current_motion_name;
  std::string current_section_name;
  int current_frame_in_section;
  double current_time_in_section;
  std::string next_motion_name;
};
}  // namespace motion_control

#endif /* MOTION_FILES_H_ */
