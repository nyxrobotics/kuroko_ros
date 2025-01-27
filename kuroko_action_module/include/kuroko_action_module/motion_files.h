#ifndef MOTION_FILES_H_
#define MOTION_FILES_H_

#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>

namespace motion_control
{
class MotionSection
{
public:
  std::string section_name;                           // Name of the current section
  std::string continue_section;                       // Section to proceed under normal conditions
  std::string stop_section;                           // Section to proceed when motion stops
  std::vector<std::string> jump_sections;             // Sections to jump to based on user input
  trajectory_msgs::JointTrajectory joint_trajectory;  // Joint trajectory for the current section

  // Get sorted joint trajectory based on input joint order
  trajectory_msgs::JointTrajectory getSortedJointTrajectory(const std::vector<std::string>& joint_names_in);
};

class MotionFile
{
public:
  std::string motion_name;                     // Name of the motion file
  std::vector<MotionSection> motion_sections;  // List of motion sections

  // Get a motion section by name
  MotionSection* getMotionSection(const std::string& section_name);

  // Check if a motion section exists
  bool hasSection(const std::string& section_name);
};

class MotionFiles
{
public:
  std::vector<MotionFile> motion_files;  // List of all motion files

  // Get a motion file by name
  MotionFile* getMotionFile(const std::string& motion_name);

  // Check if a motion exists
  bool hasMotion(const std::string& motion_name);

  // Get all motion names
  std::vector<std::string> getMotionNames();
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
