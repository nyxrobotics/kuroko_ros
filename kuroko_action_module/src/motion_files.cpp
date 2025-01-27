#include "kuroko_action_module/motion_files.h"
#include <algorithm>
#include <ros/console.h>

namespace motion_control
{
trajectory_msgs::JointTrajectory MotionSection::getSortedJointTrajectory(const std::vector<std::string>& joint_names_in)
{
  trajectory_msgs::JointTrajectory sorted_trajectory = joint_trajectory;

  for (auto& point : sorted_trajectory.points)
  {
    std::vector<double> sorted_positions(joint_names_in.size(), 0.0);
    std::vector<double> sorted_velocities(joint_names_in.size(), 0.0);
    for (size_t i = 0; i < joint_names_in.size(); ++i)
    {
      auto it = std::find(joint_trajectory.joint_names.begin(), joint_trajectory.joint_names.end(), joint_names_in[i]);
      if (it != joint_trajectory.joint_names.end())
      {
        size_t index = std::distance(joint_trajectory.joint_names.begin(), it);
        sorted_positions[i] = point.positions[index];
        if (index < point.velocities.size())
        {
          sorted_velocities[i] = point.velocities[index];
        }
      }
    }
    point.positions = sorted_positions;
    point.velocities = sorted_velocities;
  }
  sorted_trajectory.joint_names = joint_names_in;

  return sorted_trajectory;
}

MotionSection* MotionFile::getMotionSection(const std::string& section_name)
{
  for (auto& section : motion_sections)
  {
    if (section.section_name == section_name)
    {
      return &section;
    }
  }

  ROS_ERROR_STREAM("[MotionFile] Section '" << section_name << "' not found in motion '" << motion_name
                                            << "'. Skipping.");
  return nullptr;
}

bool MotionFile::hasSection(const std::string& section_name)
{
  for (const auto& section : motion_sections)
  {
    if (section.section_name == section_name)
    {
      return true;
    }
  }
  return false;
}

MotionFile* MotionFiles::getMotionFile(const std::string& motion_name)
{
  for (auto& file : motion_files)
  {
    if (file.motion_name == motion_name)
    {
      return &file;
    }
  }

  ROS_ERROR_STREAM("[MotionFiles] Motion '" << motion_name << "' not found. Skipping.");
  return nullptr;
}

bool MotionFiles::hasMotion(const std::string& motion_name)
{
  for (const auto& file : motion_files)
  {
    if (file.motion_name == motion_name)
    {
      return true;
    }
  }
  return false;
}

std::vector<std::string> MotionFiles::getMotionNames()
{
  std::vector<std::string> motion_names;
  for (const auto& file : motion_files)
  {
    motion_names.push_back(file.motion_name);
  }
  return motion_names;
}

}  // namespace motion_control
