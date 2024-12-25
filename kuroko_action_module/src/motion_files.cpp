#include "kuroko_action_module/motion_files.h"

namespace motion_control
{
trajectory_msgs::JointTrajectory MotionSection::getSortedJointTrajectory(const std::vector<std::string> joint_names_in)
{
  trajectory_msgs::JointTrajectory joint_trajectory_out;
  // Set the joint names and initialize output trajectory
  joint_trajectory_out.joint_names = joint_names_in;
  joint_trajectory_out.header.stamp = ros::Time::now();
  joint_trajectory_out.points.clear();

  // Iterate over each point in the original joint_trajectory
  for (const auto& point : joint_trajectory.points)
  {
    // Create a new trajectory point for output, named sorted_point
    trajectory_msgs::JointTrajectoryPoint sorted_point;
    sorted_point.time_from_start = point.time_from_start;

    // Resize the sorted_point's arrays to match the input joints
    sorted_point.positions.resize(joint_names_in.size(), 0.0);
    sorted_point.velocities.resize(joint_names_in.size(), 0.0);
    sorted_point.effort.resize(joint_names_in.size(), 0.0);

    // Iterate over each joint name in joint_names_in
    for (size_t i = 0; i < joint_names_in.size(); ++i)
    {
      // Find the matching joint in the original trajectory
      for (size_t j = 0; j < joint_trajectory.joint_names.size(); ++j)
      {
        if (joint_names_in[i] == joint_trajectory.joint_names[j])
        {
          // Copy the corresponding joint values to sorted_point
          sorted_point.positions[i] = point.positions[j];
          if (!point.velocities.empty())
            sorted_point.velocities[i] = point.velocities[j];
          if (!point.effort.empty())
            sorted_point.effort[i] = point.effort[j];
          break;
        }
      }
    }

    // Add the sorted point to the output trajectory
    joint_trajectory_out.points.push_back(sorted_point);
  }
  return joint_trajectory_out;
}

MotionSection MotionFile::getMotionSection(const std::string& section_name)
{
  for (const auto& section : motion_sections)
  {
    if (section.section_name == section_name)
    {
      return section;
    }
  }
  // If the section is not found, return an empty MotionSection
  return MotionSection();
}

MotionFile MotionFiles::getMotionFile(const std::string& motion_name)
{
  for (const auto& file : motion_files)
  {
    if (file.motion_name == motion_name)
    {
      return file;
    }
  }
  // If the file is not found, return an empty MotionFile
  return MotionFile();
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
  for (const auto& motion : motion_files)
  {
    motion_names.push_back(motion.motion_name);
  }
  return motion_names;
}

}  // namespace motion_control
