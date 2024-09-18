#include "kuroko_action_module/action_module.h"
#include "ros/console.h"

// Check if the C++ standard is 17 or later
#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

namespace motion_control
{
ActionModule::ActionModule()
  : control_cycle_msec_(8)
  , start_playing_requested_(false)
  , stop_playing_requested_(false)
  , action_module_enabled_(false)
  , send_next_frame_(false)
{
  enable_ = false;
  module_name_ = "action_module";
  control_mode_ = robotis_framework::PositionControl;
  motion_status_.is_running = false;
  motion_status_.current_motion_name = "";
  motion_status_.current_section_name = "";
  motion_status_.current_frame_in_section = 0;
  motion_status_.current_time_in_section = 0.0;
  motion_status_.next_motion_name = "";
  motion_status_.start_requested = false;
  motion_status_.stop_requested = false;
  motion_status_.abort_requested = false;
}

ActionModule::~ActionModule()
{
  queue_thread_.join();
}

void ActionModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ActionModule::queueThread, this));
  ROS_INFO_STREAM("[ActionModule] Start initialization");
  ros::NodeHandle ros_node;
  std::string joint_names_path = ros::package::getPath("kuroko_action_module") + "/config/joint_names.yaml";
  std::string motion_path = ros::package::getPath("kuroko_action_module") + "/motion";
  loadConfigJointNames(joint_names_path);
  ROS_INFO_STREAM("[ActionModule] Loading modules for each joint (joint_names.yaml)");
  for (auto& dxl : robot->dxls_)
  {
    std::string joint_name = dxl.first;
    robotis_framework::Dynamixel* dxl_info = dxl.second;
    // Check if the joint is in the list of joint names
    if (std::find(config_joint_names_.begin(), config_joint_names_.end(), joint_name) == config_joint_names_.end())
    {
      ROS_WARN_STREAM("[ActionModule] Joint " << joint_name << " not found in the joint_names.yaml file. Skipping.");
      continue;
    }
    ROS_INFO_STREAM("[ActionModule] Loading module for joint: " << joint_name);

    joint_name_to_dxl_id_[joint_name] = dxl_info->id_;
    dxl_id_to_joint_name_[dxl_info->id_] = joint_name;
    action_result_[joint_name] = new robotis_framework::DynamixelState();
    action_result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    action_joints_enable_[joint_name] = false;
  }
  // If there is any config_joint_names_ that is not in the robot->dxls_, show a warning and remove it
  for (const auto& joint_name : config_joint_names_)
  {
    if (joint_name_to_dxl_id_.find(joint_name) == joint_name_to_dxl_id_.end())
    {
      ROS_WARN_STREAM("[ActionModule] Joint '" << joint_name << "' not found in the robot. Removing from the list.");
      config_joint_names_.erase(std::remove(config_joint_names_.begin(), config_joint_names_.end(), joint_name),
                                config_joint_names_.end());
    }
  }
  loadAllMotions(motion_path);
  motion_status_.is_running = false;
  ROS_INFO_STREAM("[ActionModule] Finish initialization");
}

void ActionModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/motion_control/status", 5);
  done_msg_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/movement_done", 5);

  ros::Subscriber action_page_sub =
      ros_node.subscribe("/motion_control/action/page_num", 5, &ActionModule::motionNumberCallback, this);
  ros::Subscriber start_action_sub =
      ros_node.subscribe("/motion_control/action/start_action", 5, &ActionModule::startActionCallback, this);
  ros::ServiceServer is_running_server =
      ros_node.advertiseService("/motion_control/action/is_running", &ActionModule::isRunningServiceCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

bool ActionModule::isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                            op3_action_module_msgs::IsRunning::Response& res)
{
  res.is_running = isRunning();
  return true;
}

void ActionModule::motionNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO_STREAM("[ActionModule] motionNumberCallback called");
  if (!enable_)
  {
    std::string status_msg = "Action Module is not enabled";
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return;
  }

  if (msg->data == -1)
  {
    stop();
  }
  else if (msg->data == -2)
  {
    brake();
  }
  else
  {
    // ROS_INFO("Setting all joints to true");
    for (auto& joint_enable : action_joints_enable_)
    {
      joint_enable.second = true;
    }
    // ROS_INFO("Getting motion names");
    std::vector<std::string> motion_names = motion_files_.getMotionNames();
    // ROS_INFO("Motioin names:");
    // for (const auto& motion_name : motion_names)
    // {
    //   ROS_INFO_STREAM("- " << motion_name);
    // }
    int motion_id = msg->data;
    if (motion_id < 0 || motion_id > motion_names.size() - 1)
    {
      std::string status_msg = "[ActionModule] Invalid Motion ID : " + std::to_string(msg->data);
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      publishDoneMsg("[ActionModule] action_failed");
      return;
    }
    ROS_INFO_STREAM("Setting next motion name");
    std::string motion_name = motion_names[motion_id];
    motion_status_.next_motion_name = motion_name;
    motion_status_.start_requested = true;
    std::string status_msg = "Succeed to start motion id:" + std::to_string(msg->data) + ", name:" + motion_name;
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
  }
}

void ActionModule::startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg)
{
  ROS_INFO_STREAM("[ActionModule] startActionCallback called");
  if (!enable_)
  {
    std::string status_msg = "Action Module is not enabled";
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return;
  }

  if (msg->page_num == -1)
  {
    stop();
  }
  else if (msg->page_num == -2)
  {
    brake();
  }
  else
  {
    for (auto& joint_enable : action_joints_enable_)
      joint_enable.second = false;

    for (const auto& joint_name : msg->joint_name_array)
    {
      auto it = action_joints_enable_.find(joint_name);
      if (it == action_joints_enable_.end())
      {
        std::string status_msg = "Invalid Joint Name : " + joint_name;
        ROS_INFO_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        publishDoneMsg("action_failed");
        return;
      }
      else
      {
        it->second = true;
      }
    }
    std::string status_msg = "Succeed to start page " + std::to_string(msg->page_num);
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
  }
}

void ActionModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                           std::map<std::string, double> sensors)
{
  if (!enable_)
    return;

  if (action_module_enabled_)
  {
    for (auto& dxl : dxls)
    {
      std::string joint_name = dxl.first;
      auto result_it = result_.find(joint_name);
      if (result_it == result_.end())
      {
        ROS_INFO_STREAM("Joint " << joint_name << " NOT found in the result_ map, skipping");
        continue;
      }
      else
      {
        ROS_INFO_STREAM("Joint " << joint_name << " found in the result_ map");
        result_it->second->goal_position_ = dxl.second->dxl_state_->goal_position_;
        action_result_[joint_name]->goal_position_ = dxl.second->dxl_state_->goal_position_;
      }
    }
    action_module_enabled_ = false;
  }

  processMotionStep();
  // Send target joint angle if necessary
  if (send_next_frame_)
  {
    ROS_INFO_STREAM("[ActionModule] Sending next frame: " << motion_status_.current_frame_in_section);
    trajectory_msgs::JointTrajectory sorted_trajectory = motion_files_.getMotionFile(motion_status_.current_motion_name)
                                                             .getMotionSection(motion_status_.current_section_name)
                                                             .getSortedJointTrajectory(config_joint_names_);
    for (int i = 0; i < sorted_trajectory.joint_names.size(); i++)
    {
      std::string joint_name = sorted_trajectory.joint_names[i];
      if (action_joints_enable_[joint_name])
      {
        double goal_position = sorted_trajectory.points[motion_status_.current_frame_in_section].positions[i];
        result_[joint_name]->goal_position_ = goal_position;
      }
    }
    send_next_frame_ = false;
  }
}

bool ActionModule::playMotionByName(const std::string& motion_name)
{
  ROS_INFO_STREAM("[ActionModule] playMotionByName called");

  if (enable_ == false)
  {
    std::string status_msg = "Action Module is disabled";
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }
  if (motion_files_.hasMotion(motion_name))
  {
    std::string status_msg = "Motion " + motion_name + " does not exist";
    ROS_ERROR_STREAM("Motion not found: " << motion_name);
    ROS_ERROR_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return false;
  }

  motion_status_.next_motion_name = motion_name;  // Set current_motion_ to the selected motion
  motion_status_.start_requested = true;          // Set start_requested to true
  MotionFile motion_file = motion_files_.getMotionFile(motion_name);
  return true;
}

void ActionModule::loadConfigJointNames(const std::string& file_name)
{
  YAML::Node config = YAML::LoadFile(file_name);
  ROS_INFO_STREAM("[ActionModule] Loaded Joint YAML file: " << file_name);
  config_joint_names_ = config["joint_names"].as<std::vector<std::string>>();
  ROS_INFO_STREAM("[ActionModule] config_joint_names_:\n" << YAML::Dump(YAML::Node(config_joint_names_)));
}

void ActionModule::saveAllMotions(const std::string& directory)
{
  // Ensure the directory exists
  if (!fs::exists(directory))
  {
    fs::create_directories(directory);
  }

  for (const auto& motion_file : motion_files_.motion_files)
  {
    std::string file_path = directory + "/" + motion_file.motion_name + ".yaml";

    // Create a YAML node to store the motion data
    YAML::Node yaml_data;

    for (const auto& section : motion_file.motion_sections)
    {
      YAML::Node section_node;
      section_node["section_name"] = section.section_name;

      // Add next_sections if available
      if (!section.next_sections.empty())
      {
        section_node["next_sections"] = section.next_sections;
      }

      YAML::Node joint_trajectory_node;
      joint_trajectory_node["joint_names"] = section.joint_trajectory.joint_names;

      for (const auto& point : section.joint_trajectory.points)
      {
        YAML::Node point_node;
        point_node["positions"] = point.positions;
        point_node["velocities"] = point.velocities;
        point_node["accelerations"] = point.accelerations;  // Include accelerations
        point_node["effort"] = point.effort;
        point_node["time_from_start"] = point.time_from_start.toSec();

        // Set points to flow style
        point_node["positions"].SetStyle(YAML::EmitterStyle::Flow);
        point_node["velocities"].SetStyle(YAML::EmitterStyle::Flow);
        point_node["accelerations"].SetStyle(YAML::EmitterStyle::Flow);  // Set accelerations to flow style
        point_node["effort"].SetStyle(YAML::EmitterStyle::Flow);

        joint_trajectory_node["points"].push_back(point_node);
      }

      section_node["joint_trajectory"] = joint_trajectory_node;
      yaml_data["sections"].push_back(section_node);
    }

    // Write the YAML data to the file, overwriting if the file already exists
    std::ofstream fout(file_path);
    fout << yaml_data;
    fout.close();

    ROS_INFO_STREAM("[ActionModule] Motion " << motion_file.motion_name << " saved to " << file_path);
  }
}

void ActionModule::loadAllMotions(const std::string& directory)
{
  std::vector<fs::path> yaml_files;

  // Collect YAML files in the directory
  for (const auto& entry : fs::directory_iterator(directory))
  {
    if (entry.path().extension() == ".yaml")
    {
      yaml_files.push_back(entry.path());
    }
  }

  // Sort the files by their filename
  std::sort(yaml_files.begin(), yaml_files.end());

  // Load each YAML file in the sorted order
  for (const auto& file : yaml_files)
  {
    std::string motion_name = file.stem().string();
    loadMotionYAML(file.string(), motion_name);
  }
}

void ActionModule::loadMotionYAML(const std::string& file_name, const std::string& motion_name)
{
  try
  {
    YAML::Node yaml_data = YAML::LoadFile(file_name);
    ROS_INFO_STREAM("[ActionModule] Loaded Motion YAML file: " << file_name);

    motion_control::MotionFile motion_file;
    motion_file.motion_name = motion_name;

    // Load sections from the YAML
    for (const auto& section : yaml_data["sections"])
    {
      motion_control::MotionSection motion_section;
      motion_section.section_name = section["section_name"].as<std::string>();

      // Load next_sections
      if (section["next_sections"])
      {
        motion_section.next_sections = section["next_sections"].as<std::vector<std::string>>();
      }

      // Load joint trajectory
      motion_section.joint_trajectory.joint_names =
          section["joint_trajectory"]["joint_names"].as<std::vector<std::string>>();

      for (const auto& point : section["joint_trajectory"]["points"])
      {
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions = point["positions"].as<std::vector<double>>();
        trajectory_point.velocities = point["velocities"].as<std::vector<double>>();

        // Load accelerations if available
        if (point["accelerations"])
        {
          trajectory_point.accelerations = point["accelerations"].as<std::vector<double>>();
        }

        trajectory_point.effort = point["effort"].as<std::vector<double>>();
        trajectory_point.time_from_start = ros::Duration(point["time_from_start"].as<double>());

        motion_section.joint_trajectory.points.push_back(trajectory_point);
      }

      motion_file.motion_sections.push_back(motion_section);
    }

    // Store the motion file in MotionFiles
    motion_files_.motion_files.push_back(motion_file);
    ROS_INFO_STREAM("[ActionModule] Motion " << motion_name << " loaded successfully.");
  }
  catch (YAML::Exception& e)
  {
    ROS_ERROR_STREAM("[ActionModule] Failed to load Motion YAML file: " << file_name << " Error: " << e.what());
  }
}

void ActionModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type = type;
  status.module_name = "Action";
  status.status_msg = msg;

  status_msg_pub_.publish(status);
}

void ActionModule::publishDoneMsg(std::string msg)
{
  std_msgs::String done_msg;
  done_msg.data = msg;

  done_msg_pub_.publish(done_msg);
}

void ActionModule::processMotionStep()
{
  // Update the current time within the section
  motion_status_.current_time_in_section += control_cycle_msec_ / 1000.0;

  // Check if motion is not running and no start is requested
  if (!motion_status_.is_running && !motion_status_.start_requested)
    return;

  // Start the motion if it's requested
  if (!motion_status_.is_running && motion_status_.start_requested && !motion_status_.next_motion_name.empty())
  {
    ROS_INFO_STREAM("[ActionModule] processMotionStep: start motion " << motion_status_.next_motion_name);
    motion_status_.current_motion_name = motion_status_.next_motion_name;

    if (!motion_files_.hasMotion(motion_status_.current_motion_name))
    {
      ROS_ERROR_STREAM("[ActionModule] processMotionStep: motion " << motion_status_.current_motion_name
                                                                   << " not found");
      return;
    }

    motion_status_.current_section_name =
        motion_files_.getMotionFile(motion_status_.current_motion_name).motion_sections[0].section_name;
    motion_status_.current_frame_in_section = 0;
    motion_status_.is_running = true;
    motion_status_.start_requested = false;
    motion_status_.stop_requested = false;
    motion_status_.abort_requested = false;
    motion_status_.current_time_in_section = 0.0;
    send_next_frame_ = true;

    ROS_INFO_STREAM(
        "[ActionModule] processMotionStep: move to initial frame: " << motion_status_.current_frame_in_section);
    return;
  }

  // Get the current section's joint trajectory points
  const auto& points = motion_files_.getMotionFile(motion_status_.current_motion_name)
                           .getMotionSection(motion_status_.current_section_name)
                           .joint_trajectory.points;

  // Skip interpolation for the first frame, directly move to the next frame
  if (motion_status_.current_frame_in_section == 0)
  {
    const auto& first_frame = points[0];
    for (size_t i = 0; i < first_frame.positions.size(); i++)
    {
      std::string joint_name = config_joint_names_[i];
      if (action_joints_enable_[joint_name])
      {
        result_[joint_name]->goal_position_ = first_frame.positions[i];
        result_[joint_name]->goal_velocity_ = first_frame.velocities[i];
      }
    }
    // Move to the next frame
    motion_status_.current_frame_in_section++;
    return;
  }

  // Get the previous and current frame for interpolation
  const auto& previous_frame = points[motion_status_.current_frame_in_section - 1];
  const auto& current_frame = points[motion_status_.current_frame_in_section];

  // Calculate the cumulative time up to the previous frame
  double total_time_until_previous_frame = 0.0;
  for (int i = 0; i < motion_status_.current_frame_in_section; ++i)
  {
    total_time_until_previous_frame += points[i].time_from_start.toSec();
  }

  // Calculate the time elapsed within the current frame
  double current_time_in_frame = motion_status_.current_time_in_section - total_time_until_previous_frame;
  double target_time_in_frame = current_frame.time_from_start.toSec();  // Only use current frame's time_from_start

  // Perform interpolation between the previous and current frame
  if (current_time_in_frame < target_time_in_frame)
  {
    for (size_t i = 0; i < previous_frame.positions.size(); i++)
    {
      std::string joint_name = config_joint_names_[i];
      if (action_joints_enable_[joint_name])
      {
        double start_position = previous_frame.positions[i];
        double goal_position = current_frame.positions[i];
        double velocity = current_frame.velocities[i];  // Use the velocity of the current frame

        // Calculate the distance to travel
        double distance = goal_position - start_position;

        // If the velocity is not sufficient to reach the goal in time, adjust the velocity
        double required_velocity = distance / target_time_in_frame;

        // Adjust the velocity's sign to ensure movement towards the goal
        if ((goal_position - start_position) * velocity < 0)
        {
          velocity = -velocity;  // Flip the sign if it's moving in the wrong direction
        }

        // If the current velocity is too slow, overwrite it with the required velocity
        if (std::abs(velocity) < std::abs(required_velocity))
        {
          velocity = required_velocity;
        }

        // Calculate the position based on the velocity and time
        double new_position = start_position + velocity * current_time_in_frame;

        // Clamp the position to avoid overshooting the goal
        if ((velocity > 0 && new_position > goal_position) || (velocity < 0 && new_position < goal_position))
        {
          new_position = goal_position;
        }

        // Set the calculated position
        result_[joint_name]->goal_position_ = new_position;
        result_[joint_name]->goal_velocity_ = velocity;
      }
    }
  }

  // If the current frame's time has elapsed, move to the next frame
  if (current_time_in_frame >= target_time_in_frame)
  {
    // Move to the next frame
    motion_status_.current_frame_in_section++;

    // If we reached the last frame, ensure it is sent with velocity set to 0
    if (motion_status_.current_frame_in_section >= points.size())
    {
      const auto& last_frame = points.back();
      for (size_t i = 0; i < last_frame.positions.size(); i++)
      {
        std::string joint_name = config_joint_names_[i];
        if (action_joints_enable_[joint_name])
        {
          // Explicitly send the last frame's position with velocity set to 0
          result_[joint_name]->goal_position_ = last_frame.positions[i];
          result_[joint_name]->goal_velocity_ = 0.0;
        }
      }

      // Check if enough time has passed for the last frame
      if (motion_status_.current_time_in_section >= last_frame.time_from_start.toSec())
      {
        motion_status_.is_running = false;
        send_next_frame_ = false;
        ROS_INFO("[ActionModule] processMotionStep: finish motion");
        publishDoneMsg("Motion completed");
      }
    }
  }
}

void ActionModule::brake()
{
  // motion_status_.abort_requested = true;
  motion_status_.is_running = false;
}

void ActionModule::onModuleEnable()
{
  ROS_INFO_STREAM("[ActionModule] Module Enabled");
  action_module_enabled_ = true;
}

void ActionModule::onModuleDisable()
{
  ROS_INFO_STREAM("[ActionModule] Module Disabled");
  brake();
  action_module_enabled_ = false;
}

bool ActionModule::isRunning()
{
  return motion_status_.is_running;
}

void ActionModule::stop()
{
  motion_status_.stop_requested = true;
}

std::vector<std::string> ActionModule::getMotionNames()
{
  std::vector<std::string> motion_names;
  for (const auto& motion : motion_files_.motion_files)
  {
    motion_names.push_back(motion.motion_name);
  }
  return motion_names;
}

}  // namespace motion_control
