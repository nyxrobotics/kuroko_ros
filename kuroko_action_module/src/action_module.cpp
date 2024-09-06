#include "kuroko_action_module/action_module.h"

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
ActionModule::ActionModule() : control_cycle_msec_(8), joints_enabled_(false)
{
  module_name_ = "action_module";
  control_mode_ = robotis_framework::PositionControl;
  motion_status_.is_running = false;
  motion_status_.current_motion_name = "";
  motion_status_.current_section_name = "";
  motion_status_.current_frame_in_section = 0;
}

ActionModule::~ActionModule()
{
  queue_thread_.join();
}

void ActionModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  ROS_INFO_STREAM("[ActionModule] Start initialization");
  ros::NodeHandle ros_node;
  std::string joint_names_path = ros::package::getPath("kuroko_action_module") + "/config/joint_names.yaml";
  std::string motion_path = ros::package::getPath("kuroko_action_module") + "/motion";
  loadConfigJointNames(joint_names_path);
  ROS_INFO_STREAM("[ActionModule] Loading modules for each joint (joint_names.yaml)");
  for (auto& dxl : robot->dxls_)
  {
    std::string joint_name = dxl.first;
    // Check if the joint is in the list of joint names
    if (std::find(config_joint_names_.begin(), config_joint_names_.end(), joint_name) == config_joint_names_.end())
      continue;
    robotis_framework::Dynamixel* dxl_info = dxl.second;
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
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&ActionModule::queueThread, this));
  ROS_INFO_STREAM("[ActionModule] Finish initialization");
}

void ActionModule::loadConfigJointNames(const std::string& file_name)
{
  YAML::Node config = YAML::LoadFile(file_name);
  ROS_INFO_STREAM("[ActionModule] Loaded Joint YAML file: " << file_name);
  config_joint_names_ = config["joint_names"].as<std::vector<std::string>>();
  ROS_INFO_STREAM("[ActionModule] config_joint_names_:\n" << YAML::Dump(YAML::Node(config_joint_names_)));
}

void ActionModule::loadAllMotions(const std::string& directory)
{
  for (const auto& entry : fs::directory_iterator(directory))
  {
    if (entry.path().extension() == ".yaml")
    {
      std::string motion_name = entry.path().stem().string();
      loadYAMLFile(entry.path().string(), motion_name);
    }
  }
}

void ActionModule::loadYAMLFile(const std::string& file_name, const std::string& motion_name)
{
  try
  {
    YAML::Node yaml_data = YAML::LoadFile(file_name);
    ROS_INFO_STREAM("[ActionModule] Loaded Motion YAML file: " << file_name);

    // Read sections from YAML file
    for (const auto& section : yaml_data["sections"])
    {
      std::string section_name = section["section_name"].as<std::string>();
      ROS_INFO_STREAM("[ActionModule] Loading section: " << section_name);

      std::vector<std::string> motion_joint_names =
          section["joint_trajectory"]["joint_names"].as<std::vector<std::string>>();
      std::vector<std::vector<double>> positions;
      std::vector<std::vector<double>> velocities;
      std::vector<std::vector<double>> accelerations;
      std::vector<std::vector<double>> efforts;
      std::vector<double> time_from_start;

      for (const auto& point : section["joint_trajectory"]["points"])
      {
        try
        {
          // Extract data from each point
          std::vector<double> position = point["positions"].as<std::vector<double>>();
          std::vector<double> velocity = point["velocities"].as<std::vector<double>>();
          std::vector<double> acceleration = point["accelerations"].as<std::vector<double>>();
          std::vector<double> effort = point["effort"].as<std::vector<double>>();
          double time_start = point["time_from_start"].as<double>();

          std::vector<double> mapped_position(config_joint_names_.size(), 0.0);
          std::vector<double> mapped_velocity(config_joint_names_.size(), 0.0);
          std::vector<double> mapped_acceleration(config_joint_names_.size(), 0.0);
          std::vector<double> mapped_effort(config_joint_names_.size(), 0.0);

          // Map the joint values based on the config_joint_names_ order
          for (size_t i = 0; i < motion_joint_names.size(); ++i)
          {
            auto it = std::find(config_joint_names_.begin(), config_joint_names_.end(), motion_joint_names[i]);
            if (it != config_joint_names_.end())
            {
              size_t index = std::distance(config_joint_names_.begin(), it);
              mapped_position[index] = position[i];
              mapped_velocity[index] = velocity[i];
              mapped_acceleration[index] = acceleration[i];
              mapped_effort[index] = effort[i];
            }
          }
          // Store the mapped values for each point
          positions.emplace_back(mapped_position);
          velocities.emplace_back(mapped_velocity);
          accelerations.emplace_back(mapped_acceleration);
          efforts.emplace_back(mapped_effort);
          time_from_start.emplace_back(time_start);
        }
        catch (const YAML::Exception& e)
        {
          ROS_ERROR_STREAM("[ActionModule] YAML Exception while parsing points: " << e.what());
          return;
        }
      }
      // You can add code here to store the section data as needed
      ROS_INFO_STREAM("[ActionModule] Section '" << section_name << "' successfully loaded.");
    }
    ROS_INFO_STREAM("[ActionModule] Motion '" << motion_name << "' successfully loaded.");
  }
  catch (const YAML::Exception& e)
  {
    ROS_ERROR_STREAM("[ActionModule] Failed to load YAML file: " << file_name << " with error: " << e.what());
    return;
  }
}

void ActionModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;
  ros::WallDuration duration(control_cycle_msec_ / 1000.0);

  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/motion_control/status", 5);
  done_msg_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/movement_done", 5);

  ros::Subscriber action_page_sub =
      ros_node.subscribe("/motion_control/action/page_num", 5, &ActionModule::pageNumberCallback, this);
  ros::Subscriber start_action_sub =
      ros_node.subscribe("/motion_control/action/start_action", 5, &ActionModule::startActionCallback, this);
  ros::ServiceServer is_running_server =
      ros_node.advertiseService("/motion_control/action/is_running", &ActionModule::isRunningServiceCallback, this);

  ros_node.setCallbackQueue(&callback_queue);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

bool ActionModule::isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                            op3_action_module_msgs::IsRunning::Response& res)
{
  res.is_running = isRunning();
  return true;
}

void ActionModule::pageNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if (!enable_)
  {
    std::string status_msg = "Action Module is not enabled";
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return;
  }

  if (msg->data == -1)
  {
    motion_status_.stop_requested = true;
  }
  else if (msg->data == -2)
  {
    brake();
  }
  else
  {
    for (auto& joint_enable : action_joints_enable_)
      joint_enable.second = true;

    processMotionStep();

    std::string status_msg = "Succeed to start page " + std::to_string(msg->data);
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
  }
}

void ActionModule::startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg)
{
  if (!enable_)
  {
    std::string status_msg = "Action Module is not enabled";
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return;
  }

  if (msg->page_num == -1)
  {
    motion_status_.stop_requested = true;
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

    processMotionStep();

    std::string status_msg = "Succeed to start page " + std::to_string(msg->page_num);
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
  }
}

void ActionModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                           std::map<std::string, double> sensors)
{
  if (!joints_enabled_)
    return;

  if (action_module_enabled_)
  {
    for (auto& dxl : dxls)
    {
      std::string joint_name = dxl.first;
      auto result_it = result_.find(joint_name);
      if (result_it == result_.end())
        continue;
      else
      {
        result_it->second->goal_position_ = dxl.second->dxl_state_->goal_position_;
        action_result_[joint_name]->goal_position_ = dxl.second->dxl_state_->goal_position_;
      }
    }
    ROS_INFO_STREAM("Action Module is enabled");
    action_module_enabled_ = false;
  }

  processMotionStep();

  for (auto& action_enable : action_joints_enable_)
  {
    if (action_enable.second)
      result_[action_enable.first]->goal_position_ = action_result_[action_enable.first]->goal_position_;
  }

  if (motion_status_.start_requested && !motion_status_.is_running)
  {
    std::string status_msg = "Action_Start";
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
  }
  if (motion_status_.stop_requested && motion_status_.is_running)
  {
    for (auto& action_result : action_result_)
      action_result.second->goal_position_ = result_[action_result.first]->goal_position_;

    std::string status_msg = "Action_Finish";
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    publishDoneMsg("action");
  }
}

void ActionModule::playMotionByName(const std::string& motion_name)
{
  if (motion_files_.hasMotion(motion_name))
  {
    ROS_ERROR_STREAM("Motion not found: " << motion_name);
    return;
  }

  motion_status_.next_motion_name = motion_name;  // Set current_motion_ to the selected motion
  MotionFile motion_file = motion_files_.getMotionFile(motion_name);
  for (auto& section : motion_file.motion_sections)
  {
    std::string section_name = section.section_name;
    ROS_INFO_STREAM("Playing section: " << section_name);
    for (auto& point : section.joint_trajectory.points)
    {
      trajectory_msgs::JointTrajectory sorted_joint_trajectory = section.getSortedJointTrajectory(config_joint_names_);
      for (size_t j = 0; j < config_joint_names_.size(); ++j)
      {
        std::string joint_name = config_joint_names_[j];
        action_result_[joint_name]->goal_position_ = point.positions[j];
      }
      ros::Duration(point.time_from_start.toSec()).sleep();
    }
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
  if (!motion_status_.is_running)
    return;

  for (auto& joint_enable : action_joints_enable_)
  {
    if (joint_enable.second)
    {
      const std::string& joint_name = joint_enable.first;
      robotis_framework::DynamixelState* dxl_state = action_result_[joint_name];

      if (dxl_state != nullptr)
      {
        result_[joint_name]->goal_position_ = dxl_state->goal_position_;
      }
    }
  }

  motion_status_.current_frame_in_section++;
  if (motion_status_.current_frame_in_section >= motion_files_.getMotionFile(motion_status_.current_motion_name)
                                                     .getMotionSection(motion_status_.current_section_name)
                                                     .joint_trajectory.points.size())
  {
    motion_status_.is_running = false;
    publishDoneMsg("Motion completed");
  }
}

void ActionModule::brake()
{
  // motion_status_.abort_requested = true;
  motion_status_.is_running = false;
}

void ActionModule::onModuleEnable()
{
  ROS_INFO_STREAM("[ActionModule] onModuleEnable called");
  action_module_enabled_ = true;
}

void ActionModule::onModuleDisable()
{
  ROS_INFO_STREAM("[ActionModule] onModuleDisable called");
  action_module_enabled_ = false;
  brake();
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
