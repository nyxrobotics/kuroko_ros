#include "kuroko_action_module/action_module.h"
#include <vector>
#include "ros/console.h"

namespace motion_control
{
ActionModule::ActionModule() : control_cycle_msec_(8), action_module_initialized_(false)
{
  ROS_INFO_STREAM("[ActionModule] Constructor called");
  module_name_ = "action_module";
  control_mode_ = robotis_framework::PositionControl;
  enable_ = false;
}

ActionModule::~ActionModule()
{
  queue_thread_.join();
}

void ActionModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread([this] { queueThread(); });
  ROS_INFO_STREAM("[ActionModule] Initializing");

  ros::NodeHandle nh;
  std::string workspace_path = ros::package::getPath("kuroko_motion_data") + "/config";
  ROS_INFO_STREAM("[ActionModule] Loading workspace from: " << workspace_path);
  workspace_.loadWorkspace(workspace_path);
  std::vector<std::string> animation_names = workspace_.getAnimationNames();
  ROS_INFO_STREAM("[ActionModule] Found Animations:");
  for (const auto& animation_name : animation_names)
  {
    // fet total time for each animation
    double total_time = 0;
    const auto& anim = workspace_.getAnimationData(animation_name);
    std::vector<animation_system::FrameData> frames = getFrameVector(anim);
    for (const auto& frame : frames)
    {
      total_time += frame.move_duration + frame.wait_duration;
    }
    ROS_INFO_STREAM(" - " << animation_name << " (Total Time: " << total_time << " seconds)");
  }

  // collect joint names
  animation_joint_names_.clear();
  const animation_system::FrameData& pose_data = workspace_.getInitialPoseData();
  for (const auto& joint_pair : pose_data.joints)
  {
    animation_joint_names_.push_back(joint_pair.first);
  }
  std::sort(animation_joint_names_.begin(), animation_joint_names_.end());
  if (debug_messages_)
  {
    ROS_INFO_STREAM("[ActionModule] Found Joints:");
    for (const auto& joint_name : animation_joint_names_)
    {
      ROS_INFO_STREAM(" - " << joint_name);
    }
  }

  leg_joint_names_.clear();
  leg_joint_names_.push_back("hip_r_roll");
  leg_joint_names_.push_back("hip_r_pitch");
  leg_joint_names_.push_back("thigh_r_active");
  leg_joint_names_.push_back("shin_r_active");
  leg_joint_names_.push_back("ankle_r_roll");
  leg_joint_names_.push_back("ankle_r_yaw");
  leg_joint_names_.push_back("hip_l_roll");
  leg_joint_names_.push_back("hip_l_pitch");
  leg_joint_names_.push_back("thigh_l_active");
  leg_joint_names_.push_back("shin_l_active");
  leg_joint_names_.push_back("ankle_l_roll");
  leg_joint_names_.push_back("ankle_l_yaw");
  for (const auto& joint_name : leg_joint_names_)
  {
    if (std::find(animation_joint_names_.begin(), animation_joint_names_.end(), joint_name) ==
        animation_joint_names_.end())
    {
      ROS_WARN_STREAM("[ActionModule] Leg joint " << joint_name << " not found in the animation joint names. Removing.");
      leg_joint_names_.erase(std::remove(leg_joint_names_.begin(), leg_joint_names_.end(), joint_name),
                             leg_joint_names_.end());
    }
  }

  for (auto& dxl : robot->dxls_)
  {
    std::string joint_name = dxl.first;
    robotis_framework::Dynamixel* dxl_info = dxl.second;

    // Check if the joint is in the list of joint names
    if (std::find(animation_joint_names_.begin(), animation_joint_names_.end(), joint_name) ==
        animation_joint_names_.end())
    {
      ROS_WARN_STREAM("[ActionModule] Joint " << joint_name << " not found in the animation joint names. Skipping.");
      continue;
    }
    if (debug_messages_)
      ROS_INFO_STREAM("[ActionModule] Loading module for joint: " << joint_name);

    joint_name_to_dxl_id_[joint_name] = dxl_info->id_;
    dxl_id_to_joint_name_[dxl_info->id_] = joint_name;

    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    action_joints_enable_[joint_name] = true;
  }
  // If there is any animation_joint_names_ that is not in the robot->dxls_, show a warning and remove it
  for (const auto& joint_name : animation_joint_names_)
  {
    if (joint_name_to_dxl_id_.find(joint_name) == joint_name_to_dxl_id_.end())
    {
      ROS_WARN_STREAM("[ActionModule] Joint '" << joint_name << "' not found in the robot. Removing from the list.");
      animation_joint_names_.erase(std::remove(animation_joint_names_.begin(), animation_joint_names_.end(), joint_name),
                                   animation_joint_names_.end());
    }
  }
  ROS_INFO_STREAM("[ActionModule] Initialization complete");
}

void ActionModule::queueThread()
{
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  status_msg_pub_ = nh.advertise<robotis_controller_msgs::StatusMsg>("/motion_control/status", 5);
  done_msg_pub_ = nh.advertise<std_msgs::String>("/motion_control/movement_done", 5);
  sync_write_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/motion_control/sync_write_item", 5);

  ros::Subscriber action_page_sub =
      nh.subscribe("/motion_control/action/animation_num", 5, &ActionModule::animationNumberCallback, this);
  ros::Subscriber start_action_sub =
      nh.subscribe("/motion_control/action/start_action", 5, &ActionModule::startActionCallback, this);
  ros::ServiceServer is_running_server =
      nh.advertiseService("/motion_control/action/is_running", &ActionModule::isRunningServiceCallback, this);
  ros::ServiceServer get_remaining_time_server = nh.advertiseService(
      "/motion_control/action/get_remaining_time", &ActionModule::getRemainingTimeServiceCallback, this);
  ros::ServiceServer get_leg_remaining_time_server = nh.advertiseService(
      "/motion_control/action/get_leg_remaining_time", &ActionModule::getLegRemainingTimeServiceCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (nh.ok())
    queue.callAvailable(duration);
}

void ActionModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                           std::map<std::string, double> sensors)
{
  // Start trajectory playback if requested
  if (start_playing_requested_)
  {
    ROS_INFO("[ActionModule] Start playing triggered");
    is_running_ = true;
    is_running_leg_ = true;
    trajectory_start_time_ = ros::Time::now();
    trajectory_index_ = 0;
    start_playing_requested_ = false;
  }

  if (!is_running_)
    return;

  // If finished
  if (trajectory_index_ >= current_trajectory_.points.size())
  {
    ROS_INFO("[ActionModule] Animation playback finished: %s", current_animation_name_.c_str());
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish animation");
    publishDoneMsg(current_animation_name_);
    is_running_ = false;
    is_running_leg_ = false;
    return;
  }

  // If leg movements finished
  if (is_running_leg_)
  {
    if (trajectory_index_ < current_trajectory_.points.size() - 1)
    {
      // Get movement for legs in remaining trajectory
      double leg_remaining_movement = 0.0;
      for (int i = trajectory_index_; i < current_trajectory_.points.size() - 1; ++i)
      {
        const auto& start_point = current_trajectory_.points[i];
        const auto& end_point = current_trajectory_.points[i + 1];
        for (size_t j = 0; j < current_trajectory_.joint_names.size(); ++j)
        {
          const std::string& joint_name = current_trajectory_.joint_names[j];
          if (std::find(leg_joint_names_.begin(), leg_joint_names_.end(), joint_name) != leg_joint_names_.end())
          {
            double movement = fabs(end_point.positions[j] - start_point.positions[j]);
            leg_remaining_movement += movement;
          }
        }
      }
      if (leg_remaining_movement < 0.01)
      {
        ROS_INFO("[ActionModule] Leg movements finished");
        is_running_leg_ = false;
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Leg movements finished");
      }
    }
    else
    {
      ROS_INFO("[ActionModule] Leg movements finished");
      is_running_leg_ = false;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Leg movements finished");
    }
  }

  const auto& traj = current_trajectory_;
  const auto& point = traj.points[trajectory_index_];
  ros::Duration elapsed = ros::Time::now() - trajectory_start_time_;

  if (elapsed >= point.time_from_start)
  {
    for (size_t i = 0; i < traj.joint_names.size(); ++i)
    {
      const std::string& joint_name = traj.joint_names[i];
      double goal_position = point.positions[i];
      double goal_velocity = point.velocities.size() > i ? point.velocities[i] : 0.0;
      if (dxls.count(joint_name) && dxls[joint_name] && dxls[joint_name]->dxl_state_)
      {
        result_[joint_name]->goal_position_ = goal_position;
      }
      else
      {
        ROS_WARN("[ActionModule] Joint skipped: %s - dxl missing or dxl_state_ null", joint_name.c_str());
      }
    }
    ++trajectory_index_;
  }
}

void ActionModule::stop()
{
}

void ActionModule::onModuleEnable()
{
  ROS_INFO("[ActionModule] Module Enabled");
  action_module_initialized_ = true;
  // Enable all joints
  for (const auto& joint : action_joints_enable_)
  {
    action_joints_enable_[joint.first] = true;
  }

  // Move to initial pose
  const auto& init_pose = workspace_.getInitialPoseData();
  std::vector<animation_system::FrameData> frames;
  frames.push_back(init_pose);
  current_trajectory_ = createJointTrajectory(frames, control_cycle_msec_);
  current_animation_name_ = "initial_pose";
  start_playing_requested_ = true;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Moved to initial pose");
}

void ActionModule::onModuleDisable()
{
  ROS_INFO("[ActionModule] Module Disabled");
  action_module_initialized_ = false;
  // Disable all joints
  for (const auto& joint : action_joints_enable_)
  {
    action_joints_enable_[joint.first] = false;
  }
}

bool ActionModule::isRunning()
{
  return !(!is_running_ || !is_running_leg_);
}

bool ActionModule::isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                            op3_action_module_msgs::IsRunning::Response& res)
{
  res.is_running = is_running_;
  return true;
}

bool ActionModule::getRemainingTimeServiceCallback(kuroko_walking_module_msgs::GetFloat::Request& req,
                                                   kuroko_walking_module_msgs::GetFloat::Response& res)
{
  double remaining_time = 0.0;
  if (!is_running_ || trajectory_index_ > current_trajectory_.points.size() - 2)
  {
    res.data = 0.0;
    return true;
  }
  for (size_t i = trajectory_index_; i < current_trajectory_.points.size() - 2; ++i)
  {
    const auto& start_point = current_trajectory_.points[i];
    const auto& end_point = current_trajectory_.points[i + 1];
    remaining_time += (end_point.time_from_start - start_point.time_from_start).toSec();
  }
  res.data = remaining_time;
  return true;
}

bool ActionModule::getLegRemainingTimeServiceCallback(kuroko_walking_module_msgs::GetFloat::Request& req,
                                                      kuroko_walking_module_msgs::GetFloat::Response& res)
{
  double leg_leg_movement_threshold = 0.01;
  double leg_movement = 0.0;
  double leg_finish_index = current_trajectory_.points.size() - 1;
  if (!is_running_ || !is_running_leg_ || trajectory_index_ >= current_trajectory_.points.size() - 1)
  {
    res.data = 0.0;
    return true;
  }
  // search for the last point where total leg movement is greater than the threshold
  for (int i = current_trajectory_.points.size() - 1; i >= trajectory_index_; --i)
  {
    const auto& point = current_trajectory_.points[i];
    for (const auto& joint_name : leg_joint_names_)
    {
      if (point.positions.size() > 0 && point.velocities.size() > 0)
      {
        auto it = joint_name_to_dxl_id_.find(joint_name);
        if (it != joint_name_to_dxl_id_.end())
        {
          int dxl_id = it->second;
          size_t index = std::distance(current_trajectory_.joint_names.begin(),
                                       std::find(current_trajectory_.joint_names.begin(),
                                                 current_trajectory_.joint_names.end(), joint_name));
          leg_movement += fabs(point.positions[index] - result_[joint_name]->goal_position_);
        }
      }
    }
    if (leg_movement > leg_leg_movement_threshold)
    {
      if (i < current_trajectory_.points.size() - 2)
        leg_finish_index = i + 1;
      else
        leg_finish_index = i;
      break;
    }
  }
  // Get remaining time for the leg movements
  double remaining_time = 0.0;
  for (size_t i = trajectory_index_; i < leg_finish_index; ++i)
  {
    const auto& start_point = current_trajectory_.points[i];
    const auto& end_point = current_trajectory_.points[i + 1];
    remaining_time += (end_point.time_from_start - start_point.time_from_start).toSec();
  }
  res.data = remaining_time;
  return true;
}

void ActionModule::animationNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("[ActionModule] Animation number received: %d", msg->data);
  if (!enable_)
  {
    ROS_INFO("[ActionModule] Action Module is not enabled");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Action Module is not enabled");
    return;
  }

  if (msg->data == -1)
  {
    ROS_INFO("[ActionModule] Stopping all joints");
    torqueOnAll();
    return;
  }
  else if (msg->data == -2)
  {
    ROS_INFO("[ActionModule] Braking all joints");
    torqueOffAll();
    return;
  }

  std::vector<std::string> anim_names = workspace_.getAnimationNames();
  if (msg->data < 0 || msg->data >= anim_names.size())
  {
    ROS_ERROR("[ActionModule] Invalid animation index: %d", msg->data);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Invalid animation index");
    return;
  }

  current_animation_name_ = anim_names[msg->data];
  const auto& anim = workspace_.getAnimationData(current_animation_name_);

  try
  {
    current_block_id_ = anim.getStartBlockId();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("[ActionModule] Failed to get start block ID: %s", e.what());
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, e.what());
    return;
  }

  std::vector<animation_system::FrameData> frames = getFrameVector(anim);
  current_trajectory_ = createJointTrajectory(frames, control_cycle_msec_);
  start_playing_requested_ = true;

  ROS_INFO("[ActionModule] Starting animation: %s", current_animation_name_.c_str());
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Started animation: " + current_animation_name_);
}

void ActionModule::startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg)
{
  ROS_INFO("[ActionModule] Start action callback received");
  if (!enable_)
  {
    ROS_INFO("[ActionModule] Action Module is not enabled");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Action Module is not enabled");
    return;
  }

  std::vector<std::string> anim_names = workspace_.getAnimationNames();
  if (msg->page_num < 0 || msg->page_num >= anim_names.size())
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Invalid animation index");
    return;
  }

  current_animation_name_ = anim_names[msg->page_num];
  const auto& anim = workspace_.getAnimationData(current_animation_name_);

  try
  {
    current_block_id_ = anim.getStartBlockId();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("[ActionModule] Failed to get start block ID: %s", e.what());
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, e.what());
    return;
  }

  start_playing_requested_ = true;

  for (const auto& jname : msg->joint_name_array)
  {
    if (action_joints_enable_.count(jname))
      action_joints_enable_[jname] = true;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Started animation: " + current_animation_name_);
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
  std_msgs::String done;
  done.data = msg;
  done_msg_pub_.publish(done);
}

void ActionModule::torqueOnAll()
{
  robotis_controller_msgs::SyncWriteItem msg;
  msg.item_name = "torque_enable";
  for (const auto& dxl : joint_name_to_dxl_id_)
  {
    msg.joint_name.push_back(dxl.first);
    msg.value.push_back(1);
  }
  sync_write_pub_.publish(msg);
  ROS_INFO("Torque enabled for all joints");
}

void ActionModule::torqueOffAll()
{
  robotis_controller_msgs::SyncWriteItem msg;
  msg.item_name = "torque_enable";
  for (const auto& dxl : joint_name_to_dxl_id_)
  {
    msg.joint_name.push_back(dxl.first);
    msg.value.push_back(0);
  }
  sync_write_pub_.publish(msg);
  ROS_INFO("Torque disabled for all joints");
}

trajectory_msgs::JointTrajectory
ActionModule::createJointTrajectory(const std::vector<animation_system::FrameData>& frames,
                                    const double control_cycle_msec)
{
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names = animation_joint_names_;

  if (frames.empty())
    return trajectory;

  const auto& initial_frame = frames[0];
  std::map<std::string, double> current_position;
  for (const auto& name : animation_joint_names_)
  {
    auto it = initial_frame.joints.find(name);
    current_position[name] = (it != initial_frame.joints.end()) ? it->second.position : 0.0;
  }

  if (frames.size() == 1)
  {
    // Only one frame — create a single point to go directly to that pose
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(0.0);  // apply immediately

    for (const auto& name : animation_joint_names_)
    {
      double pos = current_position[name];
      point.positions.push_back(pos);
      point.velocities.push_back(0.0);
    }

    trajectory.points.push_back(point);
    return trajectory;
  }

  for (const auto& name : animation_joint_names_)
  {
    auto it = initial_frame.joints.find(name);
    current_position[name] = (it != initial_frame.joints.end()) ? it->second.position : 0.0;
  }

  double time_from_start = 0.0;

  for (size_t i = 1; i < frames.size(); ++i)
  {
    const auto& frame = frames[i];
    double total_frame_duration = frame.move_duration + frame.wait_duration;

    double control_cycle_sec = control_cycle_msec / 1000.0;
    int steps = static_cast<int>(total_frame_duration / control_cycle_sec);
    if (steps < 1)
      steps = 1;

    // --- Precompute joint-wise durations ---
    std::map<std::string, double> joint_move_duration;
    std::map<std::string, double> joint_wait_duration;

    for (const auto& name : animation_joint_names_)
    {
      const auto& joint = frame.joints.count(name) ? frame.joints.at(name) : animation_system::JointData();
      double scale = (joint.speed_scale > 1e-3) ? joint.speed_scale : 1.0;

      double move_dur = frame.move_duration / scale;
      move_dur = std::min(move_dur, total_frame_duration);  // avoid overshooting

      joint_move_duration[name] = move_dur;
      joint_wait_duration[name] = std::max(0.0, total_frame_duration - move_dur);
    }

    // --- Interpolate points ---
    for (int s = 0; s < steps; ++s)
    {
      double time_ratio = static_cast<double>(s + 1) / steps;
      double global_time = time_from_start + time_ratio * total_frame_duration;

      trajectory_msgs::JointTrajectoryPoint point;
      point.time_from_start = ros::Duration(global_time);

      for (const auto& name : animation_joint_names_)
      {
        double start_pos = current_position[name];
        double goal_pos = frame.joints.count(name) ? frame.joints.at(name).position : start_pos;
        double move_dur = joint_move_duration[name];
        double wait_dur = joint_wait_duration[name];
        double progress;

        if (global_time - time_from_start <= move_dur)
        {
          // in move duration
          double move_time = global_time - time_from_start;
          progress = move_time / std::max(move_dur, 1e-6);
        }
        else
        {
          // in wait duration
          progress = 1.0;
        }

        double position = start_pos + progress * (goal_pos - start_pos);
        double velocity = std::abs(goal_pos - start_pos) / std::max(move_dur, 1e-6);

        point.positions.push_back(position);
        point.velocities.push_back(velocity);
      }

      trajectory.points.push_back(point);
    }

    for (const auto& name : animation_joint_names_)
    {
      if (frame.joints.count(name))
        current_position[name] = frame.joints.at(name).position;
    }

    time_from_start += total_frame_duration;
  }

  if (debug_messages_)
  {
    ROS_INFO("[ActionModule] Trajectory has %lu points", trajectory.points.size());
    ROS_INFO("[ActionModule] trajectory.joint_names size = %lu", trajectory.joint_names.size());
  }

  for (const auto& name : trajectory.joint_names)
  {
    ROS_INFO("- %s", name.c_str());
  }

  return trajectory;
}

std::vector<animation_system::FrameData>
ActionModule::getFrameVector(const animation_system::AnimationData& animation_data)
{
  std::vector<animation_system::FrameData> result;
  int current_id = animation_data.getStartBlockId();
  std::set<int> visited_ids;

  if (debug_messages_)
    ROS_INFO("[ActionModule] Starting frame traversal from block_id %d", current_id);

  while (animation_data.blocks.count(current_id))
  {
    if (debug_messages_)
      ROS_INFO("[ActionModule] Processing block_id %d", current_id);
    if (visited_ids.count(current_id))
    {
      if (debug_messages_)
        ROS_WARN("[ActionModule] Detected loop at block_id %d, stopping traversal.", current_id);
      break;
    }

    visited_ids.insert(current_id);
    const auto& block = animation_data.getAnimationBlock(current_id);

    if (block.type == "frame")
    {
      try
      {
        animation_system::FrameData frame = animation_data.getFrameData(block.filename);
        result.push_back(frame);
      }
      catch (const std::exception& e)
      {
        ROS_WARN("[ActionModule] Failed to load frame '%s': %s", block.filename.c_str(), e.what());
      }
    }

    if (block.output_ids.empty())
    {
      break;  // no next block
    }

    current_id = block.output_ids.front();  // follow first output only
  }

  return result;
}

}  // namespace motion_control
