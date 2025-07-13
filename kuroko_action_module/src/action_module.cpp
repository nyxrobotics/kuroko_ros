#include "kuroko_action_module/action_module.h"
#include <vector>

namespace motion_control
{
ActionModule::ActionModule() : control_cycle_msec_(8), action_module_enabled_(false)
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
  queue_thread_ = boost::thread(boost::bind(&ActionModule::queueThread, this));
  ROS_INFO_STREAM("[ActionModule] Initializing");

  ros::NodeHandle nh;
  std::string workspace_path = ros::package::getPath("kuroko_action_module") + "/motion";
  ROS_INFO_STREAM("[ActionModule] Loading workspace from: " << workspace_path);
  workspace_.loadWorkspace(workspace_path);
  std::vector<std::string> animation_names = workspace_.getAnimationNames();
  ROS_INFO_STREAM("[ActionModule] Found Animations:");
  for (const auto& animation_name : animation_names)
  {
    ROS_INFO_STREAM(" - " << animation_name);
  }

  // collect joint names
  animation_joint_names_.clear();
  const animation_system::FrameData& pose_data = workspace_.getInitialPoseData();
  for (const auto& joint_pair : pose_data.joints)
  {
    animation_joint_names_.push_back(joint_pair.first);
  }
  std::sort(animation_joint_names_.begin(), animation_joint_names_.end());
  ROS_INFO_STREAM("[ActionModule] Found Joints:");
  for (const auto& joint_name : animation_joint_names_)
  {
    ROS_INFO_STREAM(" - " << joint_name);
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
    ROS_INFO_STREAM("[ActionModule] Loading module for joint: " << joint_name);

    joint_name_to_dxl_id_[joint_name] = dxl_info->id_;
    dxl_id_to_joint_name_[dxl_info->id_] = joint_name;

    action_result_[joint_name] = new robotis_framework::DynamixelState();
    action_result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    action_joints_enable_[joint_name] = false;
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

  nh.subscribe("/motion_control/action/animation_num", 5, &ActionModule::animationNumberCallback, this);
  nh.subscribe("/motion_control/action/start_action", 5, &ActionModule::startActionCallback, this);
  nh.advertiseService("/motion_control/action/is_running", &ActionModule::isRunningServiceCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (nh.ok())
    queue.callAvailable(duration);
}

void ActionModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                           std::map<std::string, double> sensors)
{
  if (!enable_)
    return;

  if (action_module_enabled_)
  {
    for (auto& dxl_pair : dxls)
    {
      const std::string& name = dxl_pair.first;
      auto* dxl = dxl_pair.second;
      if (result_.count(name))
      {
        result_[name]->goal_position_ = dxl->dxl_state_->goal_position_;
        action_result_[name]->goal_position_ = dxl->dxl_state_->goal_position_;
      }
    }
    action_module_enabled_ = false;
  }

  processAnimationStep();
}

void ActionModule::stop()
{
  return;
}

void ActionModule::onModuleEnable()
{
  ROS_INFO("[ActionModule] Module Enabled");
  action_module_enabled_ = true;
}

void ActionModule::onModuleDisable()
{
  ROS_INFO("[ActionModule] Module Disabled");
  action_module_enabled_ = false;
}

bool ActionModule::isRunning()
{
  return false;
}

bool ActionModule::isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                            op3_action_module_msgs::IsRunning::Response& res)
{
  res.is_running = isRunning();
  return true;
}

void ActionModule::animationNumberCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if (!enable_)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Action Module is not enabled");
    return;
  }
  if (msg->data == -1)
  {
    ROS_INFO("Stopping all joints");
    torqueOnAll();
  }
  else if (msg->data == -2)
  {
    ROS_INFO("Braking all joints");
    torqueOffAll();
  }

  std::vector<std::string> anim_names = workspace_.getAnimationNames();
  if (msg->data < 0 || msg->data >= anim_names.size())
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Invalid animation index");
    return;
  }

  std::string name = anim_names[msg->data];
  const auto& anim = workspace_.getAnimationData(name);
  const auto& blocks = anim.getBlockIds();
  if (blocks.empty())
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "No blocks in animation");
    return;
  }

  const auto& start_block = anim.getAnimationBlock("start");
  if (start_block.type != "frame")
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Start block is not a frame");
    return;
  }

  const auto& frame = anim.getFrameData(start_block.filename);
  executeFrame(frame);

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Started animation: " + name);
}

void ActionModule::startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg)
{
  if (!enable_)
  {
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

  if (!anim.blocks.count("start"))
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Start block not found");
    return;
  }

  current_block_id_ = "start";
  time_in_frame_ = 0.0;
  is_running_ = true;

  for (const auto& jname : msg->joint_name_array)
  {
    if (action_joints_enable_.count(jname))
      action_joints_enable_[jname] = true;
  }

  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Started animation: " + current_animation_name_);
}

void ActionModule::executeFrame(const animation_system::FrameData& frame)
{
  for (const auto& joint_pair : frame.joints)
  {
    const std::string& joint_name = joint_pair.first;
    const auto& joint_data = joint_pair.second;
    if (!action_joints_enable_[joint_name])
      continue;

    double position = joint_data.position;
    result_[joint_name]->goal_position_ = position;
  }
}

void ActionModule::processAnimationStep()
{
  if (!is_running_ || current_animation_name_.empty() || current_block_id_.empty())
    return;

  const animation_system::AnimationData& anim = workspace_.getAnimationData(current_animation_name_);

  if (anim.blocks.find(current_block_id_) == anim.blocks.end())
  {
    is_running_ = false;
    publishDoneMsg("animation_failed");
    return;
  }

  const animation_system::AnimationBlock& block = anim.getAnimationBlock(current_block_id_);

  if (block.type != "frame")
  {
    is_running_ = false;
    publishDoneMsg("invalid_block_type");
    return;
  }

  const animation_system::FrameData& frame = anim.getFrameData(block.filename);
  const double total_duration = frame.move_duration + frame.wait_duration;

  if (time_in_frame_ == 0.0)
  {
    executeFrame(frame);
  }

  time_in_frame_ += static_cast<double>(control_cycle_msec_) / 1000.0;

  if (time_in_frame_ < total_duration)
    return;

  if (block.output_ids.empty())
  {
    is_running_ = false;
    publishDoneMsg("animation_completed");
    return;
  }

  current_block_id_ = block.output_ids.front();
  time_in_frame_ = 0.0;
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
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "torque_enable";

  // Iterate through all dxls to enable torque
  for (const auto& dxl : joint_name_to_dxl_id_)
  {
    syncwrite_msg.joint_name.push_back(dxl.first);  // Add joint name
    syncwrite_msg.value.push_back(1);               // Enable torque (1)
  }

  // Publish SyncWrite message to enable torque for all joints
  sync_write_pub_.publish(syncwrite_msg);

  ROS_INFO("Torque enabled for all joints");
}

void ActionModule::torqueOffAll()
{
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "torque_enable";

  // Iterate through all dxls to disable torque
  for (const auto& dxl : joint_name_to_dxl_id_)
  {
    syncwrite_msg.joint_name.push_back(dxl.first);  // Add joint name
    syncwrite_msg.value.push_back(0);               // Disable torque (0)
  }

  // Publish SyncWrite message to disable torque for all joints
  sync_write_pub_.publish(syncwrite_msg);

  ROS_INFO("Torque disabled for all joints");
}

}  // namespace motion_control
