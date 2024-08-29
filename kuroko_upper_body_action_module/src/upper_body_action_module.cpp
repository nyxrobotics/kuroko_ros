#include "kuroko_upper_body_action_module/upper_body_action_module.h"

namespace motion_control
{
UpperBodyActionModule::UpperBodyActionModule()
  : control_cycle_msec_(8)
  , enable_(false)
  , current_section_(MotionSection::PAUSE_SECTION)
  , finish_type_(FinishType::ZERO_FINISH)
  , playing_(false)
  , first_driving_start_(false)
  , playing_finished_(true)
  , page_step_count_(0)
  , play_page_idx_(0)
  , stop_playing_(true)
  , action_module_enabled_(false)
  , previous_running_(false)
  , present_running_(false)
{
  module_name_ = "action_module";  // set unique module name
  control_mode_ = robotis_framework::PositionControl;
}

UpperBodyActionModule::~UpperBodyActionModule()
{
  queue_thread_.join();
}

void UpperBodyActionModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&UpperBodyActionModule::queueThread, this));

  // init result, joint_id_table
  for (auto& dxl : robot->dxls_)
  {
    std::string joint_name = dxl.first;
    robotis_framework::Dynamixel* dxl_info = dxl.second;

    joint_name_to_id_[joint_name] = dxl_info->id_;
    joint_id_to_name_[dxl_info->id_] = joint_name;
    action_result_[joint_name] = new robotis_framework::DynamixelState();
    action_result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    action_joints_enable_[joint_name] = false;
  }

  ros::NodeHandle ros_node;
  std::string path = ros::package::getPath("kuroko_upper_body_action_module") + "/data/motion.yaml";
  std::string action_file_path = ros_node.param<std::string>("action_file_path", path);

  loadYAMLFile(action_file_path);

  playing_ = false;
}

void UpperBodyActionModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publisher */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);
  done_msg_pub_ = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);

  /* subscriber */
  ros::Subscriber action_page_sub =
      ros_node.subscribe("/robotis/action/page_num", 0, &UpperBodyActionModule::pageNumberCallback, this);
  ros::Subscriber start_action_sub =
      ros_node.subscribe("/robotis/action/start_action", 0, &UpperBodyActionModule::startActionCallback, this);

  /* ROS Service Callback Functions */
  ros::ServiceServer is_running_server =
      ros_node.advertiseService("/robotis/action/is_running", &UpperBodyActionModule::isRunningServiceCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

bool UpperBodyActionModule::isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request& req,
                                                     op3_action_module_msgs::IsRunning::Response& res)
{
  res.is_running = isRunning();
  return true;
}

void UpperBodyActionModule::pageNumberCallback(const std_msgs::Int32::ConstPtr& msg)
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
    stop();
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

void UpperBodyActionModule::startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg)
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

    processMotionStep();

    std::string status_msg = "Succeed to start page " + std::to_string(msg->page_num);
    ROS_INFO_STREAM(status_msg);
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
  }
}

void UpperBodyActionModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
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
        continue;
      else
      {
        result_it->second->goal_position_ = dxl.second->dxl_state_->goal_position_;
        action_result_[joint_name]->goal_position_ = dxl.second->dxl_state_->goal_position_;
      }
    }
    action_module_enabled_ = false;
  }

  processMotionStep();

  for (auto& action_enable : action_joints_enable_)
  {
    if (action_enable.second)
      result_[action_enable.first]->goal_position_ = action_result_[action_enable.first]->goal_position_;
  }

  previous_running_ = present_running_;
  present_running_ = isRunning();

  if (present_running_ != previous_running_)
  {
    if (present_running_)
    {
      std::string status_msg = "Action_Start";
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      for (auto& action_result : action_result_)
        action_result.second->goal_position_ = result_[action_result.first]->goal_position_;

      std::string status_msg = "Action_Finish";
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      publishDoneMsg("action");
    }
  }
}

void UpperBodyActionModule::loadYAMLFile(const std::string& file_name)
{
  YAML::Node config = YAML::LoadFile(file_name);

  joint_names_ = config["joint_names"].as<std::vector<std::string>>();
  for (const auto& point : config["points"])
  {
    positions_.emplace_back(point["positions"].as<std::vector<double>>());
    velocities_.emplace_back(point["velocities"].as<std::vector<double>>());
    accelerations_.emplace_back(point["accelerations"].as<std::vector<double>>());
    efforts_.emplace_back(point["effort"].as<std::vector<double>>());
    time_from_start_.emplace_back(point["time_from_start"].as<double>());
  }
}

void UpperBodyActionModule::processMotionStep()
{
  // Process each step of the motion based on loaded YAML data
  for (size_t i = 0; i < positions_.size(); ++i)
  {
    for (size_t j = 0; j < joint_names_.size(); ++j)
    {
      std::string joint_name = joint_names_[j];
      double position = positions_[i][j];
      action_result_[joint_name]->goal_position_ = position;
    }
  }
}

void UpperBodyActionModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type = type;
  status.module_name = "Action";
  status.status_msg = msg;

  status_msg_pub_.publish(status);
}

void UpperBodyActionModule::publishDoneMsg(std::string msg)
{
  std_msgs::String done_msg;
  done_msg.data = msg;

  done_msg_pub_.publish(done_msg);
}

bool UpperBodyActionModule::isRunning()
{
  return playing_;
}

void UpperBodyActionModule::brake()
{
  playing_ = false;
}
}  // namespace motion_control
