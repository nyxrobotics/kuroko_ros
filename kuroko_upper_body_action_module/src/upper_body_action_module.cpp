#include "kuroko_upper_body_action_module/upper_body_action_module.h"

namespace motion_control
{
std::string UpperBodyActionModule::convertIntToString(int n)
{
  std::ostringstream ostr;
  ostr << n;
  return ostr.str();
}

UpperBodyActionModule::UpperBodyActionModule()
  : control_cycle_msec_(8)
  , enable_(false)
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
  /////////////// Const Variable
  /**************************************
   * Section             /----\
   *                    /|    |\
   *        /+---------/ |    | \
   *       / |        |  |    |  \
   * -----/  |        |  |    |   \----
   *      PRE  MAIN   PRE MAIN POST PAUSE
   ***************************************/

  action_file_ = 0;
}

UpperBodyActionModule::~UpperBodyActionModule()
{
  queue_thread_.join();

  if (action_file_ != 0)
    fclose(action_file_);
}

void UpperBodyActionModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  control_cycle_msec_ = control_cycle_msec;
  robot_ = robot;
  queue_thread_ = boost::thread(boost::bind(&UpperBodyActionModule::queueThread, this));

  for (auto it = robot->dxls_.begin(); it != robot->dxls_.end(); it++)
  {
    std::string joint_name = it->first;
    robotis_framework::Dynamixel* dxl_info = it->second;

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

  if (!loadMotionFromYAML(action_file_path))
  {
    ROS_ERROR("Failed to load motion data from YAML file.");
  }

  playing_ = false;
}

bool UpperBodyActionModule::loadMotionFromYAML(const std::string& yaml_file)
{
  return parseYAMLFile(yaml_file, motion_data_);
}

bool UpperBodyActionModule::parseYAMLFile(const std::string& yaml_file, trajectory_msgs::JointTrajectory& motion_data)
{
  try
  {
    YAML::Node config = YAML::LoadFile(yaml_file);

    motion_data.joint_names = config["joint_names"].as<std::vector<std::string>>();

    for (const auto& point : config["points"])
    {
      trajectory_msgs::JointTrajectoryPoint traj_point;

      traj_point.positions = point["positions"].as<std::vector<double>>();
      traj_point.velocities = point["velocities"].as<std::vector<double>>();
      traj_point.accelerations = point["accelerations"].as<std::vector<double>>();
      traj_point.effort = point["effort"].as<std::vector<double>>();
      traj_point.time_from_start = ros::Duration(point["time_from_start"].as<double>());

      motion_data.points.push_back(traj_point);
    }
    return true;
  }
  catch (YAML::Exception& e)
  {
    ROS_ERROR("Failed to load YAML file: %s", e.what());
    return false;
  }
}

void UpperBodyActionModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/motion_control/status", 0);
  done_msg_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/movement_done", 1);

  ros::Subscriber action_page_sub =
      ros_node.subscribe("/motion_control/action/page_num", 0, &UpperBodyActionModule::pageNumberCallback, this);
  ros::Subscriber start_action_sub =
      ros_node.subscribe("/motion_control/action/start_action", 0, &UpperBodyActionModule::startActionCallback, this);

  ros::ServiceServer is_running_server = ros_node.advertiseService(
      "/motion_control/action/is_running", &UpperBodyActionModule::isRunningServiceCallback, this);

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
  if (enable_ == false)
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
    for (auto& joints_enable_it : action_joints_enable_)
      joints_enable_it.second = true;

    if (start(msg->data) == true)
    {
      std::string status_msg = "Succeed to start page " + convertIntToString(msg->data);
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      std::string status_msg = "Failed to start page " + convertIntToString(msg->data);
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      publishDoneMsg("action_failed");
    }
  }
}

void UpperBodyActionModule::startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg)
{
  if (enable_ == false)
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
    for (auto& joints_enable_it : action_joints_enable_)
      joints_enable_it.second = false;

    int joint_name_array_size = msg->joint_name_array.size();
    for (int joint_idx = 0; joint_idx < joint_name_array_size; joint_idx++)
    {
      auto joints_enable_it = action_joints_enable_.find(msg->joint_name_array[joint_idx]);
      if (joints_enable_it == action_joints_enable_.end())
      {
        std::string status_msg = "Invalid Joint Name : " + msg->joint_name_array[joint_idx];
        ROS_INFO_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        publishDoneMsg("action_failed");
        return;
      }
      else
      {
        joints_enable_it->second = true;
      }
    }

    if (start(msg->page_num) == true)
    {
      std::string status_msg = "Succeed to start page " + convertIntToString(msg->page_num);
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      std::string status_msg = "Failed to start page " + convertIntToString(msg->page_num);
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      publishDoneMsg("action_failed");
    }
  }
}

void UpperBodyActionModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                                    std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  if (action_module_enabled_ == true)
  {
    for (auto& dxls_it : dxls)
    {
      std::string joint_name = dxls_it.first;

      auto result_it = result_.find(joint_name);
      if (result_it == result_.end())
        continue;
      else
      {
        result_it->second->goal_position_ = dxls_it.second->dxl_state_->goal_position_;
        action_result_[joint_name]->goal_position_ = dxls_it.second->dxl_state_->goal_position_;
      }
    }
    action_module_enabled_ = false;
  }

  actionPlayProcess(dxls);

  for (auto& action_enable_it : action_joints_enable_)
  {
    if (action_enable_it.second == true)
      result_[action_enable_it.first]->goal_position_ = action_result_[action_enable_it.first]->goal_position_;
  }

  previous_running_ = present_running_;
  present_running_ = isRunning();

  if (present_running_ != previous_running_)
  {
    if (present_running_ == true)
    {
      std::string status_msg = "Action_Start";
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      for (auto& action_result_it : action_result_)
        action_result_it.second->goal_position_ = result_[action_result_it.first]->goal_position_;

      std::string status_msg = "Action_Finish";
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      publishDoneMsg("action");
    }
  }
}

void UpperBodyActionModule::stop()
{
  stop_playing_ = true;
}

bool UpperBodyActionModule::isRunning()
{
  return playing_;
}

void UpperBodyActionModule::onModuleEnable()
{
  action_module_enabled_ = true;
}

void UpperBodyActionModule::onModuleDisable()
{
  action_module_enabled_ = false;
  brake();
}

void UpperBodyActionModule::actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel*> dxls)
{
  if (playing_ == false)
  {
    for (auto& dxls_it : dxls)
    {
      std::string joint_name = dxls_it.first;

      auto result_it = action_result_.find(joint_name);
      if (result_it == result_.end())
        continue;
      else
      {
        result_it->second->goal_position_ = dxls_it.second->dxl_state_->goal_position_;
      }
    }
    return;
  }

  // 再生処理部分はそのままの処理を維持

  // 以下のコードを追加し、motion_data_を使用して再生処理を行う
  for (const auto& traj_point : motion_data_.points)
  {
    for (size_t i = 0; i < motion_data_.joint_names.size(); ++i)
    {
      std::string joint_name = motion_data_.joint_names[i];
      auto dxls_it = dxls.find(joint_name);
      if (dxls_it == dxls.end())
        continue;

      action_result_[joint_name]->goal_position_ = traj_point.positions[i];
    }
    ros::Duration(traj_point.time_from_start).sleep();  // モーションの各ステップ間での待機時間を調整
  }

  playing_ = false;
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
}  // namespace motion_control
