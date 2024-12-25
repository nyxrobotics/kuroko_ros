#include <stdio.h>
#include "kuroko_initial_pose_module/initial_pose_module.h"
#include <utility>

namespace motion_control
{
InitialPoseModule::InitialPoseModule()
  : control_cycle_msec_(0), has_goal_joints_(false), ini_pose_only_(false), init_pose_file_path_("")
{
  enable_ = false;
  module_name_ = "initial_pose_module";
  control_mode_ = robotis_framework::PositionControl;

  initial_pose_module_state_ = new InitialPoseModuleState();
  joint_state_ = new BaseJointState();
  initial_pose_module_state_->is_moving_ = false;
}

InitialPoseModule::~InitialPoseModule()
{
  queue_thread_.join();
}

void InitialPoseModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = boost::thread(boost::bind(&InitialPoseModule::queueThread, this));

  // init result, joint_id_table
  for (auto& dxl : robot->dxls_)
  {
    std::string joint_name = dxl.first;
    robotis_framework::Dynamixel* dxl_info = dxl.second;

    joint_name_to_dxl_id_[joint_name] = dxl_info->id_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  ros::NodeHandle ros_node;

  /* Load ROS Parameter */
  ros_node.param<std::string>("init_pose_file_path", init_pose_file_path_,
                              ros::package::getPath("kuroko_initial_pose_module") + "/data/initial_pose.yaml");

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/motion_control/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/enable_ctrl_module", 1);
}

void InitialPoseModule::parseInitPoseData(const std::string& path)
{
  YAML::Node doc;
  ROS_INFO("InitialPoseModule - Loading: %s", path.c_str());
  try
  {
    // Load YAML file
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // Set default mov_time to 2.0 if not provided
  double mov_time = doc["mov_time"] ? doc["mov_time"].as<double>() : 2.0;
  initial_pose_module_state_->mov_time_ = mov_time;

  // Parse initial pose (joint positions)
  YAML::Node tar_pose_node = doc["initial_pose"];
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    std::string joint_name = yaml_it->first.as<std::string>();
    double value = yaml_it->second.as<double>();

    // Check if joint name exists in joint_name_to_dxl_id_ map
    if (joint_name_to_dxl_id_.find(joint_name) != joint_name_to_dxl_id_.end())
    {
      int id = joint_name_to_dxl_id_[joint_name];
      initial_pose_module_state_->joint_ini_pose_.coeffRef(id, 0) = value;
    }
    else
    {
      ROS_WARN("Joint name %s not found in joint_name_to_dxl_id_", joint_name.c_str());
    }
  }

  // Calculate total number of time steps
  initial_pose_module_state_->all_time_steps_ =
      int(initial_pose_module_state_->mov_time_ / initial_pose_module_state_->smp_time_) + 1;
  initial_pose_module_state_->calc_joint_tra_.resize(initial_pose_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
}

void InitialPoseModule::queueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscribe topics */
  ros::Subscriber ini_pose_msg_sub =
      ros_node.subscribe("/motion_control/base/ini_pose", 5, &InitialPoseModule::initPoseMsgCallback, this);
  set_module_client_ = ros_node.serviceClient<robotis_controller_msgs::SetModule>("/motion_control/"
                                                                                  "set_present_ctrl_modules");

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void InitialPoseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (!initial_pose_module_state_->is_moving_)
  {
    if (msg->data == "ini_pose")
    {
      // set module of all joints -> this module
      callServiceSettingModule(module_name_);

      // wait for changing the module to initial_pose_module and getting the
      // goal position
      while (!enable_ || !has_goal_joints_)
        usleep(8 * 1000);

      // parse initial pose
      parseInitPoseData(init_pose_file_path_);

      // generate trajectory
      tra_gene_tread_ = boost::thread(boost::bind(&InitialPoseModule::initPoseTrajGenerateProc, this));
    }
  }
  else
    ROS_INFO("[InitialPoseModule] previous task is alive");

  return;
}

void InitialPoseModule::initPoseTrajGenerateProc()
{
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = initial_pose_module_state_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra;

    if (initial_pose_module_state_->via_num_ == 0)
    {
      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                  initial_pose_module_state_->smp_time_,
                                                  initial_pose_module_state_->mov_time_);
    }
    else
    {
      Eigen::MatrixXd via_value = initial_pose_module_state_->joint_via_pose_.col(id);
      Eigen::MatrixXd d_via_value = initial_pose_module_state_->joint_via_dpose_.col(id);
      Eigen::MatrixXd dd_via_value = initial_pose_module_state_->joint_via_ddpose_.col(id);

      tra = robotis_framework::calcMinimumJerkTraWithViaPoints(initial_pose_module_state_->via_num_, ini_value, 0.0,
                                                               0.0, via_value, d_via_value, dd_via_value, tar_value,
                                                               0.0, 0.0, initial_pose_module_state_->smp_time_,
                                                               initial_pose_module_state_->via_time_,
                                                               initial_pose_module_state_->mov_time_);
    }

    initial_pose_module_state_->calc_joint_tra_.block(0, id, initial_pose_module_state_->all_time_steps_, 1) = tra;
  }

  initial_pose_module_state_->is_moving_ = true;
  initial_pose_module_state_->cnt_ = 0;
  ROS_INFO("[start] send trajectory");
}

void InitialPoseModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (!enable_ || !has_goal_joints_)
    usleep(8 * 1000);

  initial_pose_module_state_->mov_time_ = 5.0;
  initial_pose_module_state_->all_time_steps_ =
      int(initial_pose_module_state_->mov_time_ / initial_pose_module_state_->smp_time_) + 1;

  initial_pose_module_state_->calc_joint_tra_.resize(initial_pose_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  initial_pose_module_state_->joint_pose_ = std::move(joint_angle_pose);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = initial_pose_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                initial_pose_module_state_->smp_time_,
                                                                initial_pose_module_state_->mov_time_);

    initial_pose_module_state_->calc_joint_tra_.block(0, id, initial_pose_module_state_->all_time_steps_, 1) = tra;
  }

  initial_pose_module_state_->is_moving_ = true;
  initial_pose_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

void InitialPoseModule::poseGenerateProc(std::map<std::string, double>& joint_angle_pose)
{
  callServiceSettingModule(module_name_);

  while (!enable_ || !has_goal_joints_)
    usleep(8 * 1000);

  Eigen::MatrixXd target_pose = Eigen::MatrixXd::Zero(MAX_JOINT_ID + 1, 1);

  for (auto& joint_angle_it : joint_angle_pose)
  {
    std::string joint_name = joint_angle_it.first;
    double joint_angle_rad = joint_angle_it.second;

    std::map<std::string, int>::iterator joint_name_to_dxl_id_it = joint_name_to_dxl_id_.find(joint_name);
    if (joint_name_to_dxl_id_it != joint_name_to_dxl_id_.end())
    {
      target_pose.coeffRef(joint_name_to_dxl_id_it->second, 0) = joint_angle_rad;
    }
  }

  initial_pose_module_state_->joint_pose_ = target_pose;

  initial_pose_module_state_->mov_time_ = 5.0;
  initial_pose_module_state_->all_time_steps_ =
      int(initial_pose_module_state_->mov_time_ / initial_pose_module_state_->smp_time_) + 1;

  initial_pose_module_state_->calc_joint_tra_.resize(initial_pose_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = initial_pose_module_state_->joint_pose_.coeff(id, 0);

    ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                initial_pose_module_state_->smp_time_,
                                                                initial_pose_module_state_->mov_time_);

    initial_pose_module_state_->calc_joint_tra_.block(0, id, initial_pose_module_state_->all_time_steps_, 1) = tra;
  }

  initial_pose_module_state_->is_moving_ = true;
  initial_pose_module_state_->cnt_ = 0;
  ini_pose_only_ = true;
  ROS_INFO("[start] send trajectory");
}

bool InitialPoseModule::isRunning()
{
  return initial_pose_module_state_->is_moving_;
}

void InitialPoseModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                                std::map<std::string, double> /*sensors*/)
{
  if (!enable_)
    return;

  /*----- write curr position -----*/
  for (auto& state_iter : result_)
  {
    std::string joint_name = state_iter.first;

    robotis_framework::Dynamixel* dxl = nullptr;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    joint_state_->curr_joint_state_[joint_name_to_dxl_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_dxl_id_[joint_name]].position_ = joint_goal_position;
  }

  has_goal_joints_ = true;

  /* ----- send trajectory ----- */
  if (initial_pose_module_state_->is_moving_)
  {
    if (initial_pose_module_state_->cnt_ == 1)
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");

    for (int id = 1; id <= MAX_JOINT_ID; id++)
      joint_state_->goal_joint_state_[id].position_ =
          initial_pose_module_state_->calc_joint_tra_(initial_pose_module_state_->cnt_, id);

    initial_pose_module_state_->cnt_++;
  }

  /*----- set joint data -----*/
  for (auto& state_iter : result_)
  {
    std::string joint_name = state_iter.first;

    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_dxl_id_[joint_name]].position_;
  }

  /*---------- initialize count number ----------*/

  if ((initial_pose_module_state_->cnt_ >= initial_pose_module_state_->all_time_steps_) &&
      (initial_pose_module_state_->is_moving_))
  {
    ROS_INFO("[end] send trajectory");

    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

    initial_pose_module_state_->is_moving_ = false;
    initial_pose_module_state_->cnt_ = 0;

    // set all joints -> none
    if (ini_pose_only_)
    {
      setCtrlModule("none");
      ini_pose_only_ = false;
    }
  }
}

void InitialPoseModule::stop()
{
  return;
}

void InitialPoseModule::onModuleEnable()
{
  ROS_INFO("[InitialPoseModule] Module Enabled");
}

void InitialPoseModule::onModuleDisable()
{
  ROS_INFO("[InitialPoseModule] Module Disabled");
  has_goal_joints_ = false;
}

void InitialPoseModule::setCtrlModule(const std::string& /*module*/)
{
  std_msgs::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_.publish(control_msg);
}

void InitialPoseModule::callServiceSettingModule(const std::string& module_name)
{
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (!set_module_client_.call(set_module_srv))
  {
    ROS_ERROR("Failed to set module");
    return;
  }

  return;
}

void InitialPoseModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "InitialPoseModule";
  status_msg.status_msg = std::move(msg);

  status_msg_pub_.publish(status_msg);
}
}  // namespace motion_control
