#include "../include/kuroko_walking_gui/qnode.hpp"

namespace walking_gui
{
void QNodeKuroko::initDefaultDemo(ros::NodeHandle& ros_node)
{
  init_gyro_pub_ = ros_node.advertise<robotis_controller_msgs::SyncWriteItem>("/motion_control/sync_write_item", 0);
  set_head_joint_angle_pub_ =
      ros_node.advertise<sensor_msgs::JointState>("/motion_control/head_control/set_joint_states", 0);

  current_joint_states_sub_ =
      ros_node.subscribe("/motion_control/present_joint_states", 10, &QNodeKuroko::updateHeadJointStatesCallback, this);

  // Walking
  set_walking_command_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/walking/command", 0);
  set_walking_param_pub_ =
      ros_node.advertise<op3_walking_module_msgs::WalkingParam>("/motion_control/walking/set_params", 0);
  get_walking_param_client_ = ros_node.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/motion_control/"
                                                                                               "walking/get_params");

  // Action
  motion_index_pub_ = ros_node.advertise<std_msgs::Int32>("/motion_control/action/animation_num", 0);

  // Demo
  demo_command_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/demo_command", 0);

  std::string default_motion_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/gui_motion.yaml";
  std::string motion_path = ros_node.param<std::string>("gui_motion", default_motion_path);
  parseMotionMapFromYaml(motion_path);

  ROS_INFO("Initialized node handle for default demo");
}

void QNodeKuroko::updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  double head_pan, head_tilt;
  int num_get = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      head_pan = -msg->position[ix];
      num_get += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      head_tilt = msg->position[ix];
      num_get += 1;
    }

    if (num_get == 2)
      break;
  }

  if (num_get > 0)
    Q_EMIT updateHeadAngles(head_pan, head_tilt);
}

void QNodeKuroko::setHeadJoint(double pan, double tilt)
{
  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(-pan);
  head_angle_msg.position.push_back(tilt);

  set_head_joint_angle_pub_.publish(head_angle_msg);
}

// Walking
void QNodeKuroko::setWalkingCommand(const std::string& command)
{
  std_msgs::String commnd_msg;
  commnd_msg.data = command;
  set_walking_command_pub_.publish(commnd_msg);

  std::stringstream ss_log;
  ss_log << "Set Walking Command: " << commnd_msg.data << std::endl;

  log(INFO, ss_log.str());
}

void QNodeKuroko::refreshWalkingParam()
{
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if (get_walking_param_client_.call(walking_param_msg))
  {
    walking_param_ = walking_param_msg.response.parameters;

    // update ui
    Q_EMIT updateWalkingParameters(walking_param_);
    log(INFO, "Get Walking Parameters");
  }
  else
    log(ERROR, "Fail to Get Walking Parameters");
}

void QNodeKuroko::saveWalkingParam()
{
  std_msgs::String command_msg;
  command_msg.data = "save";
  set_walking_command_pub_.publish(command_msg);

  log(INFO, "Save Walking Parameters");
}

void QNodeKuroko::applyWalkingParam(const op3_walking_module_msgs::WalkingParam& walking_param)
{
  walking_param_ = walking_param;

  set_walking_param_pub_.publish(walking_param_);
  log(INFO, "Apply Walking Parameters");
}

void QNodeKuroko::initGyro()
{
  robotis_controller_msgs::SyncWriteItem init_gyro_msg;
  init_gyro_msg.item_name = "imu_control";
  init_gyro_msg.joint_name.push_back("kuroko_imu");
  init_gyro_msg.value.push_back(0x08);

  init_gyro_pub_.publish(init_gyro_msg);

  log(INFO, "Initialize Gyro");
}

// Motion
void QNodeKuroko::playMotion(int motion_index)
{
  if (motion_table_.find(motion_index) == motion_table_.end())
  {
    log(ERROR, "Motion index is not valid.");
    return;
  }

  // Show Motion Name
  std::stringstream log_ss;
  std::string motion_name = motion_table_[motion_index];
  log_ss << "Play Motion: [" << motion_index << "] " << motion_name;

  // Publish motion index
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;
  motion_index_pub_.publish(motion_msg);

  log(INFO, log_ss.str());
}

// Demo
void QNodeKuroko::setDemoCommand(const std::string& command)
{
  std_msgs::String demo_msg;
  demo_msg.data = command;

  demo_command_pub_.publish(demo_msg);

  std::stringstream log_ss;
  log_ss << "Demo command : " << command;
  log(INFO, log_ss.str());
}

void QNodeKuroko::setActionModuleBody()
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string module_name = "action_module";

  for (int ix = 1; ix <= 18; ix++)
  {
    std::string joint_name;

    if (!getJointNameFromID(ix, joint_name))
      continue;

    control_msg.joint_name.push_back(joint_name);
    control_msg.module_name.push_back(module_name);
  }

  // no control
  if (control_msg.joint_name.empty())
    return;

  setJointControlMode(control_msg);
}

void QNodeKuroko::setModuleToDemo()
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string body_module = "walking_module";
  std::string head_module = "head_control_module";

  for (int ix = 1; ix <= 20; ix++)
  {
    std::string joint_name;

    if (!getJointNameFromID(ix, joint_name))
      continue;

    control_msg.joint_name.push_back(joint_name);
    if (ix <= 18)
      control_msg.module_name.push_back(body_module);
    else
      control_msg.module_name.push_back(head_module);
  }

  // no control
  if (control_msg.joint_name.empty())
    return;

  setJointControlMode(control_msg);
}

void QNodeKuroko::parseMotionMapFromYaml(const std::string& path)
{
  YAML::Node doc;
  ROS_INFO("QNodeKuroko::parseMotionMapFromYaml - Loading : %s", path.c_str());
  try
  {
    // load yaml
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load motion yaml.");
    return;
  }

  // parse motion_table
  YAML::Node motion_sub_node = doc["motion"];
  for (YAML::iterator yaml_it = motion_sub_node.begin(); yaml_it != motion_sub_node.end(); ++yaml_it)
  {
    int motion_index;
    std::string motion_name;

    motion_index = yaml_it->first.as<int>();
    motion_name = yaml_it->second.as<std::string>();

    motion_table_[motion_index] = motion_name;
  }

  // parse shortcut_table
  YAML::Node shoutcut_sub_node = doc["motion_shortcut"];
  for (YAML::iterator it = shoutcut_sub_node.begin(); it != shoutcut_sub_node.end(); ++it)
  {
    int shortcut_prefix = 0x30;
    int motion_index;
    int shortcut_index;

    motion_index = it->first.as<int>();
    shortcut_index = it->second.as<int>();

    if (shortcut_index < 0 || shortcut_index > 9)
      continue;

    motion_shortcut_table_[motion_index] = shortcut_index + shortcut_prefix;
  }
}

}  // namespace walking_gui
