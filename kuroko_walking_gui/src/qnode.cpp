#include "../include/kuroko_walking_gui/qnode.hpp"

namespace walking_gui
{
QNodeKuroko::QNodeKuroko(int argc, char** argv) : init_argc_(argc), init_argv_(argv), body_height_(-1.0)
{
  // code to DEBUG
  debug_ = false;

  if (argc >= 2)
  {
    std::string arg_code(argv[1]);
    debug_ = arg_code == "debug";
  }
}

QNodeKuroko::~QNodeKuroko()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNodeKuroko::init()
{
  ros::init(init_argc_, init_argv_, "kuroko_walking_gui");

  if (!ros::master::check())
  {
    return false;
  }

  ros::start();  // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle ros_node;

  // Add your ros communications here.
  module_control_pub_ =
      ros_node.advertise<robotis_controller_msgs::JointCtrlModule>("/motion_control/set_joint_ctrl_modules", 0);
  module_control_preset_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/enable_ctrl_module", 0);
  init_pose_pub_ = ros_node.advertise<std_msgs::String>("/motion_control/base/ini_pose", 0);

  status_msg_sub_ = ros_node.subscribe("/motion_control/status", 10, &QNodeKuroko::statusMsgCallback, this);
  current_module_control_sub_ = ros_node.subscribe("/motion_control/present_joint_ctrl_modules", 10,
                                                   &QNodeKuroko::refreshCurrentJointControlCallback, this);

  get_module_control_client_ = ros_node.serviceClient<robotis_controller_msgs::GetJointModule>("/motion_control/"
                                                                                               "get_present_joint_ctrl_"
                                                                                               "modules");

  // For default demo
  initDefaultDemo(ros_node);

  // Preview
  initPreviewWalking(ros_node);

  // Config
  std::string default_config_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/gui_config.yaml";
  std::string config_path = ros_node.param<std::string>("gui_config", default_config_path);
  parseJointNameFromYaml(config_path);

  // start time
  start_time_ = ros::Time::now();

  tf_listener_.reset(new tf::TransformListener());

  // start qthread
  start();

  return true;
}

void QNodeKuroko::run()
{
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT
  rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNodeKuroko::parseJointNameFromYaml(const std::string& path)
{
  YAML::Node doc;
  ROS_INFO("QNodeKuroko::parseJointNameFromYaml - Loading: %s", path.c_str());
  try
  {
    // load yaml
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator it = id_sub_node.begin(); it != id_sub_node.end(); ++it)
  {
    int joint_id;
    std::string joint_name;

    joint_id = it->first.as<int>();
    joint_name = it->second.as<std::string>();

    id_joint_table_[joint_id] = joint_name;
    joint_id_table_[joint_name] = joint_id;

    if (debug_)
      std::cout << "ID : " << joint_id << " - " << joint_name << std::endl;
  }

  // parse module
  std::vector<std::string> modules = doc["module_list"].as<std::vector<std::string>>();

  int module_index = 0;
  for (auto module_name : modules)
  {
    index_mode_table_[module_index] = module_name;
    mode_index_table_[module_name] = module_index++;

    using_mode_table_[module_name] = false;
  }

  // parse module_joint preset
  YAML::Node sub_node = doc["module_button"];
  for (YAML::iterator yaml_it = sub_node.begin(); yaml_it != sub_node.end(); ++yaml_it)
  {
    int key_index;
    std::string module_name;

    key_index = yaml_it->first.as<int>();
    module_name = yaml_it->second.as<std::string>();

    module_table_[key_index] = module_name;
    if (debug_)
      std::cout << "Preset : " << module_name << std::endl;
  }
}

// joint id -> joint name
bool QNodeKuroko::getJointNameFromID(const int& id, std::string& joint_name)
{
  std::map<int, std::string>::iterator map_it;

  map_it = id_joint_table_.find(id);
  if (map_it == id_joint_table_.end())
    return false;

  joint_name = map_it->second;
  return true;
}

// joint name -> joint id
bool QNodeKuroko::getIDFromJointName(const std::string& joint_name, int& id)
{
  std::map<std::string, int>::iterator map_it;

  map_it = joint_id_table_.find(joint_name);
  if (map_it == joint_id_table_.end())
    return false;

  id = map_it->second;
  return true;
}

// map index -> joint id & joint name
bool QNodeKuroko::getIDJointNameFromIndex(const int& index, int& id, std::string& joint_name)
{
  std::map<int, std::string>::iterator map_it;
  int count = 0;
  for (map_it = id_joint_table_.begin(); map_it != id_joint_table_.end(); ++map_it, count++)
  {
    if (index == count)
    {
      id = map_it->first;
      joint_name = map_it->second;
      return true;
    }
  }
  return false;
}

// mode(module) index -> mode(module) name
std::string QNodeKuroko::getModeName(const int& index)
{
  std::string mode = "";
  std::map<int, std::string>::iterator map_it = index_mode_table_.find(index);

  if (map_it != index_mode_table_.end())
    mode = map_it->second;

  return mode;
}

// mode(module) name -> mode(module) index
int QNodeKuroko::getModeIndex(const std::string& mode_name)
{
  int mode_index = -1;
  std::map<std::string, int>::iterator map_it = mode_index_table_.find(mode_name);

  if (map_it != mode_index_table_.end())
    mode_index = map_it->second;

  return mode_index;
}

// number of mode(module)s
int QNodeKuroko::getModeSize()
{
  return index_mode_table_.size();
}

// number of joints
int QNodeKuroko::getJointSize()
{
  return id_joint_table_.size();
}

void QNodeKuroko::clearUsingModule()
{
  for (auto& map_it : using_mode_table_)
    map_it.second = false;
}

bool QNodeKuroko::isUsingModule(const std::string& module_name)
{
  std::map<std::string, bool>::iterator map_it = using_mode_table_.find(module_name);

  if (map_it == using_mode_table_.end())
    return false;

  return map_it->second;
}

// move ini pose : wholedody module
void QNodeKuroko::moveInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub_.publish(init_msg);

  log(INFO, "Go to robot initial pose.");
}

// set mode(module) to each joint
void QNodeKuroko::setJointControlMode(const robotis_controller_msgs::JointCtrlModule& msg)
{
  module_control_pub_.publish(msg);
}

void QNodeKuroko::setControlMode(const std::string& mode)
{
  std_msgs::String set_module_msg;
  set_module_msg.data = mode;

  module_control_preset_pub_.publish(set_module_msg);

  std::stringstream ss;
  ss << "Set Mode : " << mode;
  log(INFO, ss.str());
}

// get current mode(module) of joints
void QNodeKuroko::getJointControlMode()
{
  robotis_controller_msgs::GetJointModule get_joint;
  std::map<std::string, int> service_map;

  // _get_joint.request
  std::map<int, std::string>::iterator map_it;
  int index = 0;
  for (map_it = id_joint_table_.begin(); map_it != id_joint_table_.end(); ++map_it, index++)
  {
    get_joint.request.joint_name.push_back(map_it->second);
    service_map[map_it->second] = index;
  }

  if (get_module_control_client_.call(get_joint))
  {
    // _get_joint.response
    std::vector<int> modules;
    modules.resize(getJointSize());

    // clear current using modules
    clearUsingModule();

    for (int ix = 0; ix < get_joint.response.joint_name.size(); ix++)
    {
      std::string joint_name = get_joint.response.joint_name[ix];
      std::string module_name = get_joint.response.module_name[ix];

      std::map<std::string, int>::iterator service_it = service_map.find(joint_name);
      if (service_it == service_map.end())
        continue;

      index = service_it->second;

      service_it = mode_index_table_.find(module_name);
      if (service_it == mode_index_table_.end())
        continue;

      modules.at(index) = service_it->second;

      std::map<std::string, bool>::iterator module_it = using_mode_table_.find(module_name);
      if (module_it != using_mode_table_.end())
        module_it->second = true;
    }

    // update ui
    Q_EMIT updateCurrentJointControlMode(modules);
    log(INFO, "Get current Mode");
  }
  else
    log(ERROR, "Fail to get current joint control module");
}

void QNodeKuroko::refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr& msg)
{
  ROS_INFO("refreshCurrentJointControlCallback");
  std::vector<int> modules;
  modules.resize(getJointSize());
  std::map<std::string, int> joint_module_map;

  // clear current using modules
  clearUsingModule();

  for (int ix = 0; ix < msg->joint_name.size(); ix++)
  {
    std::string joint_name = msg->joint_name[ix];
    std::string module_name = msg->module_name[ix];

    joint_module_map[joint_name] = getModeIndex(module_name);

    std::map<std::string, bool>::iterator module_it = using_mode_table_.find(module_name);
    if (module_it != using_mode_table_.end())
      module_it->second = true;
  }

  for (int ix = 0; ix < getJointSize(); ix++)
  {
    int id = 0;
    std::string joint_name = "";

    if (!getIDJointNameFromIndex(ix, id, joint_name))
      continue;

    std::map<std::string, int>::iterator module_it = joint_module_map.find(joint_name);
    if (module_it == joint_module_map.end())
      continue;

    modules.at(ix) = module_it->second;
  }

  // update ui
  Q_EMIT updateCurrentJointControlMode(modules);

  log(INFO, "Applied Mode", "Manager");
}

// LOG
void QNodeKuroko::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)
{
  log((LogLevel)msg->type, msg->status_msg, msg->module_name);
}

void QNodeKuroko::log(const LogLevel& level, const std::string& msg, const std::string& sender)
{
  logging_model_.insertRows(logging_model_.rowCount(), 1);
  std::stringstream logging_model_msg;

  ros::Duration duration_time = ros::Time::now() - start_time_;
  int current_time = duration_time.sec;
  int min_time = 0, sec_time = 0;
  min_time = (int)(current_time / 60);
  sec_time = (int)(current_time % 60);

  std::stringstream min_str, sec_str;
  if (min_time < 10)
    min_str << "0";
  if (sec_time < 10)
    sec_str << "0";
  min_str << min_time;
  sec_str << sec_time;

  std::stringstream sender_ss;
  sender_ss << "[" << sender << "] ";

  switch (level)
  {
    case (DEBUG):
    {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (INFO):
    {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (WARN):
    {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[WARN] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (ERROR):
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "<ERROR> [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
    case (FATAL):
    {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << min_str.str() << ":" << sec_str.str() << "]: " << sender_ss.str() << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

void QNodeKuroko::clearLog()
{
  if (logging_model_.rowCount() == 0)
    return;

  logging_model_.removeRows(0, logging_model_.rowCount());
}

}  // namespace walking_gui
