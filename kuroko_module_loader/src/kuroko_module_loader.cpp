#include "kuroko_module_loader.h"
#include <unistd.h>

KurokoModuleLoader::KurokoModuleLoader(ros::NodeHandle& nh)
  : nh_(nh), controller_(KurokoJointController::getInstance()), port_handler_(nullptr)
{
  loadParameters(nh_);
  setupROS(nh_);
  setupController();
}

KurokoModuleLoader::~KurokoModuleLoader()
{
  if (port_handler_)
  {
    port_handler_->closePort();
    delete port_handler_;
  }
}

void KurokoModuleLoader::initialize()
{
  if (!controller_->gazebo_mode_)
  {
    port_handler_ = PortHandler::getPortHandler(device_name_.c_str());
    bool set_port_result = port_handler_->setBaudRate(baudrate_);
    if (!set_port_result)
    {
      ROS_ERROR("Error Set port");
      return;
    }
    PacketHandler* packet_handler = PacketHandler::getPacketHandler(protocol_version_);

    uint8_t torque = 0;
    packet_handler->read1ByteTxRx(port_handler_, default_dxl_id_, torque_on_ctrl_table_, &torque);

    if (torque != 1)
    {
      controller_->initializeDevice(init_file_);
    }
    else
    {
      ROS_INFO("Torque is already on!!");
    }
  }

  controller_->startTimer();
  usleep(200 * 1000);

  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub_.publish(init_msg);
  ROS_INFO("Go to init pose");
}

void KurokoModuleLoader::start()
{
  ros::spin();
}

void KurokoModuleLoader::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "user_long")
  {
    controller_->setCtrlModule("none");
    controller_->stopTimer();

    if (!controller_->gazebo_mode_)
    {
      if (!port_handler_)
      {
        port_handler_ = PortHandler::getPortHandler(device_name_.c_str());
        port_handler_->setBaudRate(baudrate_);
      }
      PacketHandler* packet_handler = PacketHandler::getPacketHandler(protocol_version_);

      uint8_t torque = 0;
      packet_handler->read1ByteTxRx(port_handler_, default_dxl_id_, torque_on_ctrl_table_, &torque);

      if (torque != 1)
      {
        controller_->initializeDevice(init_file_);
      }
      else
      {
        ROS_INFO("Torque is already on!!");
      }
    }

    controller_->startTimer();
    usleep(200 * 1000);

    std_msgs::String init_msg;
    init_msg.data = "ini_pose";
    init_pose_pub_.publish(init_msg);
    ROS_INFO("Go to init pose");
  }
}

void KurokoModuleLoader::dxlTorqueCheckCallback(const std_msgs::String::ConstPtr& /*msg*/)
{
  if (controller_->gazebo_mode_)
    return;

  uint8_t torque_result = 0;
  bool torque_on = true;

  for (auto& map_it : controller_->robot_->port_default_device_)
  {
    std::string default_device_name = map_it.second;
    controller_->read1Byte(default_device_name, torque_on_ctrl_table_, &torque_result);

    if (torque_result != 1)
      torque_on = false;
  }

  if (!torque_on)
  {
    controller_->stopTimer();
    controller_->initializeDevice(init_file_);
    controller_->startTimer();
  }
}

void KurokoModuleLoader::loadParameters(ros::NodeHandle& nh)
{
  protocol_version_ = 2.0;
  dxl_broadcast_id_ = 254;
  default_dxl_id_ = 1;
  power_ctrl_table_ = 24;
  rgb_led_ctrl_table_ = 26;
  torque_on_ctrl_table_ = 64;

  nh.param<std::string>("offset_file_path", offset_file_, "");
  nh.param<std::string>("robot_file_path", robot_file_, "");
  nh.param<std::string>("init_file_path", init_file_, "");
  nh.param<std::string>("device_name", device_name_, "/dev/ttyUSB0");
  nh.param<int>("baud_rate", baudrate_, 2000000);
  nh.param<bool>("is_gazebo", controller_->gazebo_mode_, false);
}

void KurokoModuleLoader::setupROS(ros::NodeHandle& nh)
{
  button_sub_ = nh.subscribe("/motion_control/open_cr/button", 1, &KurokoModuleLoader::buttonHandlerCallback, this);
  dxl_torque_sub_ = nh.subscribe("/motion_control/dxl_torque", 1, &KurokoModuleLoader::dxlTorqueCheckCallback, this);
  init_pose_pub_ = nh.advertise<std_msgs::String>("/motion_control/base/ini_pose", 0);
  demo_command_pub_ = nh.advertise<std_msgs::String>("/ball_tracker/command", 0);
}

void KurokoModuleLoader::setupController()
{
  if (!controller_->gazebo_mode_)
  {
    port_handler_ = PortHandler::getPortHandler(device_name_.c_str());
    bool set_port_result = port_handler_->setBaudRate(baudrate_);
    if (!set_port_result)
      ROS_ERROR("Error Set port");

    PacketHandler* packet_handler = PacketHandler::getPacketHandler(protocol_version_);

    int torque_on_count = 0;
    while (torque_on_count < 5)
    {
      int err_status = packet_handler->write1ByteTxRx(port_handler_, dxl_broadcast_id_, power_ctrl_table_, 1);
      if (err_status != 0)
        ROS_ERROR("Torque on DXLs! [%s]", packet_handler->getRxPacketError(err_status));
      else
        ROS_INFO("Torque on DXLs!");
      if (err_status == 0)
        break;
      else
        torque_on_count++;
    }

    usleep(100 * 1000);

    int led_full_unit = 0x1F;
    int led_range = 5;
    int led_value = led_full_unit << led_range;
    int err_status = packet_handler->write2ByteTxRx(port_handler_, dxl_broadcast_id_, rgb_led_ctrl_table_, led_value);

    if (err_status != 0)
      ROS_ERROR("Fail to control LED [%s]", packet_handler->getRxPacketError(err_status));
  }
  else
  {
    ROS_WARN("SET TO GAZEBO MODE!");
    std::string robot_name;
    nh_.param<std::string>("gazebo_robot_name", robot_name, "");
    if (!robot_name.empty())
      controller_->gazebo_robot_name_ = robot_name;
  }

  if (robot_file_.empty())
  {
    ROS_ERROR("NO robot file path in the ROS parameters.");
    throw std::runtime_error("NO robot file path");
  }

  if (!controller_->initialize(robot_file_, init_file_))
  {
    ROS_ERROR("ROBOTIS Controller Initialize Fail!");
    throw std::runtime_error("ROBOTIS Controller Initialize Fail");
  }

  if (!offset_file_.empty() && !controller_->gazebo_mode_)
    controller_->loadOffset(offset_file_);

  usleep(300 * 1000);

  if (!controller_->gazebo_mode_)
  {
    // controller_->addSensorModule((SensorModule*)KurokoImuReceiver::getInstance());
  }
  controller_->addSensorModule((SensorModule*)KurokoImuReceiver::getInstance());

  controller_->addMotionModule((MotionModule*)InitialPoseModule::getInstance());
  controller_->addMotionModule((MotionModule*)WalkingModule::getInstance());
  controller_->addMotionModule((MotionModule*)ActionModule::getInstance());
  // controller_->addMotionModule((MotionModule*)HeadControlModule::getInstance());
  // controller_->addMotionModule((MotionModule*)DirectControlModule::getInstance());
  // controller_->addMotionModule((MotionModule*)OnlineWalkingModule::getInstance());
  // controller_->addMotionModule((MotionModule*)TuningModule::getInstance());

  controller_->startTimer();
  usleep(100 * 1000);

  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub_.publish(init_msg);
  ROS_INFO("Go to init pose");
}
