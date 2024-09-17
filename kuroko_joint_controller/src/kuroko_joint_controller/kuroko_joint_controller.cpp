#include <ros/callback_queue.h>
#include <ros/package.h>

#include <utility>
#include "ros/console.h"

#include "kuroko_joint_controller/kuroko_joint_controller.h"

using namespace robotis_framework;

KurokoJointController::KurokoJointController()
  : is_timer_running_(false)
  , is_offset_enabled_(true)
  , offset_ratio_(1.0)
  , stop_timer_(false)
  , init_pose_loaded_(false)
  , timer_thread_(0)
  , controller_mode_(MOTION_MODULE_MODE)
  , debug_print_(true)
  , robot_(nullptr)
  , gazebo_mode_(false)
  , gazebo_robot_name_("kuroko")
{
  direct_sync_write_.clear();
  ROS_INFO("[KurokoJointController] Initialized");
}

void KurokoJointController::initializeSyncWrite()
{
  if (gazebo_mode_)
    return;

  ROS_INFO("[KurokoJointController::initializeSyncWrite] FIRST BULKREAD");
  for (auto& it : port_to_bulk_read_)
  {
    if (it.second != NULL)
    {
      it.second->txRxPacket();
    }
  }
  for (auto& it : port_to_bulk_read_)
  {
    if (it.second != NULL)
    {
      int error_count = 0;
      int result = COMM_SUCCESS;
      do
      {
        if (++error_count > 10)
        {
          ROS_ERROR("[KurokoJointController::initializeSyncWrite] First bulk read failed!!");
          exit(-1);
        }
        usleep(8 * 1000);
        result = it.second->txRxPacket();
      } while (result != COMM_SUCCESS);
    }
  }
  init_pose_loaded_ = true;
  ROS_INFO("[KurokoJointController::initializeSyncWrite] FIRST BULKREAD END");

  // clear syncwrite param setting
  for (auto& it : port_to_sync_write_position_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_p_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_i_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_position_d_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_p_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_i_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_velocity_d_gain_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }
  for (auto& it : port_to_sync_write_current_)
  {
    if (it.second != NULL)
      it.second->clearParam();
  }

  ROS_INFO("[KurokoJointController::initializeSyncWrite] SyncWrite Params Cleared");
  // set init syncwrite param(from data of bulkread)
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name = it.first;
    Dynamixel* dxl = it.second;

    for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
    {
      uint32_t read_data = 0;
      uint8_t sync_write_data[4];

      if (port_to_bulk_read_[dxl->port_name_]->isAvailable(dxl->id_, dxl->bulk_read_items_[i]->address_,
                                                           dxl->bulk_read_items_[i]->data_length_))
      {
        read_data = port_to_bulk_read_[dxl->port_name_]->getData(dxl->id_, dxl->bulk_read_items_[i]->address_,
                                                                 dxl->bulk_read_items_[i]->data_length_);

        sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(read_data));
        sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(read_data));
        sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(read_data));
        sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(read_data));

        if ((dxl->present_position_item_ != nullptr) &&
            (dxl->bulk_read_items_[i]->item_name_ == dxl->present_position_item_->item_name_))
        {
          dxl->dxl_state_->present_position_ =
              dxl->convertValue2Radian(read_data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
          dxl->dxl_state_->goal_position_ = dxl->dxl_state_->present_position_;

          port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
        }
        else if ((dxl->position_p_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_p_gain_ = read_data;
        }
        else if ((dxl->position_i_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_i_gain_ = read_data;
        }
        else if ((dxl->position_d_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->position_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->position_d_gain_ = read_data;
        }
        else if ((dxl->present_velocity_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_velocity_item_->item_name_))
        {
          dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(read_data);
          dxl->dxl_state_->goal_velocity_ = dxl->dxl_state_->present_velocity_;
        }
        else if ((dxl->velocity_p_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_p_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_p_gain_ = read_data;
        }
        else if ((dxl->velocity_i_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_i_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_i_gain_ = read_data;
        }
        else if ((dxl->velocity_d_gain_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->velocity_d_gain_item_->item_name_))
        {
          dxl->dxl_state_->velocity_d_gain_ = read_data;
        }
        else if ((dxl->present_current_item_ != nullptr) &&
                 (dxl->bulk_read_items_[i]->item_name_ == dxl->present_current_item_->item_name_))
        {
          dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(read_data);
          dxl->dxl_state_->goal_torque_ = dxl->dxl_state_->present_torque_;
        }
      }
    }
  }
}

bool KurokoJointController::initialize(const std::string& robot_file_path, const std::string& init_file_path)
{
  std::string dev_desc_dir_path = ros::package::getPath("robotis_device") + "/devices";

  // load robot info : port , device
  robot_ = new Robot(robot_file_path, dev_desc_dir_path);

  if (gazebo_mode_)
  {
    queue_thread_ = boost::thread(boost::bind(&KurokoJointController::msgQueueThread, this));
    ROS_INFO("[KurokoJointController::initialize] Running in gazebo mode");
    return true;
  }

  for (auto& it : robot_->ports_)
  {
    std::string port_name = it.first;
    dynamixel::PortHandler* port = it.second;
    dynamixel::PacketHandler* default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!port->setBaudRate(port->getBaudRate()))
    {
      ROS_ERROR("[KurokoJointController::initialize] PORT [%s] setup error (baudrate: %d)", port_name.c_str(),
                port->getBaudRate());
      exit(-1);
    }

    // get the default device info of the port
    std::string default_device_name = robot_->port_default_device_[port_name];
    auto dxl_it = robot_->dxls_.find(default_device_name);
    auto sensor_it = robot_->sensors_.find(default_device_name);
    if (dxl_it != robot_->dxls_.end())
    {
      Dynamixel* default_device = dxl_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(default_device->protocol_version_);

      if (default_device->goal_position_item_ != nullptr)
      {
        port_to_sync_write_position_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->goal_position_item_->address_,
                                          default_device->goal_position_item_->data_length_);
      }

      if (default_device->position_p_gain_item_ != nullptr)
      {
        port_to_sync_write_position_p_gain_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->position_p_gain_item_->address_,
                                          default_device->position_p_gain_item_->data_length_);
      }

      if (default_device->position_i_gain_item_ != nullptr)
      {
        port_to_sync_write_position_i_gain_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->position_i_gain_item_->address_,
                                          default_device->position_i_gain_item_->data_length_);
      }

      if (default_device->position_d_gain_item_ != nullptr)
      {
        port_to_sync_write_position_d_gain_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->position_d_gain_item_->address_,
                                          default_device->position_d_gain_item_->data_length_);
      }

      if (default_device->goal_velocity_item_ != nullptr)
      {
        port_to_sync_write_velocity_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->goal_velocity_item_->address_,
                                          default_device->goal_velocity_item_->data_length_);
      }

      if (default_device->velocity_p_gain_item_ != nullptr)
      {
        port_to_sync_write_velocity_p_gain_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->velocity_p_gain_item_->address_,
                                          default_device->velocity_p_gain_item_->data_length_);
      }

      if (default_device->velocity_i_gain_item_ != nullptr)
      {
        port_to_sync_write_velocity_i_gain_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->velocity_i_gain_item_->address_,
                                          default_device->velocity_i_gain_item_->data_length_);
      }

      if (default_device->velocity_d_gain_item_ != nullptr)
      {
        port_to_sync_write_velocity_d_gain_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->velocity_d_gain_item_->address_,
                                          default_device->velocity_d_gain_item_->data_length_);
      }

      if (default_device->goal_current_item_ != nullptr)
      {
        port_to_sync_write_current_[port_name] =
            new dynamixel::GroupSyncWrite(port, default_pkt_handler, default_device->goal_current_item_->address_,
                                          default_device->goal_current_item_->data_length_);
      }
    }
    else if (sensor_it != robot_->sensors_.end())
    {
      Sensor* default_device = sensor_it->second;
      default_pkt_handler = dynamixel::PacketHandler::getPacketHandler(default_device->protocol_version_);
    }

    port_to_bulk_read_[port_name] = new dynamixel::GroupBulkRead(port, default_pkt_handler);
  }

  // (for loop) check all dxls are connected.
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name = it.first;
    Dynamixel* dxl = it.second;

    if (ping(joint_name) != 0)
    {
      usleep(10 * 1000);
      if (ping(joint_name) != 0)
        ROS_ERROR("JOINT[%s] does NOT respond!!", joint_name.c_str());
    }
  }

  initializeDevice(init_file_path);
  queue_thread_ = boost::thread(boost::bind(&KurokoJointController::msgQueueThread, this));
  ROS_INFO("[KurokoJointController::initialize] Initialization complete");
  return true;
}

void KurokoJointController::initializeDevice(const std::string& init_file_path)
{
  // device initialize
  if (debug_print_)
    ROS_WARN("INIT FILE LOAD");
  ROS_INFO("KurokoJointController::initializeDevice - Loading: %s", init_file_path.c_str());
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(init_file_path);

    for (YAML::const_iterator it_doc = doc.begin(); it_doc != doc.end(); it_doc++)
    {
      std::string joint_name = it_doc->first.as<std::string>();

      YAML::Node joint_node = doc[joint_name];
      if (joint_node.size() == 0)
        continue;

      Dynamixel* dxl = nullptr;
      auto dxl_it = robot_->dxls_.find(joint_name);
      if (dxl_it != robot_->dxls_.end())
        dxl = dxl_it->second;

      if (dxl == nullptr)
      {
        ROS_WARN("Joint [%s] was not found.", joint_name.c_str());
        continue;
      }
      if (debug_print_)
        ROS_INFO("JOINT_NAME: %s", joint_name.c_str());

      uint8_t torque_enabled = 0;
      read1Byte(joint_name, dxl->torque_enable_item_->address_, &torque_enabled);

      for (YAML::const_iterator it_joint = joint_node.begin(); it_joint != joint_node.end(); it_joint++)
      {
        std::string item_name = it_joint->first.as<std::string>();

        if (debug_print_)
          ROS_INFO("  ITEM_NAME: %s", item_name.c_str());

        uint32_t value = it_joint->second.as<uint32_t>();

        ControlTableItem* item = dxl->ctrl_table_[item_name];
        if (item == nullptr)
        {
          ROS_WARN("Control Item [%s] was not found.", item_name.c_str());
          continue;
        }

        if (item->memory_type_ == EEPROM)
        {
          uint8_t data8 = 0;
          uint16_t data16 = 0;
          uint32_t data32 = 0;

          switch (item->data_length_)
          {
            case 1:
              read1Byte(joint_name, item->address_, &data8);
              if (data8 == value)
                continue;
              break;
            case 2:
              read2Byte(joint_name, item->address_, &data16);
              if (data16 == value)
                continue;
              break;
            case 4:
              read4Byte(joint_name, item->address_, &data32);
              if (data32 == value)
                continue;
              break;
            default:
              break;
          }

          if (torque_enabled == 1)
          {
            ROS_ERROR("################\nThe initial value of the EEPROM area has "
                      "been changed. \nTurn off Torque Enable and try again.");
            exit(-1);
          }
        }

        switch (item->data_length_)
        {
          case 1:
            write1Byte(joint_name, item->address_, (uint8_t)value);
            break;
          case 2:
            write2Byte(joint_name, item->address_, (uint16_t)value);
            break;
          case 4:
            write4Byte(joint_name, item->address_, value);
            break;
          default:
            break;
        }

        if (item->memory_type_ == EEPROM)
        {
          // Write to EEPROM -> delay is required (max delay: 55 msec per byte)
          usleep(item->data_length_ * 55 * 1000);
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    ROS_WARN("[KurokoJointController::initializeDevice] Failed to load init file: %s", e.what());
  }

  // [ BulkRead ] StartAddress : Present Position , Length : 10 (
  // Position/Velocity/Current )
  for (auto& it : robot_->ports_)
  {
    if (port_to_bulk_read_[it.first] != nullptr)
    {
      port_to_bulk_read_[it.first]->clearParam();
    }
  }
  for (auto& it : robot_->dxls_)
  {
    std::string joint_name = it.first;
    Dynamixel* dxl = it.second;

    if (dxl == nullptr)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

    //    // bulk read default : present position
    //    if(dxl->present_position_item != 0)
    //    {
    //        bulkread_start_addr    = dxl->present_position_item->address;
    //        bulkread_data_length   = dxl->present_position_item->data_length;
    //    }

    uint8_t torque_enabled = 0;
    read1Byte(joint_name, dxl->torque_enable_item_->address_, &torque_enabled);

    // calculate bulk read start address & data length
    auto indirect_addr_it = dxl->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != dxl->ctrl_table_.end())  // INDIRECT_ADDRESS_1 exist
    {
      if (!dxl->bulk_read_items_.empty())
      {
        uint16_t data16 = 0;

        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
        {
          int addr_leng = dxl->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
            // ROS_WARN("[%12s] INDIR_ADDR: %d, ITEM_ADDR: %d",
            // joint_name.c_str(), indirect_addr,
            // dxl->ctrl_table[dxl->bulk_read_items[i]->item_name]->address +
            // _l);

            read2Byte(joint_name, indirect_addr, &data16);
            if (data16 != dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              if (torque_enabled == 1)
              {
                ROS_ERROR("################\nThe indirect address of the "
                          "EEPROM area has been changed. \nTurn off Torque "
                          "Enable and try again.");
                exit(-1);
              }
              write2Byte(joint_name, indirect_addr,
                         dxl->ctrl_table_[dxl->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else  // INDIRECT_ADDRESS_1 NOT exist
    {
      if (!dxl->bulk_read_items_.empty())
      {
        bulkread_start_addr = dxl->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem* last_item = dxl->bulk_read_items_[0];

        for (auto& bulk_read_item : dxl->bulk_read_items_)
        {
          int addr = bulk_read_item->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = bulk_read_item;
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

    //    ROS_WARN("[%12s] start_addr: %d, data_length: %d", joint_name.c_str(),
    //    bulkread_start_addr, bulkread_data_length);
    if (bulkread_start_addr != 0)
      port_to_bulk_read_[dxl->port_name_]->addParam(dxl->id_, bulkread_start_addr, bulkread_data_length);

    // Torque ON
    if (writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1) != COMM_SUCCESS)
      writeCtrlItem(joint_name, dxl->torque_enable_item_->item_name_, 1);
  }

  for (auto& it : robot_->sensors_)
  {
    std::string sensor_name = it.first;
    Sensor* sensor = it.second;

    if (sensor == nullptr)
      continue;

    int bulkread_start_addr = 0;
    int bulkread_data_length = 0;

    // calculate bulk read start address & data length
    auto indirect_addr_it = sensor->ctrl_table_.find(INDIRECT_ADDRESS_1);
    if (indirect_addr_it != sensor->ctrl_table_.end())  // INDIRECT_ADDRESS_1 exist
    {
      if (!sensor->bulk_read_items_.empty())
      {
        uint16_t data16 = 0;

        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        // set indirect address
        int indirect_addr = indirect_addr_it->second->address_;
        for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
        {
          int addr_leng = sensor->bulk_read_items_[i]->data_length_;

          bulkread_data_length += addr_leng;
          for (int l = 0; l < addr_leng; l++)
          {
            //            ROS_WARN("[%12s] INDIR_ADDR: %d, ITEM_ADDR: %d",
            //            sensor_name.c_str(), indirect_addr,
            //            sensor->ctrl_table[sensor->bulk_read_items[i]->item_name]->address
            //            + _l);
            read2Byte(sensor_name, indirect_addr, &data16);
            if (data16 != sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l)
            {
              write2Byte(sensor_name, indirect_addr,
                         sensor->ctrl_table_[sensor->bulk_read_items_[i]->item_name_]->address_ + l);
            }
            indirect_addr += 2;
          }
        }
      }
    }
    else  // INDIRECT_ADDRESS_1 NOT exist
    {
      if (!sensor->bulk_read_items_.empty())
      {
        bulkread_start_addr = sensor->bulk_read_items_[0]->address_;
        bulkread_data_length = 0;

        ControlTableItem* last_item = sensor->bulk_read_items_[0];

        for (auto& bulk_read_item : sensor->bulk_read_items_)
        {
          int addr = bulk_read_item->address_;
          if (addr < bulkread_start_addr)
            bulkread_start_addr = addr;
          else if (last_item->address_ < addr)
            last_item = bulk_read_item;
        }

        bulkread_data_length = last_item->address_ - bulkread_start_addr + last_item->data_length_;
      }
    }

    // ROS_WARN("[%12s] start_addr: %d, data_length: %d", sensor_name.c_str(),
    // bulkread_start_addr, bulkread_data_length);
    if (bulkread_start_addr != 0)
      port_to_bulk_read_[sensor->port_name_]->addParam(sensor->id_, bulkread_start_addr, bulkread_data_length);
  }
  ROS_INFO("[KurokoJointController::initializeDevice] Device Initialization Complete");
}

void KurokoJointController::gazeboTimerThread()
{
  ros::Rate gazebo_rate(1000 / robot_->getControlCycle());

  while (!stop_timer_)
  {
    if (init_pose_loaded_)
      process();
    gazebo_rate.sleep();
  }
}

void KurokoJointController::msgQueueThread()
{
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* subscriber */
  ros::Subscriber write_control_table_sub = ros_node.subscribe("/motion_control/write_control_table", 5,
                                                               &KurokoJointController::writeControlTableCallback, this);
  ros::Subscriber sync_write_item_sub =
      ros_node.subscribe("/motion_control/sync_write_item", 10, &KurokoJointController::syncWriteItemCallback, this);
  ros::Subscriber joint_ctrl_modules_sub = ros_node.subscribe("/motion_control/set_joint_ctrl_modules", 10,
                                                              &KurokoJointController::setJointCtrlModuleCallback, this);
  ros::Subscriber enable_ctrl_module_sub =
      ros_node.subscribe("/motion_control/enable_ctrl_module", 10, &KurokoJointController::setCtrlModuleCallback, this);
  ros::Subscriber control_mode_sub = ros_node.subscribe("/motion_control/set_control_mode", 10,
                                                        &KurokoJointController::setControllerModeCallback, this);
  ros::Subscriber joint_states_sub =
      ros_node.subscribe("/motion_control/set_joint_states", 10, &KurokoJointController::setJointStatesCallback, this);
  ros::Subscriber enable_offset_sub =
      ros_node.subscribe("/motion_control/enable_offset", 10, &KurokoJointController::enableOffsetCallback, this);

  ros::Subscriber gazebo_joint_states_sub;
  if (gazebo_mode_)
    gazebo_joint_states_sub = ros_node.subscribe("/" + gazebo_robot_name_ + "/joint_states", 10,
                                                 &KurokoJointController::gazeboJointStatesCallback, this);

  /* publisher */
  goal_joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("/motion_control/goal_joint_states", 10);
  present_joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("/motion_control/present_joint_states", 10);
  current_module_pub_ =
      ros_node.advertise<robotis_controller_msgs::JointCtrlModule>("/motion_control/present_joint_ctrl_modules", 10);

  if (gazebo_mode_)
  {
    for (auto& it : robot_->dxls_)
    {
      gazebo_joint_position_pub_[it.first] =
          ros_node.advertise<std_msgs::Float64>("/" + gazebo_robot_name_ + "/" + it.first + "_position/command", 1);
      gazebo_joint_velocity_pub_[it.first] =
          ros_node.advertise<std_msgs::Float64>("/" + gazebo_robot_name_ + "/" + it.first + "_velocity/command", 1);
      gazebo_joint_effort_pub_[it.first] =
          ros_node.advertise<std_msgs::Float64>("/" + gazebo_robot_name_ + "/" + it.first + "_effort/command", 1);
    }
  }

  /* service */
  ros::ServiceServer get_joint_module_server = ros_node.advertiseService(
      "/motion_control/get_present_joint_ctrl_modules", &KurokoJointController::getJointCtrlModuleService, this);
  ros::ServiceServer set_joint_module_server = ros_node.advertiseService(
      "/motion_control/set_present_joint_ctrl_modules", &KurokoJointController::setJointCtrlModuleService, this);
  ros::ServiceServer set_module_server = ros_node.advertiseService("/motion_control/set_present_ctrl_modules",
                                                                   &KurokoJointController::setCtrlModuleService, this);
  ros::ServiceServer load_offset_server =
      ros_node.advertiseService("/motion_control/load_offset", &KurokoJointController::loadOffsetService, this);

  ros::WallDuration duration(robot_->getControlCycle() / 1000.0);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

void* KurokoJointController::timerThread(void* param)
{
  KurokoJointController* controller = (KurokoJointController*)param;
  static struct timespec next_time;
  static struct timespec curr_time;

  ROS_DEBUG("controller::thread_proc started");

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while (!controller->stop_timer_)
  {
    next_time.tv_sec += (next_time.tv_nsec + controller->robot_->getControlCycle() * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + controller->robot_->getControlCycle() * 1000000) % 1000000000;

    controller->process();

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
    if (delta_nsec < -100000)
    {
      if (controller->debug_print_)
      {
        fprintf(stderr,
                "[KurokoJointController::ThreadProc] NEXT TIME < CURR TIME.. "
                "(%f)[%ld.%09ld / %ld.%09ld]",
                delta_nsec / 1000000.0, (long)next_time.tv_sec, (long)next_time.tv_nsec, (long)curr_time.tv_sec,
                (long)curr_time.tv_nsec);
      }

      // next_time = curr_time + 3 msec
      next_time.tv_sec = curr_time.tv_sec + (curr_time.tv_nsec + 3000000) / 1000000000;
      next_time.tv_nsec = (curr_time.tv_nsec + 3000000) % 1000000000;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
  }
  return nullptr;
}

void KurokoJointController::startTimer()
{
  if (this->is_timer_running_)
    return;

  if (this->gazebo_mode_)
  {
    // create and start the thread
    gazebo_thread_ = boost::thread(boost::bind(&KurokoJointController::gazeboTimerThread, this));
  }
  else
  {
    initializeSyncWrite();

    for (auto& it : port_to_bulk_read_)
    {
      if (it.second != NULL)
      {
        it.second->txPacket();
      }
    }

    usleep(8 * 1000);

    int error;
    struct sched_param param;
    pthread_attr_t attr;

    pthread_attr_init(&attr);

    error = pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    if (error != 0)
      ROS_ERROR("pthread_attr_setschedpolicy error = %d\n", error);
    error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (error != 0)
      ROS_ERROR("pthread_attr_setinheritsched error = %d\n", error);

    memset(&param, 0, sizeof(param));
    int max_priority = sched_get_priority_max(SCHED_OTHER);
    if (max_priority == -1)
    {
      ROS_ERROR("Failed to get max priority");
      pthread_attr_destroy(&attr);
      return;
    }
    param.sched_priority = max_priority;
    error = pthread_attr_setschedparam(&attr, &param);
    if (error != 0)
      ROS_ERROR("pthread_attr_setschedparam: %s", strerror(error));

    // create and start the thread
    if ((error = pthread_create(&this->timer_thread_, &attr, this->timerThread, this)) != 0)
    {
      ROS_ERROR("Creating timer thread failed: %s", strerror(error));
      exit(-1);
    }

    ROS_INFO("[KurokoJointController::startTimer] Timer thread started");
  }

  this->is_timer_running_ = true;
}

void KurokoJointController::stopTimer()
{
  int error = 0;

  // set the flag to stop the thread
  if (this->is_timer_running_)
  {
    this->stop_timer_ = true;

    if (!this->gazebo_mode_)
    {
      // wait until the thread is stopped.
      if ((error = pthread_join(this->timer_thread_, nullptr)) != 0)
        exit(-1);

      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != NULL)
        {
          it.second->rxPacket();
        }
      }

      for (auto& it : port_to_sync_write_position_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_position_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_p_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_i_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_velocity_d_gain_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
      for (auto& it : port_to_sync_write_current_)
      {
        if (it.second != NULL)
          it.second->clearParam();
      }
    }
    else
    {
      // wait until the thread is stopped.
      gazebo_thread_.join();
    }

    this->stop_timer_ = false;
    this->is_timer_running_ = false;
  }
}

bool KurokoJointController::isTimerRunning()
{
  return this->is_timer_running_;
}

void KurokoJointController::loadOffset(const std::string& path)
{
  YAML::Node doc;

  ROS_INFO("KurokoJointController::loadOffset - Loading: %s", path.c_str());
  try
  {
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception& e)
  {
    ROS_WARN("Fail to load offset yaml.");
    return;
  }

  YAML::Node offset_node = doc["offset"];
  if (offset_node.size() == 0)
    return;

  ROS_INFO("Load offsets...");
  for (YAML::const_iterator it = offset_node.begin(); it != offset_node.end(); it++)
  {
    std::string joint_name = it->first.as<std::string>();
    double offset = it->second.as<double>();

    auto dxl_it = robot_->dxls_.find(joint_name);
    if (dxl_it != robot_->dxls_.end())
      dxl_it->second->dxl_state_->position_offset_ = offset;
  }
}

void KurokoJointController::process()
{
  // avoid duplicated function call
  static bool is_process_running = false;
  if (is_process_running)
    return;
  is_process_running = true;

  // ROS_INFO("Controller::Process()");
  // offset ratio
  if (is_offset_enabled_)
  {
    if (offset_ratio_ < 1.0)
      offset_ratio_ += 0.01;
    else
      offset_ratio_ = 1.0;
  }
  else
  {
    if (offset_ratio_ > 0.0)
      offset_ratio_ -= 0.01;
    else
      offset_ratio_ = 0.0;
  }

  ros::Time start_time;
  ros::Duration time_duration;

  if (debug_print_)
    start_time = ros::Time::now();

  sensor_msgs::JointState goal_state;
  sensor_msgs::JointState present_state;

  present_state.header.stamp = ros::Time::now();
  goal_state.header.stamp = present_state.header.stamp;

  if (controller_mode_ == MOTION_MODULE_MODE)
  {
    if (!gazebo_mode_)
    {
      // BulkRead Rx
      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != NULL)
        {
          robot_->ports_[it.first]->setPacketTimeout(0.0);
          if (debug_print_)
          {
            int result = it.second->rxPacket();
            if (result != COMM_SUCCESS)
            {
              ROS_ERROR_STREAM("Bulk Read Fail : " << it.first);
              ROS_ERROR_STREAM("Result : " << result);
            }
          }
          else
            it.second->rxPacket();
        }
      }

      // -> save to robot->dxls_[]->dxl_state_
      if (!robot_->dxls_.empty())
      {
        for (auto& dxl_it : robot_->dxls_)
        {
          Dynamixel* dxl = dxl_it.second;
          std::string port_name = dxl_it.second->port_name_;
          std::string joint_name = dxl_it.first;

          if (!dxl->bulk_read_items_.empty())
          {
            bool updated = false;
            uint32_t data = 0;
            for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
            {
              ControlTableItem* item = dxl->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(dxl->id_, item->address_, item->data_length_))
              {
                updated = true;
                data = port_to_bulk_read_[port_name]->getData(dxl->id_, item->address_, item->data_length_);

                // change dxl_state
                if (dxl->present_position_item_ != nullptr &&
                    item->item_name_ == dxl->present_position_item_->item_name_)
                {
                  dxl->dxl_state_->present_position_ =
                      dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->present_velocity_item_ != nullptr &&
                         item->item_name_ == dxl->present_velocity_item_->item_name_)
                  dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->present_current_item_ != nullptr &&
                         item->item_name_ == dxl->present_current_item_->item_name_)
                  dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(data);
                else if (dxl->goal_position_item_ != nullptr && item->item_name_ == dxl->goal_position_item_->item_name_)
                {
                  dxl->dxl_state_->goal_position_ =
                      dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->goal_velocity_item_ != nullptr && item->item_name_ == dxl->goal_velocity_item_->item_name_)
                  dxl->dxl_state_->goal_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->goal_current_item_ != nullptr && item->item_name_ == dxl->goal_current_item_->item_name_)
                  dxl->dxl_state_->goal_torque_ = dxl->convertValue2Torque(data);

                dxl->dxl_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to
            // Robot->dxls[]->dynamixel_state.update_time_stamp
            if (updated)
              dxl->dxl_state_->update_time_stamp_ =
                  TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nsec);
          }
        }
      }

      // -> save to robot->sensors_[]->sensor_state_
      if (!robot_->sensors_.empty())
      {
        for (auto& sensor_it : robot_->sensors_)
        {
          Sensor* sensor = sensor_it.second;
          std::string port_name = sensor_it.second->port_name_;
          std::string sensor_name = sensor_it.first;

          if (!sensor->bulk_read_items_.empty())
          {
            bool updated = false;
            uint32_t data = 0;
            for (int i = 0; i < sensor->bulk_read_items_.size(); i++)
            {
              ControlTableItem* item = sensor->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(sensor->id_, item->address_, item->data_length_))
              {
                updated = true;
                data = port_to_bulk_read_[port_name]->getData(sensor->id_, item->address_, item->data_length_);

                // change sensor_state
                sensor->sensor_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to
            // Robot->dxls[]->dynamixel_state.update_time_stamp
            if (updated)
              sensor->sensor_state_->update_time_stamp_ =
                  TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nsec);
          }
        }
      }

      if (debug_print_)
      {
        time_duration = ros::Time::now() - start_time;
        ROS_INFO("[KurokoJointController::process] Process() DONE in %2.6f ms", time_duration.toSec() * 1000);
      }

      // SyncWrite
      queue_mutex_.lock();

      if (!direct_sync_write_.empty())
      {
        ROS_INFO("Direct SyncWrite");
        for (auto& i : direct_sync_write_)
        {
          if (i != NULL)
          {
            i->txPacket();
            i->clearParam();
          }
        }
        direct_sync_write_.clear();
      }

      if (!port_to_sync_write_position_p_gain_.empty())
      {
        ROS_INFO("SyncWrite Position P Gain");
        for (auto& it : port_to_sync_write_position_p_gain_)
        {
          if (it.second != NULL)
          {
            it.second->txPacket();
            it.second->clearParam();
          }
        }
      }
      if (!port_to_sync_write_position_i_gain_.empty())
      {
        ROS_INFO("SyncWrite Position I Gain");
        for (auto& it : port_to_sync_write_position_i_gain_)
        {
          if (it.second != NULL)
          {
            it.second->txPacket();
            it.second->clearParam();
          }
        }
      }
      if (!port_to_sync_write_position_d_gain_.empty())
      {
        ROS_INFO("SyncWrite Position D Gain");
        for (auto& it : port_to_sync_write_position_d_gain_)
        {
          if (it.second != NULL)
          {
            it.second->txPacket();
            it.second->clearParam();
          }
        }
      }
      if (!port_to_sync_write_velocity_p_gain_.empty())
      {
        ROS_INFO("SyncWrite Velocity P Gain");
        for (auto& it : port_to_sync_write_velocity_p_gain_)
        {
          if (it.second != NULL)
          {
            it.second->txPacket();
            it.second->clearParam();
          }
        }
      }
      if (!port_to_sync_write_velocity_i_gain_.empty())
      {
        ROS_INFO("SyncWrite Velocity I Gain");
        for (auto& it : port_to_sync_write_velocity_i_gain_)
        {
          if (it.second != NULL)
          {
            it.second->txPacket();
            it.second->clearParam();
          }
        }
      }
      if (!port_to_sync_write_velocity_d_gain_.empty())
      {
        ROS_INFO("SyncWrite Velocity D Gain");
        for (auto& it : port_to_sync_write_velocity_d_gain_)
        {
          it.second->txPacket();
          it.second->clearParam();
        }
      }

      for (auto& it : port_to_sync_write_position_)
      {
        ROS_INFO("SyncWrite Position");
        if (it.second != NULL)
        {
          it.second->txPacket();
        }
      }
      for (auto& it : port_to_sync_write_velocity_)
      {
        ROS_INFO("SyncWrite Velocity");
        if (it.second != NULL)
        {
          it.second->txPacket();
        }
      }
      for (auto& it : port_to_sync_write_current_)
      {
        ROS_INFO("SyncWrite Current");
        if (it.second != NULL)
        {
          it.second->txPacket();
        }
      }
      queue_mutex_.unlock();
      // BulkRead Tx
      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != NULL)
        {
          it.second->txPacket();
        }
      }

      if (debug_print_)
      {
        time_duration = ros::Time::now() - start_time;
        fprintf(stderr, "(%2.6f) SyncWrite & BulkRead Tx \n", time_duration.nsec * 0.000001);
      }
    }
    else if (gazebo_mode_)
    {
      std_msgs::Float64 joint_msg;

      for (auto& dxl_it : robot_->dxls_)
      {
        std::string joint_name = dxl_it.first;
        Dynamixel* dxl = dxl_it.second;
        DynamixelState* dxl_state = dxl_it.second->dxl_state_;

        if (dxl->ctrl_module_name_ == "none")
        {
          joint_msg.data = dxl_state->goal_position_;
          gazebo_joint_position_pub_[joint_name].publish(joint_msg);
        }
      }

      for (auto& motion_module : motion_modules_)
      {
        if (!motion_module->getModuleEnable())
          continue;

        for (auto& dxl_it : robot_->dxls_)
        {
          std::string joint_name = dxl_it.first;
          Dynamixel* dxl = dxl_it.second;
          DynamixelState* dxl_state = dxl_it.second->dxl_state_;

          if (dxl->ctrl_module_name_ == motion_module->getModuleName())
          {
            if (motion_module->getControlMode() == PositionControl)
            {
              joint_msg.data = dxl_state->goal_position_;
              gazebo_joint_position_pub_[joint_name].publish(joint_msg);
            }
            else if (motion_module->getControlMode() == VelocityControl)
            {
              joint_msg.data = dxl_state->goal_velocity_;
              gazebo_joint_velocity_pub_[joint_name].publish(joint_msg);
            }
            else if (motion_module->getControlMode() == TorqueControl)
            {
              joint_msg.data = dxl_state->goal_torque_;
              gazebo_joint_effort_pub_[joint_name].publish(joint_msg);
            }
          }
        }
      }
    }
  }
  else if (controller_mode_ == DIRECT_CONTROL_MODE)
  {
    if (!gazebo_mode_)
    {
      // BulkRead Rx
      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != nullptr)
        {
          robot_->ports_[it.first]->setPacketTimeout(0.0);
          it.second->rxPacket();
        }
      }

      // -> save to robot->dxls_[]->dxl_state_
      if (!robot_->dxls_.empty())
      {
        for (auto& dxl_it : robot_->dxls_)
        {
          Dynamixel* dxl = dxl_it.second;
          std::string port_name = dxl_it.second->port_name_;
          std::string joint_name = dxl_it.first;

          if (!dxl->bulk_read_items_.empty())
          {
            uint32_t data = 0;
            for (int i = 0; i < dxl->bulk_read_items_.size(); i++)
            {
              ControlTableItem* item = dxl->bulk_read_items_[i];
              if (port_to_bulk_read_[port_name]->isAvailable(dxl->id_, item->address_, item->data_length_))
              {
                data = port_to_bulk_read_[port_name]->getData(dxl->id_, item->address_, item->data_length_);

                // change dxl_state
                if (dxl->present_position_item_ != nullptr &&
                    item->item_name_ == dxl->present_position_item_->item_name_)
                {
                  dxl->dxl_state_->present_position_ =
                      dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->present_velocity_item_ != nullptr &&
                         item->item_name_ == dxl->present_velocity_item_->item_name_)
                  dxl->dxl_state_->present_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->present_current_item_ != nullptr &&
                         item->item_name_ == dxl->present_current_item_->item_name_)
                  dxl->dxl_state_->present_torque_ = dxl->convertValue2Torque(data);
                else if (dxl->goal_position_item_ != nullptr && item->item_name_ == dxl->goal_position_item_->item_name_)
                {
                  dxl->dxl_state_->goal_position_ =
                      dxl->convertValue2Radian(data) - dxl->dxl_state_->position_offset_ * offset_ratio_;
                }
                else if (dxl->goal_velocity_item_ != nullptr && item->item_name_ == dxl->goal_velocity_item_->item_name_)
                  dxl->dxl_state_->goal_velocity_ = dxl->convertValue2Velocity(data);
                else if (dxl->goal_current_item_ != nullptr && item->item_name_ == dxl->goal_current_item_->item_name_)
                  dxl->dxl_state_->goal_torque_ = dxl->convertValue2Torque(data);

                dxl->dxl_state_->bulk_read_table_[item->item_name_] = data;
              }
            }

            // -> update time stamp to
            // Robot->dxls[]->dynamixel_state.update_time_stamp
            dxl->dxl_state_->update_time_stamp_ =
                TimeStamp(present_state.header.stamp.sec, present_state.header.stamp.nsec);
          }
        }
      }

      if (!direct_sync_write_.empty())
      {
        queue_mutex_.lock();
        for (auto& i : direct_sync_write_)
        {
          i->txPacket();
          i->clearParam();
        }
        direct_sync_write_.clear();
        queue_mutex_.unlock();
      }

      // BulkRead Tx
      for (auto& it : port_to_bulk_read_)
      {
        if (it.second != nullptr)
        {
          it.second->txPacket();
        }
      }
    }
  }

  // Call SensorModule Process()
  // -> for loop : call SensorModule list -> Process()
  if (!sensor_modules_.empty())
  {
    for (auto& sensor_module : sensor_modules_)
    {
      sensor_module->process(robot_->dxls_, robot_->sensors_);

      for (auto& it : sensor_module->result_)
        sensor_result_[it.first] = it.second;
    }
  }

  if (debug_print_)
  {
    time_duration = ros::Time::now() - start_time;
    fprintf(stderr, "(%2.6f) SensorModule Process() & save result \n", time_duration.nsec * 0.000001);
  }

  if (controller_mode_ == MOTION_MODULE_MODE)
  {
    // Call MotionModule Process()
    // -> for loop : call MotionModule list -> Process()
    if (!motion_modules_.empty())
    {
      queue_mutex_.lock();

      for (auto& motion_module : motion_modules_)
      {
        if (!motion_module->getModuleEnable())
          continue;

        motion_module->process(robot_->dxls_, sensor_result_);

        // for loop : joint list
        for (auto& dxl_it : robot_->dxls_)
        {
          std::string joint_name = dxl_it.first;
          Dynamixel* dxl = dxl_it.second;
          DynamixelState* dxl_state = dxl_it.second->dxl_state_;

          if (dxl->ctrl_module_name_ == motion_module->getModuleName())
          {
            // do_sync_write = true;
            DynamixelState* result_state = motion_module->result_[joint_name];

            if (result_state == nullptr)
            {
              ROS_ERROR("[%s] %s ", motion_module->getModuleName().c_str(), joint_name.c_str());
              continue;
            }

            // TODO: check update time stamp ?

            if (motion_module->getControlMode() == PositionControl)
            {
              dxl_state->goal_position_ = result_state->goal_position_;

              if (!gazebo_mode_)
              {
                // add offset
                uint32_t pos_data;
                pos_data =
                    dxl->convertRadian2Value(dxl_state->goal_position_ + dxl_state->position_offset_ * offset_ratio_);

                uint8_t sync_write_data[4] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
                sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
                sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
                sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

                if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
                  port_to_sync_write_position_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);

                // if position p gain value is changed -> sync write
                if (result_state->position_p_gain_ != none_gain_ &&
                    dxl_state->position_p_gain_ != result_state->position_p_gain_)
                {
                  dxl_state->position_p_gain_ = result_state->position_p_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_p_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_p_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_p_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_p_gain_));

                  if (port_to_sync_write_position_p_gain_[dxl->port_name_] != nullptr)
                    port_to_sync_write_position_p_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if position i gain value is changed -> sync write
                if (result_state->position_i_gain_ != none_gain_ &&
                    dxl_state->position_i_gain_ != result_state->position_i_gain_)
                {
                  dxl_state->position_i_gain_ = result_state->position_i_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_i_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_i_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_i_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_i_gain_));

                  if (port_to_sync_write_position_i_gain_[dxl->port_name_] != nullptr)
                    port_to_sync_write_position_i_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if position d gain value is changed -> sync write
                if (result_state->position_d_gain_ != none_gain_ &&
                    dxl_state->position_d_gain_ != result_state->position_d_gain_)
                {
                  dxl_state->position_d_gain_ = result_state->position_d_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->position_d_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->position_d_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->position_d_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->position_d_gain_));

                  if (port_to_sync_write_position_d_gain_[dxl->port_name_] != nullptr)
                    port_to_sync_write_position_d_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity p gain gain value is changed -> sync write
                if (result_state->velocity_p_gain_ != none_gain_ &&
                    dxl_state->velocity_p_gain_ != result_state->velocity_p_gain_)
                {
                  dxl_state->velocity_p_gain_ = result_state->velocity_p_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));

                  if (port_to_sync_write_velocity_p_gain_[dxl->port_name_] != nullptr)
                    port_to_sync_write_velocity_p_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity i gain value is changed -> sync write
                if (result_state->velocity_i_gain_ != none_gain_ &&
                    dxl_state->velocity_i_gain_ != result_state->velocity_i_gain_)
                {
                  dxl_state->velocity_i_gain_ = result_state->velocity_i_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));

                  if (port_to_sync_write_velocity_i_gain_[dxl->port_name_] != nullptr)
                    port_to_sync_write_velocity_i_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }
              }
            }
            else if (motion_module->getControlMode() == VelocityControl)
            {
              dxl_state->goal_velocity_ = result_state->goal_velocity_;

              if (!gazebo_mode_)
              {
                uint32_t vel_data = dxl->convertVelocity2Value(dxl_state->goal_velocity_);
                uint8_t sync_write_data[4] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(vel_data));
                sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(vel_data));
                sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(vel_data));
                sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(vel_data));

                if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
                  port_to_sync_write_velocity_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);

                // if velocity p gain gain value is changed -> sync write
                if (result_state->velocity_p_gain_ != none_gain_ &&
                    dxl_state->velocity_p_gain_ != result_state->velocity_p_gain_)
                {
                  dxl_state->velocity_p_gain_ = result_state->velocity_p_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_p_gain_));

                  if (port_to_sync_write_velocity_p_gain_[dxl->port_name_] != nullptr)
                    port_to_sync_write_velocity_p_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity i gain value is changed -> sync write
                if (result_state->velocity_i_gain_ != none_gain_ &&
                    dxl_state->velocity_i_gain_ != result_state->velocity_i_gain_)
                {
                  dxl_state->velocity_i_gain_ = result_state->velocity_i_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_i_gain_));

                  if (port_to_sync_write_velocity_i_gain_[dxl->port_name_] != nullptr)
                    port_to_sync_write_velocity_i_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }

                // if velocity d gain value is changed -> sync write
                if (result_state->velocity_d_gain_ != none_gain_ &&
                    dxl_state->velocity_d_gain_ != result_state->velocity_d_gain_)
                {
                  dxl_state->velocity_d_gain_ = result_state->velocity_d_gain_;
                  uint8_t sync_write_data[4] = { 0 };
                  sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(dxl_state->velocity_d_gain_));
                  sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(dxl_state->velocity_d_gain_));

                  if (port_to_sync_write_velocity_d_gain_[dxl->port_name_] != nullptr)
                    port_to_sync_write_velocity_d_gain_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);
                }
              }
            }
            else if (motion_module->getControlMode() == TorqueControl)
            {
              dxl_state->goal_torque_ = result_state->goal_torque_;

              if (!gazebo_mode_)
              {
                uint32_t curr_data = dxl->convertTorque2Value(dxl_state->goal_torque_);
                uint8_t sync_write_data[2] = { 0 };
                sync_write_data[0] = DXL_LOBYTE(curr_data);
                sync_write_data[1] = DXL_HIBYTE(curr_data);

                if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
                  port_to_sync_write_current_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);
              }
            }
          }
        }
      }

      queue_mutex_.unlock();
    }

    if (debug_print_)
    {
      time_duration = ros::Time::now() - start_time;
      fprintf(stderr, "(%2.6f) MotionModule Process() & save result \n", time_duration.nsec * 0.000001);
    }
  }

  // publish present & goal position
  for (auto& dxl_it : robot_->dxls_)
  {
    std::string joint_name = dxl_it.first;
    Dynamixel* dxl = dxl_it.second;

    present_state.name.push_back(joint_name);
    present_state.position.push_back(dxl->dxl_state_->present_position_);
    present_state.velocity.push_back(dxl->dxl_state_->present_velocity_);
    present_state.effort.push_back(dxl->dxl_state_->present_torque_);

    goal_state.name.push_back(joint_name);
    goal_state.position.push_back(dxl->dxl_state_->goal_position_);
    goal_state.velocity.push_back(dxl->dxl_state_->goal_velocity_);
    goal_state.effort.push_back(dxl->dxl_state_->goal_torque_);
  }

  // -> publish present joint_states & goal joint states topic
  present_joint_state_pub_.publish(present_state);
  goal_joint_state_pub_.publish(goal_state);

  if (debug_print_)
  {
    time_duration = ros::Time::now() - start_time;
    fprintf(stderr, "(%2.6f) Process() DONE \n", time_duration.nsec * 0.000001);
  }

  is_process_running = false;
}

void KurokoJointController::addMotionModule(MotionModule* module)
{
  // check whether the module name already exists
  for (auto& motion_module : motion_modules_)
  {
    if (motion_module->getModuleName() == module->getModuleName())
    {
      ROS_ERROR("Motion Module Name [%s] already exist !!", module->getModuleName().c_str());
      return;
    }
  }

  module->initialize(robot_->getControlCycle(), robot_);
  motion_modules_.push_back(module);
  motion_modules_.unique();
}

void KurokoJointController::removeMotionModule(MotionModule* module)
{
  motion_modules_.remove(module);
}

void KurokoJointController::addSensorModule(SensorModule* module)
{
  // check whether the module name already exists
  for (auto& sensor_module : sensor_modules_)
  {
    if (sensor_module->getModuleName() == module->getModuleName())
    {
      ROS_ERROR("Sensor Module Name [%s] already exist !!", module->getModuleName().c_str());
      return;
    }
  }

  module->initialize(robot_->getControlCycle(), robot_);
  sensor_modules_.push_back(module);
  sensor_modules_.unique();
}

void KurokoJointController::removeSensorModule(SensorModule* module)
{
  sensor_modules_.remove(module);
}

void KurokoJointController::writeControlTableCallback(const robotis_controller_msgs::WriteControlTable::ConstPtr& msg)
{
  Device* device = nullptr;

  if (debug_print_)
    fprintf(stderr, "[WriteControlTable] led control msg received\n");

  auto dev_it1 = robot_->dxls_.find(msg->joint_name);
  if (dev_it1 != robot_->dxls_.end())
  {
    device = dev_it1->second;
  }
  else
  {
    auto dev_it2 = robot_->sensors_.find(msg->joint_name);
    if (dev_it2 != robot_->sensors_.end())
    {
      device = dev_it2->second;
    }
    else
    {
      ROS_WARN("[WriteControlTable] Unknown device : %s", msg->joint_name.c_str());
      return;
    }
  }

  ControlTableItem* item = nullptr;
  auto item_it = device->ctrl_table_.find(msg->start_item_name);
  if (item_it != device->ctrl_table_.end())
  {
    item = item_it->second;
  }
  else
  {
    ROS_WARN("[WriteControlTable] Unknown item : %s", msg->start_item_name.c_str());
    return;
  }

  dynamixel::PortHandler* port = robot_->ports_[device->port_name_];
  dynamixel::PacketHandler* packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

  if (item->access_type_ == Read)
    return;

  queue_mutex_.lock();

  direct_sync_write_.push_back(new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, msg->data_length));
  direct_sync_write_[direct_sync_write_.size() - 1]->addParam(device->id_, (uint8_t*)(msg->data.data()));

  //  fprintf(stderr, "[WriteControlTable] %s -> %s : ",
  //  msg->joint_name.c_str(), msg->start_item_name.c_str()); for (auto &dt :
  //  msg->data)
  //	  fprintf(stderr, "%02X ", dt);
  //  fprintf(stderr, "\n");

  queue_mutex_.unlock();
}

void KurokoJointController::syncWriteItemCallback(const robotis_controller_msgs::SyncWriteItem::ConstPtr& msg)
{
  for (int i = 0; i < msg->joint_name.size(); i++)
  {
    Device* device;

    auto d_it1 = robot_->dxls_.find(msg->joint_name[i]);
    if (d_it1 != robot_->dxls_.end())
    {
      device = d_it1->second;
    }
    else
    {
      auto d_it2 = robot_->sensors_.find(msg->joint_name[i]);
      if (d_it2 != robot_->sensors_.end())
      {
        device = d_it2->second;
      }
      else
      {
        ROS_WARN("[SyncWriteItem] Unknown device : %s", msg->joint_name[i].c_str());
        continue;
      }
    }

    //    ControlTableItem *item  = device->ctrl_table_[msg->item_name];
    ControlTableItem* item = nullptr;
    auto item_it = device->ctrl_table_.find(msg->item_name);
    if (item_it != device->ctrl_table_.end())
    {
      item = item_it->second;
    }
    else
    {
      ROS_WARN("SyncWriteItem] Unknown item : %s", msg->item_name.c_str());
      continue;
    }

    dynamixel::PortHandler* port = robot_->ports_[device->port_name_];
    dynamixel::PacketHandler* packet_handler = dynamixel::PacketHandler::getPacketHandler(device->protocol_version_);

    if (item->access_type_ == Read)
      continue;

    queue_mutex_.lock();

    int idx = 0;
    if (direct_sync_write_.empty())
    {
      direct_sync_write_.push_back(
          new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
      idx = 0;
    }
    else
    {
      for (idx = 0; idx < direct_sync_write_.size(); idx++)
      {
        if (direct_sync_write_[idx]->getPortHandler() == port &&
            direct_sync_write_[idx]->getPacketHandler() == packet_handler)
          break;
      }

      if (idx == direct_sync_write_.size())
        direct_sync_write_.push_back(
            new dynamixel::GroupSyncWrite(port, packet_handler, item->address_, item->data_length_));
    }

    uint8_t* data = new uint8_t[item->data_length_];
    if (item->data_length_ == 1)
      data[0] = (uint8_t)msg->value[i];
    else if (item->data_length_ == 2)
    {
      data[0] = DXL_LOBYTE((uint16_t)msg->value[i]);
      data[1] = DXL_HIBYTE((uint16_t)msg->value[i]);
    }
    else if (item->data_length_ == 4)
    {
      data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)msg->value[i]));
      data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
      data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)msg->value[i]));
    }
    direct_sync_write_[idx]->addParam(device->id_, data);
    delete[] data;

    queue_mutex_.unlock();
  }
}

void KurokoJointController::setControllerModeCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "DirectControlMode")
  {
    for (auto& it : port_to_bulk_read_)
    {
      if (it.second != nullptr)
      {
        robot_->ports_[it.first]->setPacketTimeout(0.0);
        it.second->rxPacket();
      }
    }
    controller_mode_ = DIRECT_CONTROL_MODE;
  }
  else if (msg->data == "MotionModuleMode")
  {
    for (auto& it : port_to_bulk_read_)
    {
      if (it.second != nullptr)
      {
        it.second->txPacket();
      }
    }
    controller_mode_ = MOTION_MODULE_MODE;
  }
}

void KurokoJointController::setJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  queue_mutex_.lock();

  for (int i = 0; i < msg->name.size(); i++)
  {
    Dynamixel* dxl = robot_->dxls_[msg->name[i]];
    if (dxl == nullptr)
      continue;

    if ((controller_mode_ == DIRECT_CONTROL_MODE) ||
        (controller_mode_ == MOTION_MODULE_MODE && dxl->ctrl_module_name_ == "none"))
    {
      dxl->dxl_state_->goal_position_ = (double)msg->position[i];

      if (!gazebo_mode_)
      {
        // add offset
        uint32_t pos_data;
        pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ +
                                            dxl->dxl_state_->position_offset_ * offset_ratio_);

        uint8_t sync_write_data[4] = { 0 };
        sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
        sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
        sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
        sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

        if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
          port_to_sync_write_position_[dxl->port_name_]->changeParam(dxl->id_, sync_write_data);
      }
    }
  }

  queue_mutex_.unlock();
}

void KurokoJointController::setCtrlModuleCallback(const std_msgs::String::ConstPtr& msg)
{
  if (set_module_thread_.joinable())
    set_module_thread_.join();

  std::string module_name_to_set = msg->data;

  set_module_thread_ =
      boost::thread(boost::bind(&KurokoJointController::setCtrlModuleThread, this, module_name_to_set));
}

void KurokoJointController::setCtrlModule(std::string module_name)
{
  if (set_module_thread_.joinable())
    set_module_thread_.join();

  set_module_thread_ =
      boost::thread(boost::bind(&KurokoJointController::setCtrlModuleThread, this, std::move(module_name)));
}
void KurokoJointController::setJointCtrlModuleCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr& msg)
{
  if (msg->joint_name.size() != msg->module_name.size())
  {
    ROS_ERROR("[KurokoJointController] Joint name(%zu) and module name(%zu) size is not matched",
              msg->joint_name.size(), msg->module_name.size());
    return;
  }

  if (set_module_thread_.joinable())
    set_module_thread_.join();

  set_module_thread_ = boost::thread(boost::bind(&KurokoJointController::setJointCtrlModuleThread, this, msg));
}

void KurokoJointController::enableOffsetCallback(const std_msgs::Bool::ConstPtr& msg)
{
  is_offset_enabled_ = (bool)msg->data;
  if (is_offset_enabled_)
    offset_ratio_ = 0.0;
  else
    offset_ratio_ = 1.0;
}

bool KurokoJointController::getJointCtrlModuleService(robotis_controller_msgs::GetJointModule::Request& req,
                                                      robotis_controller_msgs::GetJointModule::Response& res)
{
  for (auto& idx : req.joint_name)
  {
    auto d_it = robot_->dxls_.find((std::string)idx);
    if (d_it != robot_->dxls_.end())
    {
      res.joint_name.push_back(idx);
      res.module_name.push_back(d_it->second->ctrl_module_name_);
    }
  }

  return !res.joint_name.empty();
}

bool KurokoJointController::setJointCtrlModuleService(robotis_controller_msgs::SetJointModule::Request& req,
                                                      robotis_controller_msgs::SetJointModule::Response& /*res*/)
{
  if (set_module_thread_.joinable())
    set_module_thread_.join();

  robotis_controller_msgs::JointCtrlModule modules;
  modules.joint_name = req.joint_name;
  modules.module_name = req.module_name;

  robotis_controller_msgs::JointCtrlModule::ConstPtr msg_ptr(new robotis_controller_msgs::JointCtrlModule(modules));

  if (modules.joint_name.size() != modules.module_name.size())
    return false;

  set_module_thread_ = boost::thread(boost::bind(&KurokoJointController::setJointCtrlModuleThread, this, msg_ptr));

  set_module_thread_.join();

  return true;
}

bool KurokoJointController::setCtrlModuleService(robotis_controller_msgs::SetModule::Request& req,
                                                 robotis_controller_msgs::SetModule::Response& res)
{
  if (set_module_thread_.joinable())
    set_module_thread_.join();

  std::string module_name_to_set = req.module_name;

  set_module_thread_ =
      boost::thread(boost::bind(&KurokoJointController::setCtrlModuleThread, this, module_name_to_set));

  set_module_thread_.join();

  res.result = true;
  return true;
}

bool KurokoJointController::loadOffsetService(robotis_controller_msgs::LoadOffset::Request& req,
                                              robotis_controller_msgs::LoadOffset::Response& res)
{
  loadOffset((std::string)req.file_path);
  res.result = true;
  return true;
}

void KurokoJointController::setJointCtrlModuleThread(const robotis_controller_msgs::JointCtrlModule::ConstPtr& msg)
{
  // stop module list
  std::list<MotionModule*> stop_modules;
  std::list<MotionModule*> enable_modules;

  for (unsigned int idx = 0; idx < msg->joint_name.size(); idx++)
  {
    Dynamixel* dxl = nullptr;
    std::map<std::string, Dynamixel*>::iterator dxl_it = robot_->dxls_.find((std::string)(msg->joint_name[idx]));
    if (dxl_it != robot_->dxls_.end())
      dxl = dxl_it->second;
    else
      continue;

    // enqueue
    if (dxl->ctrl_module_name_ != msg->module_name[idx])
    {
      for (auto& motion_module : motion_modules_)
      {
        if (motion_module->getModuleName() == dxl->ctrl_module_name_ && motion_module->getModuleEnable())
          stop_modules.push_back(motion_module);
      }
    }
  }

  // stop the module
  stop_modules.unique();
  for (auto& stop_module : stop_modules)
  {
    stop_module->stop();
  }

  // wait to stop
  for (auto& stop_module : stop_modules)
  {
    while (stop_module->isRunning())
      usleep(robot_->getControlCycle() * 1000);
  }

  // disable module(s)
  for (auto& stop_module : stop_modules)
  {
    stop_module->setModuleEnable(false);
  }

  // set ctrl module
  queue_mutex_.lock();

  for (unsigned int idx = 0; idx < msg->joint_name.size(); idx++)
  {
    std::string ctrl_module = msg->module_name[idx];
    std::string joint_name = msg->joint_name[idx];

    Dynamixel* dxl = nullptr;
    std::map<std::string, Dynamixel*>::iterator dxl_it = robot_->dxls_.find(joint_name);
    if (dxl_it != robot_->dxls_.end())
      dxl = dxl_it->second;
    else
      continue;

    // none
    if (ctrl_module.empty() || ctrl_module == "none")
    {
      dxl->ctrl_module_name_ = "none";

      if (gazebo_mode_)
        continue;

      uint32_t pos_data;
      pos_data =
          dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_ * offset_ratio_);

      uint8_t sync_write_data[4];
      sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
      sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
      sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
      sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

      if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
        port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

      if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
        port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
      if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
        port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
    }
    else
    {
      // check whether the module exist
      for (auto& motion_module : motion_modules_)
      {
        // if it exist
        if (motion_module->getModuleName() == ctrl_module)
        {
          std::map<std::string, DynamixelState*>::iterator result_it = motion_module->result_.find(joint_name);
          if (result_it == motion_module->result_.end())
            break;

          dxl->ctrl_module_name_ = ctrl_module;

          // enqueue enable module list
          enable_modules.push_back(motion_module);
          ControlMode mode = motion_module->getControlMode();

          if (gazebo_mode_)
            break;

          if (mode == PositionControl)
          {
            uint32_t pos_data;
            pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ +
                                                dxl->dxl_state_->position_offset_ * offset_ratio_);

            uint8_t sync_write_data[4];
            sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
            sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
            sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
            sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

            if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
              port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

            if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
              port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
            if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
              port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
          }
          else if (mode == VelocityControl)
          {
            uint32_t vel_data = dxl->convertVelocity2Value(dxl->dxl_state_->goal_velocity_);
            uint8_t sync_write_data[4];
            sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(vel_data));
            sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(vel_data));
            sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(vel_data));
            sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(vel_data));

            if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
              port_to_sync_write_velocity_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

            if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
              port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
            if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
              port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
          }
          else if (mode == TorqueControl)
          {
            uint32_t curr_data = dxl->convertTorque2Value(dxl->dxl_state_->goal_torque_);
            uint8_t sync_write_data[4];
            sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(curr_data));
            sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(curr_data));
            sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(curr_data));
            sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(curr_data));

            if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
              port_to_sync_write_current_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

            if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
              port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
            if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
              port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
          }
          break;
        }
      }
    }
  }

  // enable module(s)
  enable_modules.unique();
  for (auto& enable_module : enable_modules)
  {
    enable_module->setModuleEnable(true);
  }

  // TODO: set indirect address
  // -> check module's control_mode

  queue_mutex_.unlock();

  // publish current module
  robotis_controller_msgs::JointCtrlModule current_module_msg;
  for (auto& dxl : robot_->dxls_)
  {
    current_module_msg.joint_name.push_back(dxl.first);
    current_module_msg.module_name.push_back(dxl.second->ctrl_module_name_);
  }

  if (current_module_msg.joint_name.size() == current_module_msg.module_name.size())
    current_module_pub_.publish(current_module_msg);
}

void KurokoJointController::setCtrlModuleThread(const std::string& ctrl_module)
{
  // stop module
  std::list<MotionModule*> stop_modules;

  if (ctrl_module.empty() || ctrl_module == "none")
  {
    // enqueue all modules in order to stop
    for (auto& motion_module : motion_modules_)
    {
      if (motion_module->getModuleEnable())
        stop_modules.push_back(motion_module);
    }
  }
  else
  {
    for (auto& m_it : motion_modules_)
    {
      // if it exist
      if (m_it->getModuleName() == ctrl_module)
      {
        // enqueue the module which lost control of joint in order to stop
        for (auto& result_it : m_it->result_)
        {
          auto d_it = robot_->dxls_.find(result_it.first);

          if (d_it != robot_->dxls_.end())
          {
            // enqueue
            if (d_it->second->ctrl_module_name_ != ctrl_module)
            {
              for (auto& motion_module : motion_modules_)
              {
                if ((motion_module->getModuleName() == d_it->second->ctrl_module_name_) &&
                    (motion_module->getModuleEnable()))
                {
                  stop_modules.push_back(motion_module);
                }
              }
            }
          }
        }

        break;
      }
    }
  }

  // stop the module
  stop_modules.unique();
  for (auto& stop_module : stop_modules)
  {
    stop_module->stop();
  }

  // wait to stop
  for (auto& stop_module : stop_modules)
  {
    while (stop_module->isRunning())
      usleep(robot_->getControlCycle() * 1000);
  }

  // disable module(s)
  for (auto& stop_module : stop_modules)
  {
    stop_module->setModuleEnable(false);
  }

  // set ctrl module
  queue_mutex_.lock();

  if (debug_print_)
    ROS_INFO_STREAM("set module : " << ctrl_module);

  // none
  if ((ctrl_module.empty()) || (ctrl_module == "none"))
  {
    // set dxl's control module to "none"
    for (auto& d_it : robot_->dxls_)
    {
      Dynamixel* dxl = d_it.second;
      dxl->ctrl_module_name_ = "none";

      if (gazebo_mode_)
        continue;

      uint32_t pos_data;
      pos_data =
          dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ + dxl->dxl_state_->position_offset_ * offset_ratio_);

      uint8_t sync_write_data[4] = { 0 };
      sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
      sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
      sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
      sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

      if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
        port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

      if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
        port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
      if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
        port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
    }
  }
  else
  {
    // check whether the module exist
    for (auto& motion_module : motion_modules_)
    {
      // if it exist
      if (motion_module->getModuleName() == ctrl_module)
      {
        ControlMode mode = motion_module->getControlMode();
        for (auto& result_it : motion_module->result_)
        {
          auto d_it = robot_->dxls_.find(result_it.first);
          if (d_it != robot_->dxls_.end())
          {
            Dynamixel* dxl = d_it->second;
            dxl->ctrl_module_name_ = ctrl_module;

            if (gazebo_mode_)
              continue;

            if (mode == PositionControl)
            {
              uint32_t pos_data;
              pos_data = dxl->convertRadian2Value(dxl->dxl_state_->goal_position_ +
                                                  dxl->dxl_state_->position_offset_ * offset_ratio_);

              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(pos_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(pos_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(pos_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(pos_data));

              if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
                port_to_sync_write_position_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
                port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
                port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
            }
            else if (mode == VelocityControl)
            {
              uint32_t vel_data = dxl->convertVelocity2Value(dxl->dxl_state_->goal_velocity_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(vel_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(vel_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(vel_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(vel_data));

              if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
                port_to_sync_write_velocity_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
                port_to_sync_write_current_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
                port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
            }
            else if (mode == TorqueControl)
            {
              uint32_t curr_data = dxl->convertTorque2Value(dxl->dxl_state_->goal_torque_);
              uint8_t sync_write_data[4] = { 0 };
              sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(curr_data));
              sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(curr_data));
              sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(curr_data));
              sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(curr_data));

              if (port_to_sync_write_current_[dxl->port_name_] != nullptr)
                port_to_sync_write_current_[dxl->port_name_]->addParam(dxl->id_, sync_write_data);

              if (port_to_sync_write_velocity_[dxl->port_name_] != nullptr)
                port_to_sync_write_velocity_[dxl->port_name_]->removeParam(dxl->id_);
              if (port_to_sync_write_position_[dxl->port_name_] != nullptr)
                port_to_sync_write_position_[dxl->port_name_]->removeParam(dxl->id_);
            }
          }
        }

        break;
      }
    }
  }

  for (auto& motion_module : motion_modules_)
  {
    // set all used modules -> enable
    for (auto& d_it : robot_->dxls_)
    {
      if (d_it.second->ctrl_module_name_ == motion_module->getModuleName())
      {
        motion_module->setModuleEnable(true);
        break;
      }
    }
  }

  // TODO: set indirect address
  // -> check module's control_mode

  queue_mutex_.unlock();

  // publish current module
  robotis_controller_msgs::JointCtrlModule current_module_msg;
  for (auto& dxl_iter : robot_->dxls_)
  {
    current_module_msg.joint_name.push_back(dxl_iter.first);
    current_module_msg.module_name.push_back(dxl_iter.second->ctrl_module_name_);
  }

  if (current_module_msg.joint_name.size() == current_module_msg.module_name.size())
    current_module_pub_.publish(current_module_msg);
}

void KurokoJointController::gazeboJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  queue_mutex_.lock();

  for (unsigned int i = 0; i < msg->name.size(); i++)
  {
    auto d_it = robot_->dxls_.find((std::string)msg->name[i]);
    if (d_it != robot_->dxls_.end())
    {
      d_it->second->dxl_state_->present_position_ = msg->position[i];
      d_it->second->dxl_state_->present_velocity_ = msg->velocity[i];
      d_it->second->dxl_state_->present_torque_ = msg->effort[i];
    }
  }

  if (!init_pose_loaded_)
  {
    for (auto& it : robot_->dxls_)
      it.second->dxl_state_->goal_position_ = it.second->dxl_state_->present_position_;
    init_pose_loaded_ = true;
  }

  queue_mutex_.unlock();
}

bool KurokoJointController::isTimerStopped()
{
  if (this->is_timer_running_)
  {
    if (debug_print_)
      ROS_WARN("Process Timer is running.. STOP the timer first.");
    return false;
  }
  return true;
}

int KurokoJointController::ping(const std::string& joint_name, uint8_t* error)
{
  return ping(joint_name, nullptr, error);
}
int KurokoJointController::ping(const std::string& joint_name, uint16_t* model_number, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->ping(port_handler, dxl->id_, model_number, error);
}

int KurokoJointController::action(const std::string& joint_name)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->action(port_handler, dxl->id_);
}
int KurokoJointController::reboot(const std::string& joint_name, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->reboot(port_handler, dxl->id_, error);
}
int KurokoJointController::factoryReset(const std::string& joint_name, uint8_t option, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->factoryReset(port_handler, dxl->id_, option, error);
}

int KurokoJointController::read(const std::string& joint_name, uint16_t address, uint16_t length, uint8_t* data,
                                uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->readTxRx(port_handler, dxl->id_, address, length, data, error);
}

int KurokoJointController::readCtrlItem(const std::string& joint_name, const std::string& item_name, uint32_t* data,
                                        uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  ControlTableItem* item = dxl->ctrl_table_[item_name];
  if (item == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  int result = COMM_NOT_AVAILABLE;
  switch (item->data_length_)
  {
    case 1:
    {
      uint8_t read_data = 0;
      result = pkt_handler->read1ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    case 2:
    {
      uint16_t read_data = 0;
      result = pkt_handler->read2ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    case 4:
    {
      uint32_t read_data = 0;
      result = pkt_handler->read4ByteTxRx(port_handler, dxl->id_, item->address_, &read_data, error);
      if (result == COMM_SUCCESS)
        *data = read_data;
      break;
    }
    default:
      break;
  }
  return result;
}

int KurokoJointController::read1Byte(const std::string& joint_name, uint16_t address, uint8_t* data, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->read1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int KurokoJointController::read2Byte(const std::string& joint_name, uint16_t address, uint16_t* data, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->read2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int KurokoJointController::read4Byte(const std::string& joint_name, uint16_t address, uint32_t* data, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->read4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int KurokoJointController::write(const std::string& joint_name, uint16_t address, uint16_t length, uint8_t* data,
                                 uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->writeTxRx(port_handler, dxl->id_, address, length, data, error);
}

int KurokoJointController::writeCtrlItem(const std::string& joint_name, const std::string& item_name, uint32_t data,
                                         uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  ControlTableItem* item = dxl->ctrl_table_[item_name];
  if (item == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  int result = COMM_NOT_AVAILABLE;
  uint8_t* write_data = new uint8_t[item->data_length_];
  if (item->data_length_ == 1)
  {
    write_data[0] = (uint8_t)data;
    result = pkt_handler->write1ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  else if (item->data_length_ == 2)
  {
    write_data[0] = DXL_LOBYTE((uint16_t)data);
    write_data[1] = DXL_HIBYTE((uint16_t)data);
    result = pkt_handler->write2ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  else if (item->data_length_ == 4)
  {
    write_data[0] = DXL_LOBYTE(DXL_LOWORD((uint32_t)data));
    write_data[1] = DXL_HIBYTE(DXL_LOWORD((uint32_t)data));
    write_data[2] = DXL_LOBYTE(DXL_HIWORD((uint32_t)data));
    write_data[3] = DXL_HIBYTE(DXL_HIWORD((uint32_t)data));
    result = pkt_handler->write4ByteTxRx(port_handler, dxl->id_, item->address_, data, error);
  }
  delete[] write_data;
  return result;
}

int KurokoJointController::write1Byte(const std::string& joint_name, uint16_t address, uint8_t data, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->write1ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int KurokoJointController::write2Byte(const std::string& joint_name, uint16_t address, uint16_t data, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->write2ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int KurokoJointController::write4Byte(const std::string& joint_name, uint16_t address, uint32_t data, uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->write4ByteTxRx(port_handler, dxl->id_, address, data, error);
}

int KurokoJointController::regWrite(const std::string& joint_name, uint16_t address, uint16_t length, uint8_t* data,
                                    uint8_t* error)
{
  if (!isTimerStopped())
    return COMM_PORT_BUSY;

  Dynamixel* dxl = robot_->dxls_[joint_name];
  if (dxl == nullptr)
    return COMM_NOT_AVAILABLE;

  dynamixel::PacketHandler* pkt_handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
  dynamixel::PortHandler* port_handler = robot_->ports_[dxl->port_name_];

  return pkt_handler->regWriteTxRx(port_handler, dxl->id_, address, length, data, error);
}
