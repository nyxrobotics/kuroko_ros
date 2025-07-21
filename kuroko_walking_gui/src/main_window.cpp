#include "../include/kuroko_walking_gui/main_window.hpp"
#include <QMessageBox>
#include <QShortcut>
#include <QtGui>
#include <iostream>

namespace walking_gui
{
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget* parent)
  : QMainWindow(parent), qnode_kuroko_(argc, argv), is_updating_(false), is_walking_(false)
{
  // code to DEBUG
  debug_ = false;

  if (argc >= 2)
  {
    std::string arg_code(argv[1]);
    debug_ = arg_code == "debug";
  }

  ui_.setupUi(this);  // Calling this incidentally connects all ui's triggers to
                      // on_...() callbacks in this class.
  QObject::connect(ui_.action_about_Qt, SIGNAL(triggered(bool)), qApp,
                   SLOT(aboutQt()));  // qApp is a global variable for the application

  readSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing - qt-designer should have this
                                        // already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_kuroko_, SIGNAL(rosShutdown()), this, SLOT(close()));

  qRegisterMetaType<std::vector<int>>("std::vector<int>");
  QObject::connect(&qnode_kuroko_, SIGNAL(updateCurrentJointControlMode(std::vector<int>)), this,
                   SLOT(updateCurrentJointMode(std::vector<int>)));
  QObject::connect(&qnode_kuroko_, SIGNAL(updateHeadAngles(double, double)), this,
                   SLOT(updateHeadAngles(double, double)));

  QObject::connect(ui_.head_pan_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadAngle()));
  QObject::connect(ui_.head_tilt_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadAngle()));

  qRegisterMetaType<kuroko_walking_module_msgs::WalkingParam>("op_walking_params");
  QObject::connect(&qnode_kuroko_, SIGNAL(updateWalkingParameters(kuroko_walking_module_msgs::WalkingParam)), this,
                   SLOT(updateWalkingParams(kuroko_walking_module_msgs::WalkingParam)));

  /*********************
   ** Logging
   **********************/
  ui_.view_logging->setModel(qnode_kuroko_.loggingModel());
  QObject::connect(&qnode_kuroko_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
   ** Auto Start
   **********************/
  qnode_kuroko_.init();
  initModeUnit();
  setUserShortcut();
  updateModuleUI();

  // Set Preview widget
  bool result = ui_.widget_preview_walking->init(&qnode_kuroko_);
  if (!result)
    exit(0);
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::showNoMasterMessage()
{
  QMessageBox msg_box;
  msg_box.setText("Couldn't find the ros master.");
  msg_box.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_clear_log_clicked(bool /*check*/)
{
  qnode_kuroko_.clearLog();
}
void MainWindow::on_button_init_pose_clicked(bool /*check*/)
{
  qnode_kuroko_.moveInitPose();
}

// Walking
void MainWindow::on_button_init_gyro_clicked(bool /*check*/)
{
  qnode_kuroko_.initGyro();
}

void MainWindow::on_button_walking_start_clicked(bool /*check*/)
{
  is_walking_ = true;
  qnode_kuroko_.setWalkingCommand("start");
}

void MainWindow::on_button_walking_stop_clicked(bool /*check*/)
{
  is_walking_ = false;
  qnode_kuroko_.setWalkingCommand("stop");
}

void MainWindow::on_button_param_refresh_clicked(bool /*check*/)
{
  qnode_kuroko_.refreshWalkingParam();
}

void MainWindow::on_button_param_save_clicked(bool /*check*/)
{
  qnode_kuroko_.setWalkingCommand("save");
}

void MainWindow::on_button_param_apply_clicked(bool /*check*/)
{
  applyWalkingParams();
}

void MainWindow::on_checkBox_balance_on_clicked(bool /*check*/)
{
}
void MainWindow::on_checkBox_balance_off_clicked(bool /*check*/)
{
}

void MainWindow::on_head_center_button_clicked(bool /*check*/)
{
  qnode_kuroko_.log(QNodeKuroko::INFO, "Go Head init position");
  setHeadAngle(0, 0);
}

void MainWindow::on_button_start_tracking_clicked(bool /*check*/)
{
  qnode_kuroko_.setModuleToDemo();

  usleep(10 * 1000);

  qnode_kuroko_.setDemoCommand("start");
}

void MainWindow::on_button_stop_tracking_clicked(bool /*check*/)
{
  qnode_kuroko_.setDemoCommand("stop");
}

void MainWindow::on_button_r_kick_clicked(bool /*check*/)
{
  qnode_kuroko_.setActionModuleBody();

  usleep(10 * 1000);

  qnode_kuroko_.playMotion(RIGHT_KICK);
}

void MainWindow::on_button_l_kick_clicked(bool /*check*/)
{
  qnode_kuroko_.setActionModuleBody();

  usleep(10 * 1000);

  qnode_kuroko_.playMotion(LEFT_KICK);
}

void MainWindow::on_button_getup_front_clicked(bool /*check*/)
{
  qnode_kuroko_.setActionModuleBody();

  usleep(10 * 1000);

  qnode_kuroko_.playMotion(GETUP_FRONT);
}

void MainWindow::on_button_getup_rear_clicked(bool /*check*/)
{
  qnode_kuroko_.setActionModuleBody();

  usleep(10 * 1000);

  qnode_kuroko_.playMotion(GETUP_REAR);
}

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

// user shortcut
void MainWindow::setUserShortcut()
{
  // Setup a signal mapper to avoid creating custom slots for each tab
  QSignalMapper* sig_map = new QSignalMapper(this);

  // Setup the shortcut for the first tab : Mode
  QShortcut* short_tab1 = new QShortcut(QKeySequence("F1"), this);
  connect(short_tab1, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab1, 0);

  // Setup the shortcut for the second tab : Manipulation
  QShortcut* short_tab2 = new QShortcut(QKeySequence("F2"), this);
  connect(short_tab2, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab2, 1);

  // Setup the shortcut for the third tab : Walking
  QShortcut* short_tab3 = new QShortcut(QKeySequence("F3"), this);
  connect(short_tab3, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab3, 2);

  // Setup the shortcut for the fouth tab : Head control
  QShortcut* short_tab4 = new QShortcut(QKeySequence("F4"), this);
  connect(short_tab4, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab4, 3);

  // Setup the shortcut for the fouth tab : Motion
  QShortcut* short_tab5 = new QShortcut(QKeySequence("F5"), this);
  connect(short_tab5, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab5, 4);

  // Wire the signal mapper to the tab widget index change slot
  connect(sig_map, SIGNAL(mapped(int)), ui_.tabWidget_control, SLOT(setCurrentIndex(int)));

  QShortcut* walking_shortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
  connect(walking_shortcut, SIGNAL(activated()), this, SLOT(walkingCommandShortcut()));
}

// mode control
// it's not used now
void MainWindow::setMode(bool /*check*/)
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  QList<QComboBox*> combo_children = ui_.widget_mode->findChildren<QComboBox*>();
  for (int ix = 0; ix < combo_children.length(); ix++)
  {
    std::stringstream stream;
    std::string joint;
    int id;

    int control_index = combo_children.at(ix)->currentIndex();
    // if(_control_index == QNodeThor3::Control_None) continue;

    std::string control_mode = combo_children.at(ix)->currentText().toStdString();

    if (qnode_kuroko_.getIDJointNameFromIndex(ix, id, joint))
    {
      stream << "[" << (id < 10 ? "0" : "") << id << "] " << joint << " : " << control_mode;

      control_msg.joint_name.push_back(joint);
      control_msg.module_name.push_back(control_mode);
    }
    else
    {
      stream << "id " << ix << " : " << control_mode;
    }

    qnode_kuroko_.log(QNodeKuroko::INFO, stream.str());
  }

  // no control
  if (control_msg.joint_name.empty())
    return;

  qnode_kuroko_.log(QNodeKuroko::INFO, "set mode");

  qnode_kuroko_.setJointControlMode(control_msg);
}

void MainWindow::updateCurrentJointMode(std::vector<int> mode)
{
  QList<QComboBox*> combo_children = ui_.widget_mode->findChildren<QComboBox*>();
  for (int ix = 0; ix < combo_children.length(); ix++)
  {
    int control_index = mode.at(ix);
    combo_children.at(ix)->setCurrentIndex(control_index);

    if (debug_)
    {
      std::stringstream stream;
      std::string joint;
      int id;

      std::string control_mode = combo_children.at(ix)->currentText().toStdString();

      if (qnode_kuroko_.getIDJointNameFromIndex(ix, id, joint))
      {
        stream << "[" << (id < 10 ? "0" : "") << id << "] " << joint << " : " << control_mode;
      }
      else
      {
        stream << "id " << ix << " : " << control_mode;
      }

      qnode_kuroko_.log(QNodeKuroko::INFO, stream.str());
    }
  }

  // set module UI
  updateModuleUI();
}

void MainWindow::updateModuleUI()
{
  if (debug_)
    return;

  for (int index = 0; index < qnode_kuroko_.getModeSize(); index++)
  {
    std::string mode = qnode_kuroko_.getModeName(index);
    if (mode.empty())
      continue;

    std::map<std::string, QList<QWidget*>>::iterator module_iter = module_ui_table_.find(mode);
    if (module_iter == module_ui_table_.end())
      continue;

    bool is_enable = qnode_kuroko_.isUsingModule(mode);

    QList<QWidget*> list = module_iter->second;
    for (auto ix : list)
    {
      ix->setEnabled(is_enable);
    }
  }

  // refresh walking parameter
  if (qnode_kuroko_.isUsingModule("walking_module"))
    qnode_kuroko_.refreshWalkingParam();
}

// head control
void MainWindow::updateHeadAngles(double pan, double tilt)
{
  if (ui_.head_pan_slider->underMouse())
    return;
  if (ui_.head_pan_spinbox->underMouse())
    return;
  if (ui_.head_tilt_slider->underMouse())
    return;
  if (ui_.head_tilt_spinbox->underMouse())
    return;

  is_updating_ = true;

  ui_.head_pan_slider->setValue(pan * 180.0 / M_PI);
  ui_.head_tilt_slider->setValue(tilt * 180.0 / M_PI);
  is_updating_ = false;
}

void MainWindow::setHeadAngle()
{
  if (is_updating_)
    return;
  qnode_kuroko_.setHeadJoint(ui_.head_pan_slider->value() * M_PI / 180, ui_.head_tilt_slider->value() * M_PI / 180);
}

void MainWindow::setHeadAngle(double pan, double tilt)
{
  qnode_kuroko_.setHeadJoint(pan * M_PI / 180, tilt * M_PI / 180);
}

// walking
void MainWindow::updateWalkingParams(kuroko_walking_module_msgs::WalkingParam params)
{
  // init pose
  ui_.dSpinBox_init_x_offset->setValue(params.init_x_offset);
  ui_.dSpinBox_init_y_offset->setValue(params.init_y_offset);
  ui_.dSpinBox_init_z_offset->setValue(params.init_z_offset);
  ui_.dSpinBox_init_roll_offset->setValue(params.init_roll_offset * RADIAN2DEGREE);
  ui_.dSpinBox_init_pitch_offset->setValue(params.init_pitch_offset * RADIAN2DEGREE);
  ui_.dSpinBox_init_yaw_offset->setValue(params.init_yaw_offset * RADIAN2DEGREE);
  ui_.dSpinBox_init_hip_pitch_offset->setValue(params.init_hip_pitch_offset * RADIAN2DEGREE);
  // time
  ui_.dSpinBox_period_time->setValue(params.period_time * 1000);  // s -> ms
  ui_.dSpinBox_dsp_ratio->setValue(params.dsp_ratio);
  ui_.dSpinBox_step_fb_ratio->setValue(params.step_fb_ratio);
  // walking
  ui_.dSpinBox_x_step->setValue(params.x_step);
  ui_.dSpinBox_y_step->setValue(params.y_step);
  ui_.dSpinBox_z_step->setValue(params.z_step);
  ui_.dSpinBox_y_step->setValue(params.yaw_step);
  ui_.checkBox_move_aim_on->setChecked(params.move_aim_on);
  ui_.checkBox_move_aim_off->setChecked(!params.move_aim_on);
  // balance
  ui_.checkBox_balance_on->setChecked(params.balance_enable);
  ui_.checkBox_balance_off->setChecked(!params.balance_enable);
  ui_.dSpinBox_hip_roll_gain->setValue(params.balance_gyro_roll_gain);
  ui_.dSpinBox_knee_gain->setValue(params.balance_gyro_pitch_gain);
  ui_.dSpinBox_ankle_roll_gain->setValue(params.balance_gyro_y_gain);
  ui_.dSpinBox_ankle_pitch_gain->setValue(params.balance_gyro_x_gain);
  ui_.dSpinBox_y_swing_amplitude->setValue(params.y_swing_amplitude);
  ui_.dSpinBox_z_swing_amplitude->setValue(params.z_swing_amplitude);
  ui_.dSpinBox_hip_swing_up_amplitude_->setValue(params.hip_swing_up_amplitude_ * RADIAN2DEGREE);
  ui_.dSpinBox_shoulder_swing_amplitude->setValue(params.shoulder_swing_amplitude);
}

void MainWindow::applyWalkingParams()
{
  kuroko_walking_module_msgs::WalkingParam walking_param;

  // init pose
  walking_param.init_x_offset = ui_.dSpinBox_init_x_offset->value();
  walking_param.init_y_offset = ui_.dSpinBox_init_y_offset->value();
  walking_param.init_z_offset = ui_.dSpinBox_init_z_offset->value();
  walking_param.init_roll_offset = ui_.dSpinBox_init_roll_offset->value() * DEGREE2RADIAN;
  walking_param.init_pitch_offset = ui_.dSpinBox_init_pitch_offset->value() * DEGREE2RADIAN;
  walking_param.init_yaw_offset = ui_.dSpinBox_init_yaw_offset->value() * DEGREE2RADIAN;
  walking_param.init_hip_pitch_offset = ui_.dSpinBox_init_hip_pitch_offset->value() * DEGREE2RADIAN;
  // time
  walking_param.period_time = ui_.dSpinBox_period_time->value() * 0.001;  // ms -> s
  walking_param.dsp_ratio = ui_.dSpinBox_dsp_ratio->value();
  walking_param.step_fb_ratio = ui_.dSpinBox_step_fb_ratio->value();
  ;
  // walking
  walking_param.x_step = ui_.dSpinBox_x_step->value();
  walking_param.y_step = ui_.dSpinBox_y_step->value();
  walking_param.z_step = ui_.dSpinBox_z_step->value();
  walking_param.yaw_step = ui_.dSpinBox_a_step->value() * DEGREE2RADIAN;
  walking_param.move_aim_on = ui_.checkBox_move_aim_on->isChecked();
  // balance
  walking_param.balance_enable = ui_.checkBox_balance_on->isChecked();
  walking_param.balance_gyro_roll_gain = ui_.dSpinBox_hip_roll_gain->value();
  walking_param.balance_gyro_pitch_gain = ui_.dSpinBox_knee_gain->value();
  walking_param.balance_gyro_y_gain = ui_.dSpinBox_ankle_roll_gain->value();
  walking_param.balance_gyro_x_gain = ui_.dSpinBox_ankle_pitch_gain->value();
  walking_param.y_swing_amplitude = ui_.dSpinBox_y_swing_amplitude->value();
  walking_param.z_swing_amplitude = ui_.dSpinBox_z_swing_amplitude->value();
  walking_param.hip_swing_up_amplitude_ = ui_.dSpinBox_hip_swing_up_amplitude_->value() * DEGREE2RADIAN;
  walking_param.shoulder_swing_amplitude = ui_.dSpinBox_shoulder_swing_amplitude->value();

  qnode_kuroko_.applyWalkingParam(walking_param);
}

void MainWindow::walkingCommandShortcut()
{
  if (is_walking_)
  {
    is_walking_ = false;
    qnode_kuroko_.setWalkingCommand("stop");
  }
  else
  {
    is_walking_ = true;
    qnode_kuroko_.setWalkingCommand("start");
  }
}

/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_action_about_triggered()
{
  QMessageBox::about(this, tr("About"), tr("<h2>Kuroko GUI Demo 0.10</h2><p>Copyright ROBOTIS</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::initModeUnit()
{
  int number_joint = qnode_kuroko_.getJointSize();

  // preset button
  QHBoxLayout* preset_layout = new QHBoxLayout;
  QSignalMapper* signal_mapper = new QSignalMapper(this);

  // yaml preset
  for (auto& module_it : qnode_kuroko_.module_table_)
  {
    std::string preset_name = module_it.second;
    QPushButton* preset_button = new QPushButton(tr(preset_name.c_str()));
    if (debug_)
      std::cout << "name : " << preset_name << std::endl;

    preset_layout->addWidget(preset_button);

    signal_mapper->setMapping(preset_button, preset_button->text());
    QObject::connect(preset_button, SIGNAL(clicked()), signal_mapper, SLOT(map()));
  }

  QObject::connect(signal_mapper, SIGNAL(mapped(QString)), this, SLOT(setMode(QString)));

  ui_.widget_mode_preset->setLayout(preset_layout);

  // joints
  QGridLayout* grid_layout = new QGridLayout;
  for (int ix = 0; ix < number_joint; ix++)
  {
    std::stringstream label_stream;
    std::string joint_name;
    int joint_id;

    if (!qnode_kuroko_.getIDJointNameFromIndex(ix, joint_id, joint_name))
      continue;

    label_stream << "[" << (joint_id < 10 ? "0" : "") << joint_id << "] " << joint_name;
    QLabel* id_label = new QLabel(tr(label_stream.str().c_str()));

    QStringList module_list;
    for (int index = 0; index < qnode_kuroko_.getModeSize(); index++)
    {
      std::string module_name = qnode_kuroko_.getModeName(index);
      if (!module_name.empty())
        module_list << module_name.c_str();
    }

    QComboBox* module_combo = new QComboBox();
    module_combo->setObjectName(tr(joint_name.c_str()));
    module_combo->addItems(module_list);
    module_combo->setEnabled(false);  // not changable
    int num_row = ix / 2 + 1;
    int num_col = (ix % 2) * 3;
    grid_layout->addWidget(id_label, num_row, num_col, 1, 1);
    grid_layout->addWidget(module_combo, num_row, num_col + 1, 1, 2);
  }

  // get/set buttons
  QPushButton* get_mode_button = new QPushButton(tr("Get Mode"));
  grid_layout->addWidget(get_mode_button, (number_joint / 2) + 2, 0, 1, 3);
  QObject::connect(get_mode_button, SIGNAL(clicked(bool)), &qnode_kuroko_, SLOT(getJointControlMode()));

  ui_.widget_mode->setLayout(grid_layout);

  // make module widget table
  for (int index = 0; index < qnode_kuroko_.getModeSize(); index++)
  {
    std::string module_name = qnode_kuroko_.getModeName(index);
    if (module_name.empty())
      continue;
    std::string module_reg = "*_" + module_name;

    QRegExp reg_exp(QRegExp(tr(module_reg.c_str())));
    reg_exp.setPatternSyntax(QRegExp::Wildcard);

    QList<QWidget*> widget_list = ui_.centralwidget->findChildren<QWidget*>(reg_exp);
    module_ui_table_[module_name] = widget_list;

    if (debug_)
      std::cout << "Module widget : " << module_name << " [" << widget_list.size() << "]" << std::endl;
  }

  // make motion tab
  if (qnode_kuroko_.getModeIndex("action_module") != -1)
  {
    std::cout << "Action module" << std::endl;
    initMotionUnit();
  }
  else
  {
    std::cout << "No action module" << std::endl;
  }
}

void MainWindow::initMotionUnit()
{
  // preset button
  QGridLayout* motion_layout = new QGridLayout;
  QSignalMapper* signal_mapper = new QSignalMapper(this);

  // yaml preset
  int index = 0;
  for (auto& motion_it : qnode_kuroko_.motion_table_)
  {
    int motion_index = motion_it.first;
    std::string motion_name = motion_it.second;
    QString q_motion_name = QString::fromStdString(motion_name);
    QPushButton* motion_button = new QPushButton(q_motion_name);

    int button_size = (motion_index < 0) ? 2 : 1;
    int num_row = index / 4;
    int num_col = index % 4;
    motion_layout->addWidget(motion_button, num_row, num_col, 1, button_size);

    // hotkey
    std::map<int, int>::iterator shortcut_it = qnode_kuroko_.motion_shortcut_table_.find(motion_index);
    if (shortcut_it != qnode_kuroko_.motion_shortcut_table_.end())
      motion_button->setShortcut(QKeySequence(shortcut_it->second));

    signal_mapper->setMapping(motion_button, motion_index);
    QObject::connect(motion_button, SIGNAL(clicked()), signal_mapper, SLOT(map()));

    index += button_size;
  }

  int num_row = index / 4;
  num_row = (index % 4 == 0) ? num_row : num_row + 1;
  QSpacerItem* vertical_spacer = new QSpacerItem(20, 400, QSizePolicy::Minimum, QSizePolicy::Expanding);
  motion_layout->addItem(vertical_spacer, num_row, 0, 1, 4);

  QObject::connect(signal_mapper, SIGNAL(mapped(int)), &qnode_kuroko_, SLOT(playMotion(int)));

  ui_.scroll_widget_motion->setLayout(motion_layout);
}

void MainWindow::setMode(const QString& mode_name)
{
  qnode_kuroko_.setControlMode(mode_name.toStdString());
}

void MainWindow::readSettings()
{
  QSettings settings("Qt-Ros Package", "kuroko_walking_gui");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::writeSettings()
{
  QSettings settings("Qt-Ros Package", "kuroko_walking_gui");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

}  // namespace walking_gui
