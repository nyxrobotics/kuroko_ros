#ifndef KUROKO_WALKING_GUI_MAIN_WINDOW_H
#define KUROKO_WALKING_GUI_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include "qnode.hpp"
#include "ui_main_window.h"
#include <QMainWindow>

#endif
/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace walking_gui
{
#define DEGREE2RADIAN (M_PI / 180.0)
#define RADIAN2DEGREE (180.0 / M_PI)

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = nullptr);
  ~MainWindow() override;

  void readSettings();   // Load up qt program settings at startup
  void writeSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent* event) override;  // Overloaded function
  void showNoMasterMessage();

public Q_SLOTS:

  /******************************************
   ** Auto-connections (connectSlotsByName())
   *******************************************/
  void on_action_about_triggered();
  void on_button_clear_log_clicked(bool check);
  void on_button_init_pose_clicked(bool check);

  // Walking
  void on_button_init_gyro_clicked(bool check);
  void on_button_walking_start_clicked(bool check);
  void on_button_walking_stop_clicked(bool check);

  void on_button_param_refresh_clicked(bool check);
  void on_button_param_apply_clicked(bool check);
  void on_button_param_save_clicked(bool check);

  void on_checkBox_balance_on_clicked(bool check);
  void on_checkBox_balance_off_clicked(bool check);

  // Head Control
  void on_head_center_button_clicked(bool check);

  // Demo
  void on_button_demo_start_clicked(bool check);
  void on_button_demo_stop_clicked(bool check);
  void on_button_r_kick_clicked(bool check);
  void on_button_l_kick_clicked(bool check);
  void on_button_getup_front_clicked(bool check);
  void on_button_getup_back_clicked(bool check);

  /******************************************
   ** Manual connections
   *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically
  void setMode(bool check);
  void updateCurrentJointMode(std::vector<int> mode);
  void setMode(const QString& mode_name);

  // Head Control
  void updateHeadAngles(double pan, double tilt);

  // Walking
  void updateWalkingParams(op3_walking_module_msgs::WalkingParam params);
  void walkingCommandShortcut();

protected Q_SLOTS:
  void setHeadAngle();

private:
  enum MotionIndex
  {
    INITIAL_POSE = 1,
    GETUP_FRONT = 2,
    GETUP_REAR = 3,
    L_GRIP_FRONT = 4,
    L_HOOK_FRONT = 5,
    L_PUNCH_HIGH = 6,
    L_PUNCH_LOW = 7,
    R_GRIP_FRONT = 8,
    R_HOOK_FRONT = 9,
    R_PUNCH_HIGH = 10,
    R_PUNCH_LOW = 11,
    WALK_READY = 12,
    RIGHT_KICK = 13,
    LEFT_KICK = 14,
    CEREMONY = 15,
  };

  void setUserShortcut();
  void initModeUnit();
  void initMotionUnit();

  void updateModuleUI();
  void setHeadAngle(double pan, double tilt);
  void applyWalkingParams();

  Ui::MainWindowDesign ui_;
  QNodeKuroko qnode_kuroko_;
  bool debug_;

  bool is_updating_;
  bool is_walking_;
  std::map<std::string, QList<QWidget*>> module_ui_table_;
};

}  // namespace walking_gui

#endif  // KUROKO_WALKING_GUI_MAIN_WINDOW_H
