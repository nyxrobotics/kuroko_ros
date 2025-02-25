#ifndef KUROKO_WALKING_MODULE_H_
#define KUROKO_WALKING_MODULE_H_

#include "kuroko_walking_parameter.h"

#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <eigen_conversions/eigen_msg.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "kuroko_kinematics/kuroko_kinematics.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_trajectory_calculator.h"

namespace motion_control
{
typedef struct
{
  double x_, y_, z_;
} Position3D;

typedef struct
{
  double x_, y_, z_, roll_, pitch_, yaw_;
} Pose3D;

class WalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkingModule>
{
public:
  enum
  {
    PHASE0 = 0,
    PHASE1 = 1,
    PHASE2 = 2,
    PHASE3 = 3
  };

  WalkingModule();
  ~WalkingModule() override;

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, double> sensors) override;
  void stop() override;
  bool isRunning() override;
  void onModuleEnable() override;
  void onModuleDisable() override;

  int getCurrentPhase()
  {
    return phase_;
  }
  double getBodySwingY()
  {
    return body_swing_y_;
  }
  double getBodySwingZ()
  {
    return body_swing_z_;
  }

private:
  enum
  {
    WALKING_DISABLE = 0,
    WALKING_ENABLE = 1,
    WALKING_INIT_POSE = 2,
    WALKING_READY = 3
  };

  const bool debug_;

  void queueThread();

  /* ROS Topic Callback Functions */
  void walkingCommandCallback(const std_msgs::String::ConstPtr& msg);
  void walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg);
  bool getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request& req,
                                  op3_walking_module_msgs::GetWalkingParam::Response& res);

  /* ROS Service Callback Functions */
  void processPhase(const double& time_unit);
  bool updateLegTargetAngles(std::vector<double>& leg_joints);
  void gyroFeedback(const double& roll_gyro_err, const double& pitch_gyro_err, std::vector<double>& balance_angle);

  void publishStatusMsg(unsigned int type, std::string msg);
  double wSin(double time, double period, double period_shift, double mag, double mag_shift);
  void updateTimeParam();
  void updateMovementParam();
  void updatePoseParam();
  void startWalking();
  void loadWalkingParam(const std::string& path);
  void saveWalkingParam(std::string& path);
  void iniPoseTraGene(double mov_time);

  KurokoKinematics* kuroko_kinematics_;
  int control_cycle_msec_;
  std::string param_path_;
  boost::thread queue_thread_;
  boost::mutex publish_mutex_;

  /* ROS Topic Publish Functions */
  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;

  Eigen::MatrixXd calc_joint_trajectory_;

  Eigen::MatrixXd target_position_;  // Target values for joint angles inside the gait program
  Eigen::MatrixXd goal_position_;    // Target angles currently set for Servo Motors
  Eigen::MatrixXd init_position_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;
  int walking_state_;
  int init_pose_count_;
  op3_walking_module_msgs::WalkingParam walking_param_;
  double previous_x_move_amplitude_;

  // Leg parameters
  double leg_default_length_;
  double leg_default_separaion_;

  // variable for walking
  double period_time_;
  double dsp_ratio_;
  double ssp_ratio_;
  double x_swap_period_time_;
  double x_move_period_time_;
  double y_swap_period_time_;
  double y_move_period_time_;
  double z_swap_period_time_;
  double z_move_period_time_;
  double a_move_period_time_;
  double ssp_time_;
  double l_ssp_start_time_;
  double l_ssp_end_time_;
  double r_ssp_start_time_;
  double r_ssp_end_time_;
  double phase1_time_;
  double phase2_time_;
  double phase3_time_;

  double x_offset_;
  double y_offset_;
  double z_offset_;
  double r_offset_;
  double p_offset_;
  double a_offset_;

  double x_swap_phase_shift_;
  double x_swap_amplitude_;
  double x_swap_amplitude_shift_;
  double x_move_phase_shift_;
  double x_move_amplitude_;
  double x_move_amplitude_shift_;
  double y_swap_phase_shift_;
  double y_swap_amplitude_;
  double y_swap_amplitude_shift_;
  double y_move_phase_shift_;
  double y_move_amplitude_;
  double y_move_amplitude_shift_;
  double z_swap_phase_shift_;
  double z_swap_amplitude_;
  double z_swap_amplitude_shift_;
  double z_move_phase_shift_;
  double z_move_amplitude_;
  double z_move_amplitude_shift_;
  double a_move_phase_shift_;
  double a_move_amplitude_;
  double a_move_amplitude_shift_;

  double pelvis_offset_;
  double pelvis_swing_;
  double hit_pitch_offset_;
  double arm_swing_gain_;

  bool ctrl_running_;
  bool real_running_;
  double time_;

  int phase_;
  double body_swing_y_;
  double body_swing_z_;
};

}  // namespace motion_control

#endif /* KUROKO_WALKING_MODULE_H_ */
