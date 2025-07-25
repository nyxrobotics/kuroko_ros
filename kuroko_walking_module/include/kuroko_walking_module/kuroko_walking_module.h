#ifndef KUROKO_WALKING_MODULE_H_
#define KUROKO_WALKING_MODULE_H_

#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
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

#include "kuroko_walking_module_msgs/GetWalkingParam.h"
#include "kuroko_walking_module_msgs/SetWalkingParam.h"
#include "kuroko_walking_module_msgs/WalkingParam.h"
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
    WALK_DISABLE = 0,
    WALK_ENABLE = 1,
    WALK_INITIAL_POSE = 2,
    WALK_READY = 3
  };

  const bool debug_;

  void queueThread();

  /* ROS Topic Callback Functions */
  void walkingCommandCallback(const std_msgs::String::ConstPtr& msg);
  void walkingParameterCallback(const kuroko_walking_module_msgs::WalkingParam::ConstPtr& msg);
  bool getWalkigParameterCallback(kuroko_walking_module_msgs::GetWalkingParam::Request& req,
                                  kuroko_walking_module_msgs::GetWalkingParam::Response& res);

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
  Eigen::Vector3d quaterionToRpy(const Eigen::Quaterniond& q);
  Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d& rpy);

  KurokoKinematics* kuroko_kinematics_;
  int control_cycle_msec_;
  std::string param_path_;
  boost::thread queue_thread_;
  boost::mutex publish_mutex_;

  /* ROS Topic Publish Functions */
  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;
  kuroko_walking_module_msgs::WalkingParam walking_param_;

  Eigen::MatrixXd calc_joint_trajectory_;

  Eigen::MatrixXd target_position_;  // Target values for joint angles inside the gait program
  Eigen::MatrixXd goal_position_;    // Target angles currently set for Servo Motors
  Eigen::MatrixXd init_position_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;
  int walking_state_;
  int init_pose_count_;

  // Leg parameters
  double leg_default_length_;
  double leg_default_separaion_;

  // variable for walking
  double walk_period_;
  double dsp_ratio_;
  double l_ssp_start_time_;
  double l_ssp_end_time_;
  double r_ssp_start_time_;
  double r_ssp_end_time_;
  double phase1_time_;
  double phase2_time_;
  double phase3_time_;

  double init_x_offset_;
  double init_y_offset_;
  double init_z_offset_;
  double init_roll_offset_;
  double init_pitch_offset_;
  double init_yaw_offset_;
  double init_hip_pitch_offset_;

  double x_swing_amplitude_;
  double y_swing_amplitude_;
  double z_swing_amplitude_;
  double roll_swing_amplitude_;

  double step_length_x_;
  double step_length_y_;
  double step_length_yaw_;
  double foot_lift_height_;

  double previous_step_length_x_;
  double previous_step_length_y_;
  double previous_step_length_yaw_;
  double previous_foot_lift_height_;

  double step_y_accel_max_;
  double step_x_accel_max_;
  double step_yaw_accel_max_;

  double hip_swing_up_amplitude_;
  double hip_swing_down_amplitude_;
  double chest_swing_amplitude_;
  double shoulder_swing_amplitude_;

  bool request_walk_;
  bool is_walking_;
  double time_;

  int phase_;
  double body_swing_y_;
  double body_swing_z_;
};

}  // namespace motion_control

#endif /* KUROKO_WALKING_MODULE_H_ */
