#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "home_position_publisher");
  ros::NodeHandle nh;

  // Load parameters from the motion_player/home_position namespace
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  double node_start_delay;

  nh.getParam("motion_player/home_position/joint_names", joint_names);
  nh.getParam("motion_player/home_position/positions", positions);
  nh.param("node_start_delay", node_start_delay, 0.1);

  // Insert delay
  ros::Duration(node_start_delay).sleep();

  // Create the message
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = joint_names;
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = ros::Duration(1.0);
  traj.points.push_back(point);

  // Publish to the topic
  ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/kuroko/trajectory_controller/command", 5);

  // Wait until the publisher is ready
  while (pub.getNumSubscribers() == 0 && ros::ok())
  {
    ROS_WARN_ONCE("Waiting for a subscriber to connect to /kuroko/trajectory_controller/command");
    ros::Duration(0.1).sleep();
  }

  // Publish the trajectory
  pub.publish(traj);
  ROS_INFO("Published home position trajectory.");

  return 0;
}
