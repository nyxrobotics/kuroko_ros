#include <ros/ros.h>
#include <std_msgs/Float64.h>
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

  // Create a vector of publishers for each joint
  std::vector<ros::Publisher> publishers;
  for (const auto& joint_name : joint_names)
  {
    std::string topic_name = "/kuroko/" + joint_name + "_position_controller/command";
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(topic_name, 10);
    publishers.push_back(pub);
  }

  // Wait until all publishers are ready
  for (const auto& pub : publishers)
  {
    while (pub.getNumSubscribers() == 0 && ros::ok())
    {
      ROS_WARN_ONCE("Waiting for subscribers to connect to joint controllers.");
      ros::Duration(0.1).sleep();
    }
  }

  // Publish the positions to the respective topics
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    std_msgs::Float64 msg;
    msg.data = positions[i];
    publishers[i].publish(msg);
  }

  ROS_INFO("Published home positions to all joint controllers.");

  return 0;
}
