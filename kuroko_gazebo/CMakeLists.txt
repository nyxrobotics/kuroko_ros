cmake_minimum_required(VERSION 3.0.2)
project(kuroko_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Make sure the Python script is installed and marked executable
catkin_install_python(PROGRAMS
  scripts/unpause_physics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
