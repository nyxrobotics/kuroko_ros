cmake_minimum_required(VERSION 2.8.12)
project(kuroko_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
