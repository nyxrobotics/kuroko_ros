cmake_minimum_required(VERSION 3.0.2)
project(kuroko_description)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
