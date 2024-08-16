################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(kuroko_device_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             robotis_controller_msgs
             op3_walking_module_msgs
             op3_online_walking_module_msgs
             cmake_modules
             dynamixel_sdk
             robotis_framework_common
             robotis_device
             kuroko_joint_controller
             robotis_math
             kuroko_kinematics
             kuroko_walking_module
             kuroko_online_walking_module
             kuroko_initial_pose_module
             kuroko_tuning_module
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    robotis_controller_msgs
    kuroko_action_module_msgs
    op3_walking_module_msgs
    op3_online_walking_module_msgs
    cmake_modules
    dynamixel_sdk
    robotis_framework_common
    robotis_device
    kuroko_joint_controller
    robotis_math
    kuroko_kinematics
    kuroko_balance_control
    open_cr_module
    kuroko_initial_pose_module
    kuroko_head_control_module
    kuroko_walking_module
    kuroko_action_module
    kuroko_direct_control_module
    kuroko_online_walking_module
    kuroko_tuning_module
    op3_tuning_module_msgs
)

################################################################################
# Build
################################################################################
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/kuroko_device_manager.cpp src/main.cpp)
add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
