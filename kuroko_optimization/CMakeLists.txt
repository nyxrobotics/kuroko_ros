cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(
    SET
    CMP0048
    NEW
  )
endif(POLICY CMP0048)
project(kuroko_optimization)

################################################################################
# Find packages
################################################################################
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             robotis_math
             scilab_optimization
             kuroko_online_walking_module_msgs
)

################################################################################
# Include paths
################################################################################
include_directories(include ${catkin_INCLUDE_DIRS})

################################################################################
# Declare catkin package
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    robotis_math
    scilab_optimization
    kuroko_online_walking_module_msgs
)

################################################################################
# Build node
################################################################################
add_executable(${PROJECT_NAME}_node src/kuroko_optimization.cpp src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
