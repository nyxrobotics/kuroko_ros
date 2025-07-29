cmake_minimum_required(VERSION 3.0.2)
project(kuroko_roboone)

find_package(Eigen3 REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             jsk_recognition_msgs
             robotis_controller_msgs
             message_filters
             kuroko_walking_module_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    jsk_recognition_msgs
    message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/roboone_auto.cpp src/main.cpp)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
