cmake_minimum_required(VERSION 3.0.2)
project(kuroko_upper_body_module)
add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             robotis_controller_msgs
             cmake_modules
             robotis_framework_common
             robotis_device
             robotis_math
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    robotis_controller_msgs
    cmake_modules
    robotis_framework_common
    robotis_device
    robotis_math
  DEPENDS Boost EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/kuroko_upper_body_module.cpp)
add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
