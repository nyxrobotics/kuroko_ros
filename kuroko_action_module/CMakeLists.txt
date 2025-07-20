cmake_minimum_required(VERSION 3.0.2)
project(kuroko_action_module)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             roslib
             std_msgs
             robotis_controller_msgs
             op3_action_module_msgs
             robotis_framework_common
             robotis_device
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    robotis_controller_msgs
    op3_action_module_msgs
    robotis_framework_common
    robotis_device
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/action_module.cpp
  src/animation_files.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Check if C++17 or higher is available
if(CMAKE_CXX_STANDARD
   GREATER_EQUAL
   17
)
  message(STATUS "Using C++17, linking with std::filesystem")
else()
  message(STATUS "Using C++11, linking with std::experimental::filesystem")
  target_link_libraries(${PROJECT_NAME} stdc++fs)
endif()

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

install(
  DIRECTORY motion
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
