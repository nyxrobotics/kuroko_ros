cmake_minimum_required(VERSION 2.8.12)
project(kuroko_motion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  joint_trajectory_controller
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Define macros
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()
# Get node names from subdirectories in src
SUBDIRLIST(SUBDIRS "${CMAKE_CURRENT_SOURCE_DIR}/src")
message(STATUS "SUBDIRS:${SUBDIRS}")
message(STATUS "Showing nodes....")
foreach(NODENAME IN LISTS SUBDIRS)
  message(STATUS "NODENAME:${NODENAME}")
  file(
    GLOB
    MY_SOURCES
    "src/${NODENAME}/*.cpp"
  )
  message(STATUS "MY_SOURCES:${MY_SOURCES}")
  add_executable(${NODENAME} ${MY_SOURCES})
  target_include_directories(${NODENAME}
    PUBLIC
    include/${NODENAME}
  )
  target_link_libraries(${NODENAME} ${catkin_LIBRARIES})
endforeach()
