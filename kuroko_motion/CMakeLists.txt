cmake_minimum_required(VERSION 3.0.2)
project(kuroko_motion)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             joint_trajectory_controller
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Define macros
macro(
  SUBDIRLIST
  result
  curdir
)
  file(
    GLOB children
    RELATIVE ${curdir}
    ${curdir}/*
  )
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(
        APPEND
        dirlist
        ${child}
      )
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Get node names from subdirectories in src
subdirlist(SUBDIRS "${CMAKE_CURRENT_SOURCE_DIR}/src")
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
  target_include_directories(${NODENAME} PUBLIC include/${NODENAME})
  target_link_libraries(${NODENAME} ${catkin_LIBRARIES})
endforeach()
