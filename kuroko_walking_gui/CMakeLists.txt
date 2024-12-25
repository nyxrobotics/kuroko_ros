##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.0.2)
project(kuroko_walking_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(
  catkin REQUIRED
  COMPONENTS rqt_gui
             rqt_gui_cpp
             roscpp
             roslib
             std_msgs
             sensor_msgs
             geometry_msgs
             visualization_msgs
             interactive_markers
             cmake_modules
             eigen_conversions
             kuroko_action_module
             robotis_controller_msgs
             op3_walking_module_msgs
             op3_online_walking_module_msgs
             humanoid_nav_msgs
             tf
)

find_package(Eigen3 REQUIRED)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} "
   STREQUAL
   "5 "
)
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(
    Qt4
    COMPONENTS QtCore QtGui
    REQUIRED
  )
  include(${QT_USE_FILE})
endif()

set(kuroko_walking_gui_SRCS
    src/main.cpp
    src/main_window.cpp
    src/preview_walking_form.cpp
    src/qnode.cpp
    src/qnode_default_demo.cpp
    src/qnode_preview_walking.cpp
)

set(kuroko_walking_gui_HDRS
    include/kuroko_walking_gui/main_window.hpp
    include/kuroko_walking_gui/preview_walking_form.h
    include/kuroko_walking_gui/qnode.hpp
)

set(kuroko_walking_gui_UIS ui/main_window.ui ui/preview_walking_form.ui)

set(kuroko_walking_gui_RESOURCES resources/images.qrc)

set(kuroko_walking_gui_INCLUDE_DIRECTORIES
    include
    include/${PROJECT_NAME}
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)
if(NOT
   EXISTS
   "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
endif()

catkin_package(
  INCLUDE_DIRS ${kuroko_walking_gui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rqt_gui
    rqt_gui_cpp
    roscpp
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    interactive_markers
    cmake_modules
    eigen_conversions
    kuroko_action_module
    robotis_controller_msgs
    op3_walking_module_msgs
    op3_online_walking_module_msgs
    humanoid_nav_msgs
)
#catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} "
   STREQUAL
   "5 "
)
  qt5_wrap_cpp(kuroko_walking_gui_MOCS ${kuroko_walking_gui_HDRS})
else()
  qt4_wrap_cpp(kuroko_walking_gui_MOCS ${kuroko_walking_gui_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} "
   STREQUAL
   "5 "
)
  qt5_wrap_ui(kuroko_walking_gui_UIS_H ${kuroko_walking_gui_UIS})
else()
  qt4_wrap_ui(kuroko_walking_gui_UIS_H ${kuroko_walking_gui_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} "
   STREQUAL
   "5 "
)
  qt5_add_resources(kuroko_walking_gui_RCC ${kuroko_walking_gui_RESOURCES})
else()
  qt4_add_resources(kuroko_walking_gui_RCC ${kuroko_walking_gui_RESOURCES})
endif()

include_directories(
  ${kuroko_walking_gui_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
add_executable(
  ${PROJECT_NAME}
  ${kuroko_walking_gui_SRCS}
  ${kuroko_walking_gui_MOCS}
  ${kuroko_walking_gui_UIS_H}
  ${kuroko_walking_gui_RCC}
)
add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} "
   STREQUAL
   "5 "
)
  target_link_libraries(
    ${PROJECT_NAME}
    Qt5::Widgets
    yaml-cpp
  )
else()
  target_link_libraries(
    ${PROJECT_NAME}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    yaml-cpp
  )
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/kuroko_walking_gui/*.hpp include/kuroko_walking_gui/*.h)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

#add_executable(kuroko_walking_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#add_dependencies(kuroko_walking_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(kuroko_walking_gui
#  ${QT_LIBRARIES}
#  ${catkin_LIBRARIES}
#  yaml-cpp)
#install(TARGETS kuroko_walking_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
