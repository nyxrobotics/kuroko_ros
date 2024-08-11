cmake_minimum_required(VERSION 2.8.12)
project(kuroko_description)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Add executables for the C++ nodes
add_executable(publish_trajectory src/publish_trajectory.cpp)
target_link_libraries(publish_trajectory ${catkin_LIBRARIES})

add_executable(publish_position src/publish_position.cpp)
target_link_libraries(publish_position ${catkin_LIBRARIES})

# Install targets
install(
  TARGETS publish_trajectory publish_position
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
