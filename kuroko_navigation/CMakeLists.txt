cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(
    SET
    CMP0048
    NEW
  )
endif(POLICY CMP0048)

project(kuroko_navigation)
find_package(catkin REQUIRED)
catkin_package()
install(
  DIRECTORY config
            launch
            maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
