cmake_minimum_required(VERSION 3.0.2)
project(mr_msgs)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
