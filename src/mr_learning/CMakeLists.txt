cmake_minimum_required(VERSION 3.0.2)
project(mr_learning)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/dqn_train.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
