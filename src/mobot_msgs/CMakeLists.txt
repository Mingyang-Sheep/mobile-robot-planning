cmake_minimum_required(VERSION 3.0.2)
project(mobot_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  RLState.msg
  RLReward.msg
)

add_action_files(
  FILES
  GlobalPlan.action
)

add_service_files(
  FILES
  SwitchPlanner.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    nav_msgs
    actionlib_msgs
)

