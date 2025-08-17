cmake_minimum_required(VERSION 3.0.2)
project(tello_llm_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rviz
  sensor_msgs
  std_msgs
  cv_bridge
  tf2_ros
  message_generation
  actionlib_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  Move.srv
  LLMQuery.srv
)

add_action_files(
  FILES
  ExecuteTask.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tello_llm_ros
 CATKIN_DEPENDS geometry_msgs roscpp rospy rviz sensor_msgs std_msgs message_runtime
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
