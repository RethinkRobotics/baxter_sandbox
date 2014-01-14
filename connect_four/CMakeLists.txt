cmake_minimum_required(VERSION 2.8.3)
project(connect_four)

find_package(catkin
  REQUIRED
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  baxter_core_msgs
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  baxter_core_msgs
  baxter_interface
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
