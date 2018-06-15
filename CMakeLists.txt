cmake_minimum_required(VERSION 2.8.3)
project(asv_localization)

find_package(catkin REQUIRED COMPONENTS
  asv_sensors
  nav_msgs
  rospy
  sensor_msgs
  geometry_msgs
  tf2_ros
  gmapping
  map_server
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs tf2_ros nav_msgs gmapping map_server
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)