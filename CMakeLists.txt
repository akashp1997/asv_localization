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
)