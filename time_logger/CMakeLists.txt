cmake_minimum_required(VERSION 3.0.2)
project(time_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/time_logger_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
