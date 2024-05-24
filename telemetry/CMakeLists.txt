cmake_minimum_required(VERSION 3.0.2)
project(telemetry)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  debra
)

# Ensure Python setup happens after messages
catkin_python_setup()

# Add message files
add_message_files(
  FILES
  command_msg.msg
)

# Generate message files
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Add catkin package information
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime debra payload
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)


# Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  scripts/telemetry_node.py
  scripts/fake_data_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
