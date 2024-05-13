cmake_minimum_required(VERSION 3.0.2)
project(seawolf)

find_package(catkin REQUIRED COMPONENTS
    rospy
    hullbot_msgs
    hb_common_msgs
    diagnostics_tools
    mavros_msgs
    h6_bringup
    actionlib
)

catkin_python_setup()


catkin_package(
    CATKIN_DEPENDS
    rospy
    hullbot_msgs
    hb_drivers_msgs
    hb_control_msgs
    hb_common_msgs
    hb_localisation_msgs
    diagnostics_tools
    mavros_msgs
    h6_bringup
    smach_msgs
    actionlib
)

catkin_install_python(PROGRAMS
    scripts/TopsideApp.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  assets/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/assets
)

install(DIRECTORY
  rqt/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rqt
)


install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY
  kivy/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/kivy
)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

if (CATKIN_ENABLE_TESTING)
  ## Add gtest based cpp test target and link libraries
  # catkin_add_gtest(${PROJECT_NAME}-test test/cpp_control_handler_test.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test
    # ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  endif()
  # Rostests
  find_package(rostest REQUIRED)



  ## Add folders to be run by python nosetests
  catkin_add_nosetests(test)

  # Create a target ${PROJECT_NAME}_coverage_report
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
    )
  endif()
endif()
