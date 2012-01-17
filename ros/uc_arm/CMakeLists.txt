CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(RobotArm_RACSQT_uc)

FIND_PACKAGE(ArexxRobotArm REQUIRED)
INCLUDE(${CMAKE_ROOT}/Modules/arexx_robot_arm_crosscompile.cmake)

INCLUDE_DIRECTORIES(/opt/ros/electric/stacks/rosserial/rosserial_arduino/libraries/ros_lib)

ADD_ROBOT_ARM_EXECUTABLE(RobotArm_RACSQT
	RobotArm_ROS.cpp
)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/RobotArm_ROS.hex DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/examples/ros)
INSTALL(FILES ${RobotArm_RACSQT_uc_SOURCE_DIR}/RobotArm_ROS.c DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/examples/ros)
INSTALL(FILES ${RobotArm_RACSQT_uc_SOURCE_DIR}/CMakeLists.txt DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/examples/ros)


