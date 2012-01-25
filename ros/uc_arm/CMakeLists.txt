CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(RobotArm_ROS_uc)

FIND_PACKAGE(ArexxRobotArm REQUIRED)
INCLUDE("arexx_robot_arm_crosscompile")

#INCLUDE_DIRECTORIES(/opt/ros/electric/stacks/rosserial/rosserial_arduino/libraries/ros_lib)
INCLUDE_DIRECTORIES(${RobotArm_ROS_uc_SOURCE_DIR}/ros_lib)

ADD_ROBOT_ARM_EXECUTABLE(RobotArm_ROS
	RobotArm_ROS.cpp
)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/RobotArm_ROS.hex DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/ros/firmware)
INSTALL(FILES ${RobotArm_ROS_uc_SOURCE_DIR}/RobotArm_ROS.cpp DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/ros/firmware)
INSTALL(FILES ${RobotArm_ROS_uc_SOURCE_DIR}/CMakeLists.txt DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/ros/firmware)
INSTALL(FILES ${RobotArm_ROS_uc_SOURCE_DIR}/ArexxArmHardware.h DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/ros/firmware)


