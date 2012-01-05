CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(ArexxRobotArm)

INCLUDE(arexx_robot_arm_crosscompile.cmake)
IF(NOT CMAKE_CROSSCOMPILING)
	ERROR()
ENDIF()

INCLUDE_DIRECTORIES(${ArexxRobotArm_SOURCE_DIR} ${ArexxRobotArm_SOURCE_DIR}/RobotArmBase ${ArexxRobotArm_SOURCE_DIR}/RobotArmI2C)


ADD_LIBRARY(arexx_robot_arm STATIC
	RobotArmBase/RobotArmBaseLib.c
	RobotArmBase/RobotArmUart.c
	RobotArmI2C/I2C_Yeti_Display.c
)

INSTALL(TARGETS arexx_robot_arm ARCHIVE DESTINATION share/robotloader/examples/RobotArmExamples)
INSTALL(FILES ../../cmake/FindArexxRobotArm.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cmake-2.8/Modules)

