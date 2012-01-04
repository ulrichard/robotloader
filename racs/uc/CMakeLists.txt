CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(RobotArm_RACSQT)

FIND_PACKAGE(ArexxRobotArm REQUIRED)

INCLUDE(arexx_robot_arm_crosscompile.cmake)
INCLUDE_DIRECTORIES(${ArexxRobotArm_INCLUDE_DIRS})

ADD_EXECUTABLE(RobotArm_RACSQT
	RobotArm_RACSQT.c
	# todo : get the linking with the static lib working and remove the inclusion of the lib sources here
	${ArexxRobotArm_INCLUDE_DIR}/RobotArmBase/RobotArmBaseLib.c
	${ArexxRobotArm_INCLUDE_DIR}/RobotArmBase/RobotArmUart.c
)

LINK_LIBRARIES(RobotArm_RACSQT
	${ArexxRobotArm_LIBRARY}
)

INSTALL(TARGETS RobotArm_RACSQT RUNTIME DESTINATION share/robotloader/examples/racsqt)

