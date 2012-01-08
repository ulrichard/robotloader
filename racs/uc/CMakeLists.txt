CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(RobotArm_RACSQT_uc)

FIND_PACKAGE(ArexxRobotArm REQUIRED)

INCLUDE(arexx_robot_arm_crosscompile.cmake)

ADD_ROBOT_ARM_EXECUTABLE(RobotArm_RACSQT
	RobotArm_RACSQT.c
)

INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/RobotArm_RACSQT.hex DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/examples/racsqt)
INSTALL(FILES ${RobotArm_RACSQT_uc_SOURCE_DIR}/RobotArm_RACSQT.c DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/examples/racsqt)
INSTALL(FILES ${RobotArm_RACSQT_uc_SOURCE_DIR}/CMakeLists.txt DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/examples/racsqt)


