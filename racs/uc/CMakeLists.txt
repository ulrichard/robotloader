CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(RobotArm_RACSQT_uc)

FIND_PACKAGE(ArexxRobotArm REQUIRED)

INCLUDE(arexx_robot_arm_crosscompile.cmake)
INCLUDE_DIRECTORIES(${ArexxRobotArm_INCLUDE_DIRS})

ADD_EXECUTABLE(RobotArm_RACSQT.elf
	RobotArm_RACSQT.c

	# if you don't trust the static lib, maybe because you have another microprocessor, you can instead compile the lib sources directly in your prroject:
#	${ArexxRobotArm_SOURCES}
)

TARGET_LINK_LIBRARIES(RobotArm_RACSQT.elf
	${ArexxRobotArm_LIBRARY}
)

IF(CMAKE_OBJCOPY)
    ADD_CUSTOM_TARGET(RobotArm_RACSQT.hex)
    ADD_DEPENDENCIES(RobotArm_RACSQT.hex RobotArm_RACSQT.elf)

    ADD_CUSTOM_COMMAND(TARGET RobotArm_RACSQT.hex POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom ${CMAKE_CURRENT_BINARY_DIR}/RobotArm_RACSQT.elf ${CMAKE_CURRENT_BINARY_DIR}/RobotArm_RACSQT.hex
    )
ENDIF()

#IF(AVRDUDE)
#    ADD_CUSTOM_TARGET(flash)
#    ADD_DEPENDENCIES(flash RobotArm_RACSQT.hex)

#    ADD_CUSTOM_COMMAND(TARGET flash POST_BUILD
#        COMMAND ${AVRDUDE} -P ${PORT} -b ${ARDUINO_UPLOAD_SPEED} -c ${ARDUINO_PROTOCOL} -p ${ARDUINO_MCU} -V -F -U flash:w:RobotArm_RACSQT.hex:i
#    )
#ENDIF()

#INSTALL(TARGETS RobotArm_RACSQT.hex RUNTIME DESTINATION share/robotloader/examples/racsqt)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/RobotArm_RACSQT.hex DESTINATION ${CMAKE_INSTALL_PREFIX}/share/robotloader/examples/racsqt)


