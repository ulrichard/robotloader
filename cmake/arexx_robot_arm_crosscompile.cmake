SET(CMAKE_SYSTEM_NAME Generic)

SET(CMAKE_C_COMPILER /usr/bin/avr-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/avr-g++)
SET(CMAKE_AR /usr/bin/avr-ar)
SET(CMAKE_LINKER /usr/bin/avr-ld)
SET(CMAKE_NM /usr/bin/avr-nm)
SET(CMAKE_OBJCOPY /usr/bin/avr-objcopy)
SET(CMAKE_OBJDUMP /usr/bin/avr-objdump)
SET(CMAKE_RANLIB /usr/bin/avr-ranlib)
SET(CMAKE_STRIP /usr/bin/avr-strip)

set(CMAKE_EXE_LINKER_FLAGS "-static")

SET(CSTANDARD "-std=gnu99")
SET(CDEBUG "-gstabs")
SET(CWARN "-Wall -Wstrict-prototypes")
SET(CTUNING "-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums")
SET(COPT "-Os")
SET(CINCS "")
SET(CMCU "-mmcu=atmega64")
SET(CDEFS "-DF_CPU=16384000")

SET(CFLAGS "${CMCU} ${CDEBUG} ${CDEFS} ${CINCS} ${COPT} ${CWARN} ${CSTANDARD} ${CEXTRA}")
SET(CXXFLAGS "${CMCU} ${CDEFS} ${CINCS} ${COPT}")

SET(CMAKE_C_FLAGS  ${CFLAGS})
SET(CMAKE_CXX_FLAGS ${CXXFLAGS})

IF(NOT CMAKE_OBJCOPY)
	ERROR()
ENDIF()

MACRO(ADD_ROBOT_ARM_EXECUTABLE ROBOT_ARM_EXE_NAME)
	INCLUDE_DIRECTORIES(${ArexxRobotArm_INCLUDE_DIRS})

	ADD_EXECUTABLE(${ROBOT_ARM_EXE_NAME}.elf
		${ARGV1}
		${ARGV2}
		${ARGV3}
		${ARGV4}
		${ARGV5}
		${ARGV6}
		${ARGV7}
		${ARGV8}
		${ARGV9}

		# if you don't trust the static lib, maybe because you have another microprocessor, you can instead compile the lib sources directly in your prroject:
#		${ArexxRobotArm_SOURCES}
	)

	TARGET_LINK_LIBRARIES(${ROBOT_ARM_EXE_NAME}.elf
		${ArexxRobotArm_LIBRARY}
	)


	ADD_CUSTOM_COMMAND(TARGET ${ROBOT_ARM_EXE_NAME}.elf POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom ${ROBOT_ARM_EXE_NAME}.elf ${ROBOT_ARM_EXE_NAME}.hex
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	)
ENDMACRO(ADD_ROBOT_ARM_EXECUTABLE)

#IF(AVRDUDE)
#    ADD_CUSTOM_TARGET(flash)
#    ADD_DEPENDENCIES(flash RobotArm_RACSQT.hex)

#    ADD_CUSTOM_COMMAND(TARGET flash POST_BUILD
#        COMMAND ${AVRDUDE} -P ${PORT} -b ${ARDUINO_UPLOAD_SPEED} -c ${ARDUINO_PROTOCOL} -p ${ARDUINO_MCU} -V -F -U flash:w:RobotArm_RACSQT.hex:i
#    )
#ENDIF()

