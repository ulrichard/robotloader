# - Locate ArexxRobotArm library
# This module defines
#  ArexxRobotArm_LIBRARY, the library to link against
#  ArexxRobotArm_FOUND, if false, do not try to link to FTGL
#  ArexxRobotArm_INCLUDE_DIR, where to find headers.
#
#=============================================================================
# Copyright 2011 Richard Ulrich.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

FIND_PATH(ArexxRobotArm_INCLUDE_DIR RobotArmBase/RobotArmBase.h 
  HINTS
  $ENV{ArexxRobotArm_DIR}
  PATH_SUFFIXES include src
  PATHS
  /usr/include
  /usr/local/include
  /sw/include
  /opt/local/include
  /usr/freeware/include
  /usr/share/robotloader/examples/RobotArmLib
)

FIND_LIBRARY(ArexxRobotArm_LIBRARY
  NAMES arexx_robot_arm
  HINTS
  $ENV{ArexxRobotArm_DIR}
  PATH_SUFFIXES lib64 lib
  PATHS
  /usr/lib
  /usr/local/lib
  /sw
  /usr/freeware
  /usr/share/robotloader/examples/RobotArmLib
)

# handle the QUIETLY and REQUIRED arguments and set FTGL_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE("FindPackageHandleStandardArgs")
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ArexxRobotArm  DEFAULT_MSG  ArexxRobotArm_LIBRARY  ArexxRobotArm_INCLUDE_DIR)

MARK_AS_ADVANCED(ArexxRobotArm_LIBRARY ArexxRobotArm_INCLUDE_DIR)

