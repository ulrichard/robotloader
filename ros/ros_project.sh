#! /bin/sh
# This file is just to help me remember the ros commands as I'm very new to ros...
# http://www.ros.org/wiki/urdf/Tutorials/Create%20your%20own%20urdf%20file
# http://www.ros.org/wiki/urdf/Tutorials/Parse%20a%20urdf%20file

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/richi/sourcecode/robotloader/ros

#roscreate-pkg arexx_robotarm urdf
roscd arexx_robotarm
#rosmake

make
bin/parser arexx_mini_urdf.xml
