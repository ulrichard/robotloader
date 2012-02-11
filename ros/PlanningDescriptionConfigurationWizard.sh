#! /bin/sh
# This file is just to help me remember the ros commands as I'm very new to ros...

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/richi/sourcecode/robotloader/ros

# launch the wizard
# http://www.ros.org/wiki/arm_navigation/Tutorials/tools/Planning%20Description%20Configuration%20Wizard
roslaunch planning_environment planning_description_configuration_wizard.launch urdf_package:=arexx_robotarm urdf_path:=arexx_mini_urdf.xml

# launch the visualizer
# http://www.ros.org/wiki/arm_navigation/Tutorials/tools/Planning%20Components%20Visualizer
roslaunch arexx_mini_arm_navigation planning_components_visualizer.launch

