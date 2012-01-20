#! /bin/sh
# This file is just to help me remember the ros commands as I'm very new to ros...
# http://www.ros.org/wiki/electric/Installation/Ubuntu
# http://www.ros.org/wiki/urdf/Tutorials/Create%20your%20own%20urdf%20file
# http://www.ros.org/wiki/urdf/Tutorials/Parse%20a%20urdf%20file

#sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu oneiric main" > /etc/apt/sources.list.d/ros-latest.list'
#wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
#sudo apt-get update
#sudo apt-get install ros-electric-ros-base
#echo "source /opt/ros/electric/setup.bash" >> ~/.bashrc
#. ~/.bashrc

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/richi/sourcecode/robotloader/ros

#roscreate-pkg arexx_robotarm urdf
roscd arexx_robotarm
#rosmake

make
bin/parser arexx_mini_urdf.xml

# show the model in Rviz
roslaunch urdf_tutorial display.launch model:=arexx_mini_urdf.xml gui:=True


