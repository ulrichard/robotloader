#! /bin/bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/richi/sourcecode/robotloader/ros

# run the core system
roscore &

# connect to the arm with rosserial
#rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=38400 &
rosrun uc_arm robot_arm_serial_node.py _port:=/dev/ttyUSB0 _baud:=38400 &

# send commands
rostopic pub ArexxArmServo6 std_msgs/Int16  0
rostopic pub ArexxArmServo5 std_msgs/Int16  0
rostopic pub ArexxArmLed std_msgs/UInt8  1



