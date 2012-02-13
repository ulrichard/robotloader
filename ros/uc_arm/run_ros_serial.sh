#! /bin/bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/richi/sourcecode/robotloader/ros

# run the core system
roscore &

# connect to the arm with rosserial
#rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=38400 &
rosrun uc_arm robot_arm_serial_node.py _port:=/dev/ttyUSB0 _baud:=38400 &

# see what's connected
rosnode list
rosnode info /arexx_mini_arm
rostopic list -v
#rostopic echo /ArexxArmServo1
rosservice list
rosparam list
rosparam get /arexx_mini_arm/port

# logger windows
#rxloggerlevel
#rxconsole

# send commands
rostopic pub -1 /ArexxArmServo6 std_msgs/Int16  0
rostopic pub -1 /ArexxArmServo1 std_msgs/Int16  0
rostopic pub -1 /ArexxArmLed std_msgs/UInt8  1

# see the current consumption
rxplot /ArexxArmServoCurrent1




