#! /bin/bash

# run the core system
roscore &

# connect to the arm with rosserial
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=38400 &

# send commands
rostopic pub ArexxArmServo6 std_msgs/Int16  0
rostopic pub ArexxArmServo5 std_msgs/Int16  0
rostopic pub ArexxArmServo4 std_msgs/Int16  0



