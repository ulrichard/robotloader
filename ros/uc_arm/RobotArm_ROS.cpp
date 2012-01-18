// ArexxRobotArm as a ROS node
// Looking at the rosserial_arduino and adapting it to the arexx robot arm
// http://www.ros.org/wiki/rosserial_arduino/Tutorials
// Richard Ulrich <richi@paraeasy.ch>
// GPL v3

// robot arm lib
//#include "RobotArmBaseLib.h" // missing include guard
// ros
#include "ros_arexx.h"
// std lib
#include <ctype.h>
/*****************************************************************************/
int main(void)
{
	initRobotBase(); // Always call this first!
	Power_Servos();
	mSleep(200);

	ros::NodeHandle nh;
	nh.initNode();

	uint8_t servospeed = 3;	// The speed (Fast 0  ......  10 slow)
	// Main loop:
	while(true)
	{
		char cc;
		receiveBytesToBuffer(1, &cc);

//		recbuf[bufpos++] = cc;


		nh.spinOnce();

//		s_Move(servonum, servopos, servospeed);

//		bufpos = 0;
		continue;
	}
	// End of main loop!
	// ---------------------------------------

	return 0;
}
