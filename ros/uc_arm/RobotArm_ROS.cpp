// ArexxRobotArm as a ROS node
// Looking at the rosserial_arduino and adapting it to the arexx robot arm
// http://www.ros.org/wiki/rosserial_arduino/Tutorials
// Richard Ulrich <richi@paraeasy.ch>
// GPL v3

// robot arm lib
#include "RobotArmBaseLib.h"
// ros
#include "ros_arexx.h"
#include <std_msgs/Int16.h>
// std lib
#include <ctype.h>
/*****************************************************************************/
template<uint8_t servonum>
void servo_cb(const std_msgs::Int16& cmd_msg)
{
    s_Move(servonum, cmd_msg.data, 3);  // servoNr, pos, speed
}
/*****************************************************************************/
template<uint8_t servonum>
void subscribe_servo(ros::NodeHandle& nh)
{
    static char buf[] = "ArexxArmServoX";
    buf[13] = '0' + servonum;
    ros::Subscriber<std_msgs::Int16> sub(buf, servo_cb<servonum>);
    nh.subscribe(sub);
}
/*****************************************************************************/
int main(void)
{
	initRobotBase(); // Always call this first!
	Power_Servos();
	mSleep(200);

	ros::NodeHandle nh;
	nh.initNode();

    subscribe_servo<1>(nh);
    subscribe_servo<2>(nh);
    subscribe_servo<3>(nh);
    subscribe_servo<4>(nh);
    subscribe_servo<5>(nh);
    subscribe_servo<6>(nh);

    PowerLEDgreen();

	while(true) // main loop
	{
		nh.spinOnce();
		mSleep(1);
	}

	PowerLEDred();

	return 0;
}
/*****************************************************************************/
/// http://stackoverflow.com/questions/920500/what-is-the-purpose-of-cxa-pure-virtual
extern "C" void __cxa_pure_virtual() { while (1); }
/*****************************************************************************/

