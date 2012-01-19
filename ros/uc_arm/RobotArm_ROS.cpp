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
int main(void)
{
	initRobotBase(); // Always call this first!
	Power_Servos();
	mSleep(200);

	ros::NodeHandle nh;
	nh.initNode();

    ros::Subscriber<std_msgs::Int16> sub1("ArexxArmServo1", servo_cb<1>);
    nh.subscribe(sub1);
    ros::Subscriber<std_msgs::Int16> sub2("ArexxArmServo2", servo_cb<2>);
    nh.subscribe(sub2);
    ros::Subscriber<std_msgs::Int16> sub3("ArexxArmServo3", servo_cb<3>);
    nh.subscribe(sub3);
    ros::Subscriber<std_msgs::Int16> sub4("ArexxArmServo4", servo_cb<4>);
    nh.subscribe(sub4);
    ros::Subscriber<std_msgs::Int16> sub5("ArexxArmServo5", servo_cb<5>);
    nh.subscribe(sub5);
    ros::Subscriber<std_msgs::Int16> sub6("ArexxArmServo6", servo_cb<6>);
    nh.subscribe(sub6);

	while(true) // main loop
	{
		nh.spinOnce();
		mSleep(1);
	}

	return 0;
}
/*****************************************************************************/
/// http://stackoverflow.com/questions/920500/what-is-the-purpose-of-cxa-pure-virtual
extern "C" void __cxa_pure_virtual() { while (1); }
/*****************************************************************************/

