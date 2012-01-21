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
#include <std_msgs/UInt8.h>
// std lib
#include <ctype.h>
/*****************************************************************************/
// callback functions
template<uint8_t servonum>
void servo_cb(const std_msgs::Int16& cmd_msg)
{
    s_Move(servonum, cmd_msg.data, 3);  // servoNr, pos, speed
}
/*****************************************************************************/
void led_cb(const std_msgs::UInt8& cmd_msg)
{
    switch(cmd_msg.data)
    {
        case 1:
            PowerLEDgreen();
            break;
        case 2:
            PowerLEDorange();
            break;
        case 3:
            PowerLEDred();
            break;
        default:
            PowerLEDoff();
    }
}
/*****************************************************************************/
int main(void)
{
	initRobotBase(); // Always call this first!
	Power_Servos();
	mSleep(200);


	ros::NodeHandle nh;
	nh.initNode();

    // setting up the subscribers
    ros::Subscriber<std_msgs::Int16> subscrServo1("ArexxArmServo1", servo_cb<1>);
    nh.subscribe(subscrServo1);
    ros::Subscriber<std_msgs::Int16> subscrServo2("ArexxArmServo1", servo_cb<2>);
    nh.subscribe(subscrServo2);
    ros::Subscriber<std_msgs::Int16> subscrServo3("ArexxArmServo1", servo_cb<3>);
    nh.subscribe(subscrServo3);
    ros::Subscriber<std_msgs::Int16> subscrServo4("ArexxArmServo1", servo_cb<4>);
    nh.subscribe(subscrServo4);
    ros::Subscriber<std_msgs::Int16> subscrServo5("ArexxArmServo1", servo_cb<5>);
    nh.subscribe(subscrServo5);
    ros::Subscriber<std_msgs::Int16> subscrServo6("ArexxArmServo6", servo_cb<6>);
    nh.subscribe(subscrServo6);
    ros::Subscriber<std_msgs::UInt8> subscrLed("ArexxArmLed", led_cb);
    nh.subscribe(subscrLed);

    // setting up the publishers
    std_msgs::Int16 servo1curr;
    ros::Publisher pubServo1("ArexxArmServoCurrent1", &servo1curr);
    nh.advertise(pubServo1);


    PowerLEDgreen();

	while(true) // main loop
	{
		setBeepsound();
		mSleep(100);
		clearBeepsound();

        // read the sensors
        servo1curr.data = readADC(1);
        pubServo1.publish(&servo1curr);

        // ros communication
		nh.spinOnce();
	}

	PowerLEDred();

	return 0;
}
/*****************************************************************************/
/// http://stackoverflow.com/questions/920500/what-is-the-purpose-of-cxa-pure-virtual
extern "C" void __cxa_pure_virtual() { while (1); }
/*****************************************************************************/

