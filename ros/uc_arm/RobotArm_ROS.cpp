// ArexxRobotArm as a ROS node
// Looking at the rosserial_arduino and adapting it to the arexx robot arm
// http://www.ros.org/wiki/rosserial_arduino/Tutorials
// Richard Ulrich <richi@paraeasy.ch>
// GPL v3

//#define ROBOT_ARM_UART_DEBUGGING

#include "ArexxArmHardware.h"
// robot arm lib
#include <RobotArmBaseLib.h>
// ros
#include <ros/node_handle.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
// std lib
#include <ctype.h>

/*****************************************************************************/
namespace ros
{
/*
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__)
  // downsize our buffers
  typedef NodeHandle_<ArexxArmHardware, 6, 6, 150, 150> NodeHandle;

#elif defined(__AVR_ATmega328P__)
  typedef NodeHandle_<ArexxArmHardware, 25, 25, 280, 280> NodeHandle;
#else

//  typedef NodeHandle_<ArexxArmHardware, 25, 25, 512, 512> NodeHandle;
  typedef NodeHandle_<ArexxArmHardware> NodeHandle;

#endif
*/

    typedef NodeHandle_<ArexxArmHardware, 10, 10, 400, 400> NodeHandle;
}
/*****************************************************************************/
// callback functions
template<uint8_t servonum>
void servo_cb(const std_msgs::Int16& cmd_msg)
{
    Move(servonum, cmd_msg.data);  // servoNr, pos
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
    // busy wait a little
    for(size_t i=0; i<1024; ++i)
        int j = i * i * i;
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

/*
    std_msgs::String str_msg;
    ros::Publisher chatter("chatter", &str_msg);
    char hello[13] = "hello world!";
*/

    PowerLEDoff();

    uint16_t lastADC = 0;

	while(true) // main loop
	{
#ifdef ROBOT_ARM_UART_DEBUGGING
//        writeString_P("main loop\n");
#endif

        if(getStopwatch1() - lastADC > 5000)
        {
            lastADC = getStopwatch1();
            // read the sensors
            servo1curr.data = readADC(1);
            pubServo1.publish(&servo1curr);

            // chatter
 //           str_msg.data = hello;
 //           chatter.publish(&str_msg);
        }

        // ros communication
		nh.spinOnce();
		mSleep(1);

		PowerLEDorange();
	}

	PowerLEDred();

	return 0;
}
/*****************************************************************************/
namespace ros
{
  void normalizeSecNSec(unsigned long& sec, unsigned long& nsec){
    unsigned long nsec_part= nsec % 1000000000UL;
    unsigned long sec_part = nsec / 1000000000UL;
    sec += sec_part;
    nsec = nsec_part;
  }

  Time& Time::fromNSec(long t)
  {
    sec = t / 1000000000;
    nsec = t % 1000000000;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  Time& Time::operator +=(const Duration &rhs)
  {
    sec += rhs.sec;
    nsec += rhs.nsec;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

  Time& Time::operator -=(const Duration &rhs){
    sec += -rhs.sec;
    nsec += -rhs.nsec;
    normalizeSecNSec(sec, nsec);
    return *this;
  }

}
/*****************************************************************************/
/// http://stackoverflow.com/questions/920500/what-is-the-purpose-of-cxa-pure-virtual
extern "C" void __cxa_pure_virtual() { while (1); }
/*****************************************************************************/

