#pragma once

// robot arm lib
//extern "C"
//{
    #include "RobotArmBaseLib.h"
//}


class ArexxArmHardware
{
public:
    ArexxArmHardware(long baud = 38400)
		: baud_(baud) { }

    ArexxArmHardware(ArexxArmHardware& rhs)
		: baud_(rhs.baud_) { }

//    void setBaud(long baud)
//    { this->baud_= baud; }

    int getBaud()
    { return baud_; }

    void init()
    { startStopwatch1(); }

    int read()
    {
		char cc;
		receiveBytesToBuffer(1, &cc);
		return cc;
	}

    void write(uint8_t* data, int length)
    {
    	for(int i=0; i<length; i++)
        	writeChar(data[i]);
    }

    unsigned long time()
    { return getStopwatch1(); }

  protected:
    const long baud_;
};


