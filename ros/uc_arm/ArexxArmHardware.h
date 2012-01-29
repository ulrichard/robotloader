#pragma once

// robot arm lib
#include <RobotArmBaseLib.h>


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
    {
        startStopwatch1();
        clearReceptionBuffer();
    }

    int read()
    {
        if(0 == getBufferLength())
            return -1;

        const char    cc = readChar();
        const uint8_t uc = (uint8_t)(cc);
        return uc;
	}

    void write(uint8_t* data, int length)
    {
    	for(int i=0; i<length; i++)
        	writeChar(data[i]);
    }

    void writeStr(const char* data, const bool newline)
    {
        write(reinterpret_cast<uint8_t*>(const_cast<char*>(data)), strlen(data));
        if(newline)
            writeChar('\n');
    }

    void writeInt(const int16_t val, const bool newline)
    {
        writeInteger(val, BIN);
        if(newline)
            writeChar('\n');
    }


    unsigned long time()
    { return getStopwatch1(); }

  protected:
    const long baud_;
};


