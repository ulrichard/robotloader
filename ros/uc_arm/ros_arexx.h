#pragma once

#include "ros/node_handle.h"
#include "ArexxArmHardware.h"

namespace ros
{
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega168__)
  /* downsize our buffers */
  typedef NodeHandle_<ArexxArmHardware, 6, 6, 150, 150> NodeHandle;

#elif defined(__AVR_ATmega328P__)

  typedef NodeHandle_<ArexxArmHardware, 25, 25, 280, 280> NodeHandle;

#else

  typedef NodeHandle_<ArexxArmHardware> NodeHandle;

#endif   
}

