#ifndef __PARAMS__myDevice__
#define __PARAMS__myDevice__

class myDevice;

#include <cstddef>
#include "params/Terminal.hh"
#include <cstddef>
#include <string>
#include <cstddef>
#include "params/Platform.hh"

#include "params/BasicPioDevice.hh"

struct myDeviceParams
    : public BasicPioDeviceParams
{
   
    Terminal * terminal;
    std::string devicename;
    Platform * platform;
};

#endif // __PARAMS__myDevice__
