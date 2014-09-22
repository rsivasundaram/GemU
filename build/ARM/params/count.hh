#ifndef __PARAMS__count__
#define __PARAMS__count__

class count;

#include <cstddef>
#include <string>
#include <cstddef>
#include "params/Platform.hh"

#include "params/myDevice.hh"

struct countParams
    : public myDeviceParams
{
    count * create();
    std::string devicename;
    Platform * platform;
};

#endif // __PARAMS__count__
