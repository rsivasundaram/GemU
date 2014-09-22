#ifndef __PARAMS__new_Terminal__
#define __PARAMS__new_Terminal__

class new_Terminal;

#include <cstddef>
#include "base/types.hh"
#include <cstddef>
#include "params/IntrControl.hh"
#include <cstddef>
#include "base/types.hh"
#include <cstddef>

#include "params/SimObject.hh"

struct new_TerminalParams
    : public SimObjectParams
{
    new_Terminal * create();
    uint16_t port;
    IntrControl * intr_control;
    int number;
    bool output;
};

#endif // __PARAMS__new_Terminal__
