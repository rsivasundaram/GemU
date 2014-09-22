#include "sim/init.hh"

extern "C" {
    void init_param_myDevice();
}

EmbeddedSwig embed_swig_param_myDevice(init_param_myDevice);
