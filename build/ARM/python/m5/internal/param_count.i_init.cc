#include "sim/init.hh"

extern "C" {
    void init_param_count();
}

EmbeddedSwig embed_swig_param_count(init_param_count);
