#include "sim/init.hh"

extern "C" {
    void init_param_new_Terminal();
}

EmbeddedSwig embed_swig_param_new_Terminal(init_param_new_Terminal);
