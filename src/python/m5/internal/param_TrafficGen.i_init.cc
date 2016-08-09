#include "sim/init.hh"

extern "C" {
    void init_param_TrafficGen();
}

EmbeddedSwig embed_swig_param_TrafficGen(init_param_TrafficGen);
