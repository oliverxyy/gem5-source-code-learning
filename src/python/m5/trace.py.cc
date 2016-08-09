#include "sim/init.hh"

namespace {

const uint8_t data_m5_trace[] = {
    120,156,173,81,65,75,195,48,20,126,105,187,234,96,120,17,
    188,9,34,8,189,184,156,118,17,17,65,241,184,67,230,169,
    23,169,109,232,58,146,166,164,175,195,222,253,223,250,94,24,
    219,188,202,66,250,248,242,248,222,247,125,73,75,216,173,136,
    190,231,27,128,254,149,64,69,91,128,1,200,247,88,64,46,
    2,142,192,68,96,99,200,99,176,9,228,9,136,42,134,111,
    162,78,160,74,2,72,121,96,149,77,72,168,249,161,181,204,
    88,28,83,42,110,192,110,192,0,155,186,117,94,151,199,254,
    47,236,127,201,92,218,2,54,17,108,98,248,72,216,118,149,
    9,234,47,131,40,114,121,43,76,175,241,156,133,90,212,190,
    45,76,104,163,47,74,141,9,161,114,91,120,60,35,160,219,
    226,211,232,42,99,151,67,233,31,168,200,181,179,90,58,211,
    108,181,255,26,71,217,121,87,251,194,202,90,219,197,125,143,
    60,39,123,95,202,110,196,181,107,165,93,200,160,63,167,51,
    43,87,77,207,148,91,150,227,252,32,254,117,29,78,251,238,
    7,173,98,2,138,79,138,251,42,253,19,248,36,169,211,253,
    123,220,29,66,103,124,151,96,30,162,12,216,24,188,56,122,
    216,121,24,87,204,85,28,91,5,254,244,244,233,248,111,62,
    90,87,13,70,63,93,179,30,55,102,98,22,93,69,211,248,
    23,232,47,140,24,
};

EmbeddedPython embedded_m5_trace(
    "m5/trace.py",
    "/home/oliverxyy/program/gem5-stable/src/python/m5/trace.py",
    "m5.trace",
    data_m5_trace,
    309,
    682);

} // anonymous namespace
