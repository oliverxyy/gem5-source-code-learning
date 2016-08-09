#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_X86NativeTrace[] = {
    120,156,181,144,77,75,195,64,16,134,103,147,180,106,21,233,
    69,4,65,208,91,16,236,158,90,74,17,145,234,185,74,90,
    65,115,9,219,205,218,84,178,77,217,141,146,156,245,127,235,
    76,98,170,253,1,110,178,47,243,197,236,51,35,225,231,184,
    120,111,206,0,236,29,26,49,254,12,82,0,205,32,100,192,
    200,119,32,117,96,70,150,11,169,11,218,131,208,195,140,7,
    202,131,23,6,113,11,62,1,62,0,158,195,22,196,109,152,
    250,59,216,104,249,133,199,103,104,229,123,40,211,165,190,159,
    191,42,153,215,33,146,139,218,220,71,153,136,124,249,174,102,
    70,72,149,31,162,255,52,28,252,9,201,134,148,234,199,68,
    122,130,134,2,2,68,170,208,33,228,208,37,80,4,11,166,
    62,77,20,144,216,35,148,170,199,104,180,221,211,30,99,70,
    24,153,240,98,56,224,171,42,145,83,162,151,36,126,139,184,
    118,81,162,104,37,180,138,162,188,83,57,58,139,223,82,114,
    61,42,40,215,170,26,77,22,69,36,83,97,109,85,69,94,
    162,68,172,140,79,200,191,98,199,40,60,201,180,226,89,138,
    175,153,162,44,249,218,100,11,35,52,95,40,221,191,180,185,
    152,167,138,91,35,249,134,108,155,186,183,46,171,185,206,169,
    95,27,165,205,232,155,248,100,231,7,40,186,223,219,172,58,
    128,102,251,24,93,11,124,167,70,188,125,120,172,186,25,27,
    56,205,166,254,7,182,90,226,85,189,182,235,211,6,186,203,
    58,172,235,124,3,1,165,143,115,
};

EmbeddedPython embedded_m5_objects_X86NativeTrace(
    "m5/objects/X86NativeTrace.py",
    "/home/oliverxyy/program/gem5-stable/src/arch/x86/X86NativeTrace.py",
    "m5.objects.X86NativeTrace",
    data_m5_objects_X86NativeTrace,
    345,
    640);

} // anonymous namespace