#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_BadDevice[] = {
    120,156,173,144,75,75,3,49,16,128,39,219,135,90,80,60,
    11,66,192,203,34,216,189,88,240,32,34,226,185,148,214,139,
    189,44,233,102,218,93,217,52,75,18,75,247,172,255,91,103,
    178,86,255,128,217,236,100,102,152,199,55,83,192,207,233,209,
    255,40,1,252,45,41,154,174,128,26,224,133,181,4,106,1,
    38,129,101,2,66,247,0,19,88,11,208,125,248,4,248,0,
    120,93,246,64,15,96,145,14,41,177,250,162,147,10,210,2,
    139,235,78,61,35,241,164,124,85,204,42,251,140,187,170,192,
    112,18,93,186,179,138,3,68,194,94,134,184,34,5,1,150,
    130,81,168,47,209,80,27,236,195,219,128,121,62,200,28,194,
    124,145,50,245,156,179,252,41,99,227,46,91,41,77,207,184,
    44,253,5,121,166,202,160,180,107,169,99,31,25,172,68,231,
    172,147,118,155,30,49,217,49,137,60,223,82,84,158,135,81,
    52,140,213,239,53,155,125,14,104,27,140,254,98,191,207,75,
    84,26,93,24,144,57,83,78,153,192,19,47,130,171,182,155,
    24,211,53,225,98,41,15,243,39,252,29,137,172,180,6,51,
    91,87,59,116,251,182,205,26,103,55,84,36,219,160,153,220,
    248,160,86,53,102,222,21,25,15,241,187,153,113,211,198,241,
    36,87,225,118,67,193,223,52,141,112,188,68,51,25,55,12,
    227,35,77,151,53,23,135,181,252,39,72,92,214,125,183,158,
    135,75,174,198,16,35,113,158,124,3,98,117,128,110,
};

EmbeddedPython embedded_m5_objects_BadDevice(
    "m5/objects/BadDevice.py",
    "/home/oliverxyy/program/gem5-stable/src/dev/BadDevice.py",
    "m5.objects.BadDevice",
    data_m5_objects_BadDevice,
    334,
    581);

} // anonymous namespace
