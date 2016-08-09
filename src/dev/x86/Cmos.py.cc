#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Cmos[] = {
    120,156,173,82,77,107,220,48,16,29,217,235,205,238,18,218,
    252,129,130,41,148,184,129,174,147,64,210,16,74,200,71,47,
    189,44,193,222,67,235,139,81,109,101,215,197,178,140,36,111,
    237,115,250,191,219,25,121,67,251,3,42,201,195,155,167,241,
    204,211,72,5,236,135,143,223,109,8,96,86,8,74,92,12,
    106,128,245,30,177,17,121,80,123,32,125,200,124,96,228,251,
    80,79,64,6,144,5,232,79,64,248,240,196,160,12,224,23,
    192,51,192,183,108,10,229,20,210,232,0,83,86,191,113,68,
    12,145,37,115,50,194,87,104,238,185,169,138,199,74,125,22,
    187,170,16,35,255,26,205,215,171,203,47,141,77,85,167,11,
    241,88,53,118,130,220,131,84,166,248,87,242,61,73,190,67,
    32,0,50,70,194,51,143,20,163,66,84,155,161,166,0,126,
    76,73,41,234,123,70,230,192,49,51,16,51,146,136,90,137,
    156,67,146,70,83,204,146,80,78,115,184,175,158,222,93,95,
    83,69,67,114,74,177,139,251,171,203,184,64,98,185,221,154,
    5,114,167,103,49,174,243,211,179,115,115,130,110,58,24,43,
    100,104,43,41,66,171,194,206,136,48,58,94,169,159,199,225,
    147,210,33,47,108,199,107,183,251,222,188,195,112,60,20,133,
    153,106,211,32,159,172,31,66,94,115,45,67,60,171,208,186,
    107,173,193,237,136,234,216,25,154,60,111,184,20,121,110,23,
    206,145,170,236,106,114,169,47,118,104,133,157,35,40,250,62,
    47,106,110,140,139,34,111,43,120,41,180,13,168,32,215,92,
    186,248,53,138,24,127,68,144,120,4,220,37,53,54,111,171,
    38,162,238,254,53,230,35,154,120,171,164,136,85,93,237,132,
    238,135,33,110,181,218,96,182,120,35,228,197,7,99,249,247,
    90,196,70,23,241,75,155,168,111,203,118,112,13,125,75,73,
    232,170,167,108,156,115,156,135,108,229,30,134,147,45,47,150,
    45,137,51,238,164,228,105,213,15,150,174,100,124,22,9,123,
    9,29,159,5,246,206,201,118,249,255,163,94,87,255,211,216,
    219,155,55,148,140,136,5,91,176,35,118,228,253,1,57,65,
    178,117,
};

EmbeddedPython embedded_m5_objects_Cmos(
    "m5/objects/Cmos.py",
    "/home/oliverxyy/program/gem5-stable/src/dev/x86/Cmos.py",
    "m5.objects.Cmos",
    data_m5_objects_Cmos,
    482,
    814);

} // anonymous namespace