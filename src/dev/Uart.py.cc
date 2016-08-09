#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Uart[] = {
    120,156,173,82,77,111,212,64,12,245,36,217,143,110,11,236,
    161,112,171,20,110,17,18,27,4,90,137,3,66,168,234,185,
    170,210,244,64,46,209,52,241,54,169,146,77,52,51,187,218,
    156,203,255,6,123,146,44,133,43,228,195,178,61,150,253,158,
    223,100,48,60,46,253,223,124,0,29,147,147,211,39,160,2,
    136,7,79,244,158,3,149,3,181,11,137,11,34,119,1,93,
    216,8,200,61,248,1,240,4,240,61,241,32,159,0,122,54,
    59,61,102,39,144,207,224,54,152,83,227,242,39,61,129,32,
    207,176,121,215,187,47,201,92,74,93,102,55,101,115,133,251,
    50,67,227,81,234,78,42,147,61,199,119,201,248,174,200,65,
    128,68,48,202,196,97,16,60,87,0,205,193,41,60,206,0,
    231,240,120,194,96,159,28,72,22,54,121,58,38,93,155,60,
    131,232,54,224,17,145,67,70,159,50,101,220,135,59,26,184,
    42,10,77,99,224,166,146,102,211,168,218,55,69,169,253,220,
    162,242,201,107,169,198,111,54,43,125,70,69,113,129,190,65,
    85,151,91,89,5,47,152,10,147,76,211,173,172,49,77,205,
    194,6,117,147,239,42,14,121,160,233,218,158,92,172,118,104,
    171,229,189,54,74,102,198,86,103,135,67,90,160,204,81,153,
    9,131,144,74,214,182,106,132,99,166,125,26,183,198,240,74,
    228,182,179,231,237,120,206,65,60,64,178,193,17,31,111,241,
    183,209,159,200,132,69,83,99,216,84,229,30,213,161,235,194,
    86,53,15,52,49,124,192,122,253,94,27,121,95,97,168,85,
    22,242,114,88,141,85,219,217,141,189,229,6,140,119,42,250,
    247,181,176,179,184,230,243,199,245,135,163,106,98,84,237,252,
    47,213,88,47,151,85,224,118,209,146,251,189,122,166,2,55,
    33,37,122,141,152,103,100,61,222,73,52,255,131,199,191,145,
    89,14,29,180,55,144,185,14,120,193,230,132,76,189,94,181,
    188,127,109,169,113,164,154,67,103,5,232,47,105,36,198,27,
    116,236,243,31,48,217,105,95,250,75,243,245,130,27,113,98,
    33,22,98,233,188,153,253,2,156,16,198,127,
};

EmbeddedPython embedded_m5_objects_Uart(
    "m5/objects/Uart.py",
    "/home/oliverxyy/program/gem5-stable/src/dev/Uart.py",
    "m5.objects.Uart",
    data_m5_objects_Uart,
    476,
    944);

} // anonymous namespace