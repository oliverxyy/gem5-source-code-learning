#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Vnc[] = {
    120,156,173,82,77,111,211,64,16,157,117,18,39,148,34,65,
    197,199,17,159,42,131,68,124,170,202,1,161,170,7,164,114,
    128,202,169,144,200,197,114,214,147,120,171,245,135,118,55,85,
    114,46,255,27,102,166,105,64,32,113,170,19,63,237,206,142,
    223,188,125,51,26,118,207,128,222,179,4,192,127,166,69,69,
    127,5,22,160,81,48,87,160,120,31,129,141,224,170,26,0,
    42,88,42,168,134,240,3,224,22,224,251,124,0,213,8,112,
    32,209,120,31,29,66,53,134,89,58,33,58,243,147,158,84,
    209,42,60,34,152,153,230,235,226,26,117,184,11,49,188,13,
    156,247,173,213,23,109,191,14,250,79,81,231,44,234,152,22,
    8,172,133,164,204,35,86,71,101,113,8,215,84,57,102,113,
    183,17,204,199,144,207,82,254,38,143,8,252,115,130,69,233,
    49,187,105,53,191,134,185,167,117,237,95,211,129,46,251,176,
    118,152,232,186,108,87,88,37,75,87,54,232,147,208,37,75,
    99,209,139,110,17,85,20,45,157,20,69,56,144,77,211,85,
    107,203,219,33,39,108,123,148,184,222,108,138,26,203,10,93,
    24,209,246,178,36,54,201,56,239,58,43,161,79,165,245,24,
    158,208,74,42,21,187,250,41,95,243,55,248,83,130,172,238,
    26,204,58,107,110,208,109,182,219,172,119,221,138,190,201,86,
    216,156,188,243,161,92,88,204,188,211,217,254,110,228,219,180,
    223,202,165,223,48,73,76,16,43,254,137,223,116,60,67,71,
    100,255,250,250,254,63,190,146,169,212,109,246,53,150,200,152,
    123,78,157,230,200,132,157,230,50,249,99,46,248,226,47,167,
    189,148,35,171,205,225,43,58,230,28,107,124,192,54,233,59,
    23,140,220,244,25,1,165,38,218,26,108,67,210,174,155,5,
    186,148,245,230,210,66,118,47,103,231,114,169,51,102,187,25,
    174,116,127,73,36,226,46,179,5,206,190,104,67,224,180,29,
    203,3,154,202,218,167,76,50,217,155,122,164,190,164,44,44,
    28,18,52,39,211,253,60,231,112,63,226,20,237,121,6,188,
    52,69,72,30,80,148,204,229,135,187,73,252,120,124,223,241,
    167,234,32,122,57,250,5,138,64,210,237,
};

EmbeddedPython embedded_m5_objects_Vnc(
    "m5/objects/Vnc.py",
    "/home/oliverxyy/program/gem5-stable/src/base/vnc/Vnc.py",
    "m5.objects.Vnc",
    data_m5_objects_Vnc,
    491,
    977);

} // anonymous namespace