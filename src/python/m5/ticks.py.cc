#include "sim/init.hh"

namespace {

const uint8_t data_m5_ticks[] = {
    120,156,173,148,79,111,220,68,20,192,223,216,155,221,120,179,
    109,67,75,150,168,20,97,33,66,86,162,141,79,17,106,85,
    133,210,150,6,14,84,149,147,83,132,180,242,218,179,27,39,
    246,204,50,51,91,186,82,246,66,248,0,28,122,226,200,141,
    175,195,55,225,142,4,239,189,89,111,212,138,99,44,123,252,
    252,102,252,254,254,102,114,88,94,45,124,158,196,0,118,136,
    66,129,183,128,10,224,196,203,1,84,2,234,0,78,2,16,
    69,8,89,8,178,5,217,26,20,45,248,21,23,181,161,88,
    99,161,3,69,27,78,214,161,232,240,103,4,5,202,56,118,
    161,216,128,73,11,78,186,100,247,104,208,67,31,229,191,120,
    189,28,8,20,29,57,255,57,51,106,130,239,63,222,222,173,
    159,230,20,18,77,133,248,60,163,176,158,3,199,132,209,96,
    88,11,252,71,128,125,2,46,128,76,192,5,192,89,8,103,
    45,112,107,224,218,112,41,232,22,24,245,234,187,127,248,157,
    226,84,142,6,100,241,37,59,183,95,162,120,88,233,81,86,
    197,99,35,127,154,73,149,207,99,43,93,156,185,120,167,136,
    93,153,159,219,120,42,13,234,114,173,138,65,135,98,93,167,
    216,149,147,70,101,149,139,72,51,181,195,113,249,70,22,156,
    198,177,153,73,22,114,109,164,251,0,5,52,248,172,210,249,
    249,139,198,133,11,189,9,126,227,223,92,131,148,134,1,52,
    131,125,132,67,114,170,107,153,232,170,124,45,205,155,249,60,
    153,26,61,49,89,157,76,100,189,255,192,186,108,84,201,196,
    154,60,153,206,221,169,86,73,189,159,112,200,123,248,125,27,
    127,199,160,124,118,43,207,159,147,229,46,215,182,23,180,69,
    91,244,69,78,126,131,37,0,92,233,223,151,149,14,168,210,
    181,128,5,214,18,235,108,190,0,12,24,187,143,217,245,127,
    9,0,235,137,229,197,210,99,133,93,7,198,1,92,226,162,
    231,164,201,90,160,210,102,118,157,245,63,54,250,71,141,62,
    98,253,37,184,46,92,8,56,219,32,165,239,28,45,187,3,
    174,71,120,185,27,75,61,185,228,254,173,173,250,231,217,233,
    112,173,21,150,200,217,7,255,215,208,172,50,50,43,230,49,
    247,136,91,59,246,173,77,236,158,253,140,200,51,90,77,98,
    55,159,202,120,119,199,238,198,99,109,252,130,87,210,28,249,
    206,223,108,60,213,251,123,51,87,86,41,53,62,165,178,57,
    154,250,198,57,83,142,102,78,126,107,140,54,41,115,66,101,
    46,109,169,176,79,42,151,105,187,225,188,66,103,142,146,24,
    87,58,243,12,88,103,88,99,244,76,21,108,48,83,243,99,
    125,5,12,121,59,198,248,216,60,91,161,104,7,43,255,239,
    70,203,193,93,47,74,200,240,123,40,221,39,237,54,163,180,
    217,106,139,80,220,8,182,68,36,110,241,179,29,132,98,242,
    207,159,127,255,245,195,232,213,215,121,179,143,87,132,189,165,
    144,129,41,16,68,129,221,161,141,140,204,97,53,174,186,173,
    184,96,246,43,2,6,97,244,26,156,70,4,207,145,156,67,
    18,142,60,127,15,23,220,30,100,109,5,209,130,143,5,36,
    107,27,95,31,47,2,184,8,136,185,243,22,152,223,168,57,
    200,150,159,191,12,65,40,150,150,108,217,79,137,168,76,237,
    186,120,201,149,199,194,105,207,8,23,230,0,215,124,175,98,
    109,10,60,30,112,166,208,203,197,182,212,202,222,143,221,169,
    140,39,239,147,88,207,172,139,71,210,147,88,114,111,146,166,
    237,37,34,40,169,189,241,1,218,171,164,33,106,186,49,94,
    136,43,175,64,120,209,209,78,49,216,32,246,232,8,77,169,
    247,41,245,39,189,211,240,152,242,44,17,200,200,165,155,84,
    107,154,93,133,49,92,153,79,153,17,62,133,40,243,215,89,
    133,103,87,228,15,167,161,255,162,73,12,235,250,104,218,224,
    80,116,237,81,181,79,201,228,61,198,232,150,232,51,72,81,
    208,19,173,176,27,220,22,55,69,15,113,234,138,116,139,50,
    161,131,55,189,75,67,159,130,33,67,126,247,204,109,74,155,
    141,179,241,123,143,178,121,145,85,86,250,146,240,159,91,205,
    159,108,131,55,243,112,152,85,213,112,120,149,220,181,100,72,
    222,30,215,186,152,85,242,224,19,178,71,13,234,137,77,58,
    112,195,104,61,250,176,29,68,31,69,226,63,233,229,172,82,
};

EmbeddedPython embedded_m5_ticks(
    "m5/ticks.py",
    "/home/oliverxyy/program/gem5-stable/src/python/m5/ticks.py",
    "m5.ticks",
    data_m5_ticks,
    976,
    1914);

} // anonymous namespace
