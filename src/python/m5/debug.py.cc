#include "sim/init.hh"

namespace {

const uint8_t data_m5_debug[] = {
    120,156,181,87,93,79,220,86,16,157,235,253,254,160,161,36,
    16,33,165,13,77,155,202,138,84,182,47,188,208,168,105,40,
    42,84,106,80,107,148,68,89,137,90,198,190,44,6,219,187,
    216,119,35,144,224,137,62,240,11,250,220,127,209,127,213,191,
    208,206,25,219,192,74,169,218,7,179,187,190,154,251,57,103,
    206,156,177,189,62,21,159,26,95,223,173,16,101,191,177,17,
    240,79,81,68,20,43,26,42,82,232,91,20,89,52,180,100,
    174,70,81,141,226,58,13,235,20,55,104,216,200,87,212,101,
    180,73,195,38,197,45,26,182,242,209,6,69,109,138,59,52,
    236,112,191,73,124,252,176,75,65,139,180,162,3,69,65,27,
    35,151,68,239,134,61,210,61,88,195,62,156,237,218,29,6,
    18,254,205,31,91,177,101,208,221,12,125,243,42,60,13,147,
    29,219,194,88,151,155,221,48,158,68,250,135,200,27,153,62,
    119,191,31,199,147,241,52,9,48,112,179,40,243,15,117,176,
    145,106,239,216,204,163,171,141,163,227,177,209,91,155,27,63,
    143,83,115,227,97,146,134,137,249,41,204,140,15,82,234,124,
    53,112,40,136,89,86,194,202,214,246,105,76,134,151,43,186,
    84,180,185,231,210,5,209,185,16,116,108,81,250,140,78,22,
    40,145,105,30,92,190,80,52,226,105,69,71,22,98,219,220,
    235,211,133,69,231,22,253,106,209,201,75,186,168,209,121,141,
    178,5,80,122,158,175,171,97,221,129,69,75,91,219,124,18,
    255,222,110,51,179,236,245,247,219,94,175,102,189,254,65,39,
    87,255,215,235,159,133,215,244,234,195,94,77,93,54,215,120,
    219,66,177,237,168,129,105,108,254,11,9,229,52,94,42,165,
    216,37,255,24,30,39,193,110,49,77,59,89,143,219,13,47,
    211,43,160,63,91,55,80,213,203,40,202,144,131,21,254,60,
    205,214,249,202,238,221,74,84,185,180,137,116,39,129,78,76,
    216,102,211,70,95,6,51,78,144,14,12,242,112,128,165,6,
    89,57,14,131,220,8,116,230,59,152,147,94,226,197,218,134,
    225,244,203,33,236,49,200,175,111,112,174,127,24,70,65,170,
    19,27,249,149,38,91,231,102,112,56,142,245,96,28,133,239,
    117,122,122,118,54,152,164,227,81,234,197,131,145,142,215,190,
    202,140,183,31,233,65,150,250,131,201,153,57,28,39,131,120,
    109,16,232,253,233,104,149,251,112,114,168,163,201,83,156,245,
    37,78,84,13,117,95,245,85,83,117,213,99,110,31,49,87,
    179,35,139,234,107,37,104,152,27,9,223,47,203,16,64,55,
    160,182,125,54,52,161,250,152,94,20,141,5,241,193,168,33,
    233,48,234,200,31,140,6,106,15,70,19,217,129,209,162,162,
    212,218,40,53,24,157,162,212,80,124,29,49,122,228,236,218,
    72,140,175,10,199,86,41,245,37,18,169,179,54,92,78,138,
    8,205,21,32,187,82,82,59,121,89,74,117,33,10,151,73,
    203,194,113,34,57,114,3,46,210,188,160,234,82,106,209,65,
    117,92,139,55,55,76,66,227,186,223,224,188,186,144,214,81,
    18,67,173,40,89,137,33,4,2,162,163,178,8,46,36,30,
    22,58,203,29,37,243,68,226,193,80,189,208,183,58,125,86,
    238,104,230,101,179,92,232,63,95,133,66,104,97,226,249,201,
    11,122,123,158,243,83,203,105,129,251,29,187,93,50,194,55,
    17,157,38,94,36,140,8,122,179,192,214,72,155,50,227,111,
    114,202,28,12,59,247,177,15,75,253,72,123,169,233,205,46,
    21,45,219,8,206,121,128,133,168,30,127,154,178,136,77,201,
    188,51,87,50,92,9,205,40,102,119,58,9,60,163,95,224,
    184,143,132,229,123,252,173,91,115,106,73,61,84,190,85,232,
    229,90,51,15,73,116,114,36,119,115,149,179,195,84,30,55,
    175,217,193,82,231,81,25,111,222,125,80,234,36,52,58,174,
    46,128,190,232,196,103,73,122,97,146,185,238,143,55,90,233,
    126,0,251,226,44,118,1,206,17,44,255,23,114,231,147,74,
    105,239,9,106,206,60,200,112,221,87,51,160,203,34,85,255,
    66,248,45,169,151,176,107,51,176,243,251,166,62,203,100,86,
    240,87,6,221,97,64,244,75,197,120,113,243,127,239,69,83,
    125,39,136,63,227,237,175,43,70,220,40,132,124,39,128,159,
    240,246,119,21,3,110,231,128,211,187,146,197,231,188,125,175,
    98,204,221,2,243,221,73,227,11,222,238,85,140,186,83,160,
    190,51,121,224,245,67,223,6,109,247,203,12,187,46,222,139,
    92,87,168,115,221,120,28,76,35,238,58,139,215,16,63,69,
    243,24,13,10,89,106,67,244,38,9,20,62,228,252,27,180,
    149,64,254,152,183,227,136,12,76,52,85,167,222,233,242,117,
    235,107,207,149,33,188,206,116,138,183,127,7,126,29,188,159,
    24,60,148,202,231,236,170,156,233,8,173,114,119,22,246,193,
    132,60,203,226,181,213,169,9,35,121,79,116,240,7,64,92,
    59,157,202,67,18,172,207,115,130,191,197,195,65,30,157,243,
    86,223,90,226,199,230,188,213,89,92,26,252,3,112,151,164,
    215,
};

EmbeddedPython embedded_m5_debug(
    "m5/debug.py",
    "/home/oliverxyy/program/gem5-stable/src/python/m5/debug.py",
    "m5.debug",
    data_m5_debug,
    1137,
    3443);

} // anonymous namespace