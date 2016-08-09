#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_enum_X86IntelMPAddressType[] = {
    120,156,197,87,235,110,27,85,16,158,179,187,118,98,199,110,
    156,166,205,165,13,173,1,21,44,4,49,183,82,164,86,129,
    130,64,106,165,166,101,93,212,212,173,88,28,239,113,188,137,
    189,107,237,158,180,53,114,254,144,10,120,1,30,129,31,188,
    13,111,4,243,205,174,29,39,77,17,191,66,148,157,156,157,
    157,51,103,46,223,204,153,180,41,251,201,241,243,101,149,40,
    249,66,17,249,252,171,168,71,212,87,212,84,164,180,34,191,
    66,123,57,138,63,37,63,71,47,137,154,22,105,139,14,121,
    97,211,19,139,194,146,236,201,83,207,22,142,162,97,145,180,
    67,205,28,61,10,23,200,209,121,218,43,82,252,35,41,165,
    66,69,91,254,12,249,179,244,146,181,243,162,32,10,103,9,
    204,162,48,11,228,207,9,179,72,126,73,22,115,52,172,144,
    46,81,179,12,177,230,57,86,251,30,171,157,23,181,127,65,
    173,207,95,22,200,63,7,113,182,235,49,36,29,72,202,121,
    243,162,133,85,216,180,187,0,122,8,183,248,229,60,53,207,
    11,119,113,154,123,129,154,23,132,123,113,154,187,68,205,37,
    225,46,79,115,87,168,185,34,220,85,106,174,34,6,141,218,
    121,14,102,240,55,255,212,56,152,100,74,76,158,233,56,9,
    162,208,11,194,78,20,88,248,158,7,65,232,219,32,51,89,
    14,190,70,14,254,36,73,128,111,101,57,56,32,82,120,39,
    234,89,116,32,139,3,139,134,53,26,41,218,117,200,183,105,
    196,199,228,96,210,142,162,67,139,158,218,16,56,96,234,112,
    164,174,144,99,210,4,236,74,164,82,77,51,116,144,163,81,
    142,26,91,35,11,140,189,2,197,127,208,79,107,162,116,86,
    148,90,52,98,234,208,161,67,7,121,122,196,66,204,218,45,
    32,190,106,107,196,158,50,167,81,115,216,218,205,41,119,225,
    138,31,196,97,171,175,205,101,94,123,58,220,239,123,91,159,
    127,118,39,52,186,119,239,193,109,223,143,117,146,60,28,14,
    116,173,56,222,16,37,235,131,150,233,186,162,193,70,104,250,
    3,35,154,163,80,155,57,94,116,130,208,247,250,145,191,223,
    211,102,22,106,189,78,208,211,158,39,31,239,244,7,81,108,
    190,137,227,40,118,17,93,97,246,162,214,100,7,98,219,238,
    69,137,174,225,52,57,198,133,122,3,233,206,64,52,194,0,
    177,26,155,125,157,180,227,96,96,56,105,169,70,72,67,91,
    13,233,18,146,252,192,164,222,141,250,186,30,245,2,206,240,
    139,225,176,62,136,163,157,184,213,175,239,232,254,245,15,18,
    211,218,238,233,250,246,126,208,243,235,28,129,250,96,104,186,
    81,88,239,95,175,7,28,12,142,81,175,254,250,232,172,179,
    52,160,148,60,15,118,188,64,60,244,186,186,55,208,113,25,
    220,75,176,65,85,84,73,229,149,173,106,170,204,171,28,63,
    182,90,179,230,212,102,0,31,219,240,27,72,115,166,177,133,
    132,43,218,179,40,94,3,114,118,249,87,33,213,140,159,6,
    190,89,242,237,59,4,39,229,238,218,192,67,202,28,9,218,
    24,118,44,121,11,0,8,73,32,147,163,221,60,165,80,98,
    4,166,216,138,135,160,44,14,53,22,43,119,40,249,157,56,
    216,12,162,17,101,0,59,180,73,133,21,50,69,20,44,115,
    151,248,192,159,5,163,141,26,204,223,20,128,152,110,144,68,
    207,67,73,3,214,82,85,13,142,204,131,225,253,237,93,221,
    54,201,85,102,60,142,246,171,237,86,24,70,166,218,242,253,
    106,203,152,56,216,222,55,58,169,154,168,122,45,169,33,179,
    238,194,24,99,19,125,28,236,12,83,200,63,99,42,125,241,
    131,182,225,151,69,121,145,44,36,218,48,62,186,145,159,48,
    31,42,118,180,113,97,164,65,144,35,49,68,224,227,65,20,
    199,179,220,57,126,191,61,182,68,48,90,203,143,17,149,232,
    94,199,20,5,156,173,36,241,196,18,240,5,135,80,252,172,
    213,219,215,162,157,193,100,216,32,44,83,27,206,22,137,43,
    240,106,28,4,241,44,140,66,127,200,134,6,237,119,97,195,
    138,224,177,36,136,188,200,104,156,97,154,231,191,121,181,100,
    181,157,12,131,249,49,14,151,16,1,18,20,168,12,8,140,
    201,67,238,71,53,75,26,138,56,39,117,250,22,86,216,236,
    174,129,188,1,114,5,228,234,216,255,51,11,66,249,100,16,
    110,224,96,75,60,23,31,145,50,123,236,163,127,172,214,86,
    143,106,141,27,104,3,53,99,161,178,142,106,198,65,179,141,
    55,64,89,84,170,209,166,228,33,90,59,106,75,148,161,140,
    184,32,176,58,42,19,137,152,91,65,36,102,199,8,119,1,
    219,105,236,238,76,97,215,69,178,4,184,238,234,184,93,122,
    144,72,33,235,162,111,75,241,157,12,121,21,228,205,51,143,
    251,17,248,118,94,1,223,77,216,80,201,192,87,22,208,21,
    249,169,88,109,59,75,198,228,98,93,60,1,58,32,206,57,
    5,113,239,96,101,191,234,254,255,5,182,204,233,111,167,192,
    6,59,173,105,223,54,121,49,92,134,75,211,48,91,230,113,
    225,81,184,204,19,128,37,19,192,135,50,1,200,20,33,131,
    81,218,204,109,233,231,233,34,135,216,116,108,90,202,110,246,
    164,192,148,61,123,49,172,70,157,170,17,231,209,123,111,93,
    75,214,175,37,55,185,171,86,55,164,159,165,125,53,237,156,
    177,30,160,243,97,235,55,47,218,90,174,81,121,243,188,180,
    209,121,210,244,188,236,122,102,196,93,68,100,173,113,200,165,
    229,39,38,70,167,63,219,160,23,39,65,135,15,119,113,106,
    81,34,110,171,101,70,87,81,137,105,94,218,234,101,116,147,
    175,252,124,133,44,192,125,77,152,154,221,70,106,184,248,4,
    239,220,247,143,33,232,172,60,114,235,124,196,247,99,228,228,
    143,144,131,199,30,87,197,175,60,5,41,128,231,23,2,54,
    24,2,89,85,76,138,8,96,88,172,102,38,143,212,105,19,
    132,244,164,6,166,6,145,224,86,149,220,16,209,116,160,184,
    75,191,77,213,222,248,218,183,179,249,117,250,218,119,38,253,
    76,64,245,159,174,118,231,120,227,67,150,186,173,4,98,105,
    55,59,42,231,163,187,99,50,109,114,55,63,51,132,205,166,
    231,121,48,237,233,17,190,112,113,94,86,139,214,20,106,62,
    2,249,120,2,24,53,230,157,133,149,87,233,245,215,189,151,
    222,35,79,96,138,35,198,207,207,72,124,79,252,103,147,154,
    125,105,18,228,97,226,130,227,206,131,88,227,110,193,134,243,
    72,107,134,50,39,165,71,78,88,104,25,155,60,10,165,147,
    61,6,1,247,109,202,26,180,139,169,195,93,167,236,14,19,
    156,167,61,38,212,207,165,203,72,226,221,79,192,199,184,113,
    231,126,230,163,28,210,142,66,142,82,104,100,203,228,147,129,
    181,247,116,63,138,135,167,10,151,79,126,54,128,212,131,88,
    119,180,105,119,79,221,50,255,170,128,129,253,155,175,11,255,
    241,237,171,255,38,42,179,98,251,89,43,62,251,190,34,185,
    187,149,246,239,13,204,15,201,6,19,12,127,133,249,130,202,
    91,248,71,196,230,126,89,86,142,93,170,20,156,210,92,193,
    41,204,216,114,59,151,25,231,69,167,80,42,171,130,53,253,
    252,3,91,114,184,188,
};

EmbeddedPython embedded_m5_internal_enum_X86IntelMPAddressType(
    "m5/internal/enum_X86IntelMPAddressType.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/enum_X86IntelMPAddressType.py",
    "m5.internal.enum_X86IntelMPAddressType",
    data_m5_internal_enum_X86IntelMPAddressType,
    1622,
    4239);

} // anonymous namespace
