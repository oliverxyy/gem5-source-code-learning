#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BiModeBP[] = {
    120,156,189,88,109,115,219,198,17,222,3,64,74,164,36,139,
    122,183,45,57,98,219,113,194,122,26,177,77,163,56,51,81,
    221,68,153,116,166,153,169,226,130,201,216,97,50,69,33,226,
    68,130,2,1,14,112,178,204,140,212,15,149,251,242,7,250,
    3,250,161,31,250,111,250,143,154,221,61,0,4,37,122,226,
    153,86,180,200,243,113,177,216,219,151,103,247,246,174,3,233,
    191,18,126,63,174,3,36,177,0,240,240,35,32,0,24,8,
    104,11,16,82,128,183,14,167,37,136,223,7,175,4,175,0,
    218,6,72,3,174,112,98,194,55,6,132,139,252,78,25,2,
    147,41,2,70,85,144,22,180,75,240,44,92,1,75,150,225,
    180,10,241,31,65,8,17,10,120,238,205,129,55,15,175,80,
    58,78,42,44,112,30,136,88,101,98,5,188,5,38,86,193,
    91,228,201,2,140,106,32,23,161,189,68,108,237,59,40,246,
    17,138,93,102,177,255,33,177,30,62,217,0,239,14,177,163,
    94,95,19,167,69,156,188,222,50,75,169,101,90,174,64,123,
    53,155,175,21,230,235,133,249,6,207,113,213,85,232,111,66,
    127,11,250,119,225,4,29,177,146,175,112,15,164,9,253,251,
    208,190,15,18,63,247,224,10,125,229,173,22,222,216,230,55,
    214,242,55,118,248,141,7,208,126,0,18,63,59,250,141,50,
    180,26,155,232,127,255,191,248,175,129,254,7,181,136,195,11,
    25,39,126,20,58,126,120,18,249,6,61,47,211,64,209,234,
    208,48,151,134,237,83,10,219,191,129,99,230,25,105,216,46,
    1,5,11,178,37,48,224,146,39,151,6,140,26,112,33,160,
    111,129,103,194,5,46,83,34,5,186,2,174,12,248,214,36,
    134,75,28,45,116,238,91,96,41,29,179,62,59,87,75,154,
    131,203,18,92,148,160,245,252,194,32,194,105,5,226,127,193,
    119,59,44,116,158,133,26,112,129,163,5,87,22,92,150,225,
    25,50,33,169,95,33,243,197,243,11,180,20,41,173,134,133,
    218,30,21,204,37,83,60,63,14,221,129,84,203,56,119,134,
    110,236,14,156,67,255,119,145,39,15,159,54,170,25,83,148,
    236,13,93,213,179,249,45,147,220,49,24,42,150,22,133,82,
    45,224,228,196,15,61,103,16,121,103,129,84,243,36,202,57,
    241,3,233,56,252,240,183,131,97,20,171,207,226,56,138,109,
    242,40,19,131,200,205,223,32,127,118,130,40,145,13,90,141,
    151,177,73,188,34,238,147,33,75,36,5,88,83,122,217,147,
    73,39,246,135,10,3,165,37,18,55,73,107,80,136,120,72,
    108,28,154,189,104,32,155,81,224,99,84,95,142,70,205,97,
    28,117,209,196,102,87,14,246,223,77,148,123,28,200,230,241,
    153,31,120,205,231,31,126,208,28,142,84,47,10,155,131,253,
    166,31,42,137,126,9,154,147,30,217,67,142,85,146,125,238,
    119,29,159,173,114,122,50,24,202,120,137,168,247,105,93,81,
    19,139,162,44,76,209,16,75,56,43,225,215,20,59,198,130,
    56,242,201,174,14,217,74,136,178,138,24,162,192,10,56,53,
    32,222,33,132,244,241,35,40,164,136,147,22,61,51,248,217,
    239,201,33,154,218,55,41,238,154,120,193,168,66,120,33,231,
    1,5,58,4,134,70,9,250,101,208,144,65,164,105,12,197,
    35,26,145,157,196,24,40,220,130,228,31,128,14,70,176,92,
    64,10,164,43,19,68,88,3,85,165,92,70,234,38,46,248,
    103,198,98,171,65,234,31,49,40,84,207,79,162,243,144,93,
    79,115,206,158,22,122,230,233,232,139,227,190,236,168,100,23,
    9,95,71,103,245,142,27,134,145,170,187,158,87,119,149,138,
    253,227,51,37,147,186,138,234,15,147,6,69,211,94,201,112,
    149,203,27,13,51,28,81,204,17,71,250,135,231,119,20,254,
    88,227,31,28,133,68,42,196,68,47,242,18,164,147,136,174,
    84,54,41,169,200,201,17,43,194,144,113,136,149,150,71,190,
    59,248,251,147,76,19,198,101,163,156,161,40,145,193,137,170,
    50,32,221,36,113,88,19,162,51,246,72,240,11,55,56,147,
    44,29,1,164,80,33,154,106,29,110,31,125,119,201,146,204,
    112,182,38,140,66,111,132,202,249,157,119,104,221,187,140,193,
    69,70,225,6,34,112,14,199,50,254,95,22,155,70,199,74,
    113,87,206,176,71,245,79,1,71,94,164,193,71,28,94,97,
    173,105,24,92,44,216,32,206,199,31,211,140,94,182,119,104,
    120,64,195,91,52,236,102,54,223,170,225,75,215,13,127,76,
    139,25,108,45,219,69,161,49,51,187,188,137,156,186,55,206,
    41,44,136,45,202,13,131,50,104,156,27,22,21,207,248,9,
    141,200,202,89,103,66,242,37,149,106,202,33,22,70,233,130,
    192,167,217,56,29,216,75,118,141,172,159,207,144,108,19,60,
    139,24,237,22,48,106,83,128,24,160,246,189,172,20,58,196,
    161,161,105,111,147,168,210,20,55,215,105,248,209,76,124,61,
    6,89,247,6,200,62,162,117,107,41,200,150,24,92,85,252,
    214,140,142,153,6,32,223,28,215,174,129,139,144,101,77,65,
    214,219,52,51,111,154,60,75,80,165,134,254,166,0,42,210,
    205,40,218,115,132,147,209,22,153,81,132,211,22,110,243,207,
    194,45,220,185,13,222,185,127,206,59,55,239,254,220,3,233,
    226,108,114,125,214,147,18,249,227,196,132,205,116,71,78,42,
    56,162,53,47,71,245,232,164,174,216,96,170,165,7,15,147,
    189,135,201,71,88,37,235,79,184,62,233,58,169,43,97,44,
    135,84,201,232,213,207,94,118,36,111,133,252,203,113,116,225,
    114,184,136,57,233,22,139,200,218,32,111,26,153,155,185,132,
    39,42,166,202,125,251,142,174,230,142,38,189,63,167,149,170,
    236,101,83,108,33,138,170,130,213,113,116,185,230,54,139,159,
    226,247,144,60,79,38,75,160,166,216,110,105,101,217,14,178,
    200,254,217,4,82,110,211,10,187,137,98,191,202,16,82,30,
    35,132,190,102,134,248,191,1,119,159,2,254,10,132,1,12,
    117,138,248,60,65,40,232,107,196,254,7,224,212,152,178,243,
    115,141,105,209,110,207,28,88,122,146,199,204,170,27,129,207,
    225,239,133,188,202,182,107,51,237,47,139,219,181,149,215,39,
    6,207,27,109,201,214,100,33,163,200,244,220,132,216,116,117,
    26,167,234,184,254,231,157,33,86,231,91,69,210,188,94,195,
    33,117,190,29,227,136,54,188,109,177,102,20,208,241,11,26,
    222,203,129,33,50,218,109,105,182,11,175,223,154,29,93,255,
    191,161,229,45,86,120,121,142,77,201,4,228,152,47,101,152,
    127,47,199,188,228,109,234,21,159,52,104,52,40,206,87,134,
    192,35,31,246,103,116,194,178,64,150,160,93,166,236,224,86,
    90,164,201,35,178,210,69,133,110,98,15,100,119,28,105,71,
    229,161,214,81,164,225,229,237,151,4,178,254,32,112,7,199,
    158,251,228,148,214,161,197,58,89,58,25,153,230,181,162,230,
    148,10,226,117,202,243,207,253,204,130,23,183,95,14,62,64,
    177,185,230,12,126,47,234,112,13,248,178,39,235,3,57,56,
    198,163,100,207,31,214,79,2,183,203,49,49,83,203,190,200,
    44,83,28,212,235,253,68,242,136,198,168,222,137,66,172,208,
    103,29,21,197,117,79,226,17,75,122,245,119,235,92,222,235,
    126,82,119,143,241,169,219,81,26,218,147,169,201,45,171,27,
    119,19,238,78,79,207,105,58,155,152,58,120,114,246,177,73,
    31,64,190,149,234,83,93,94,173,185,253,214,153,130,107,226,
    225,73,141,116,133,162,222,194,222,163,225,167,48,179,162,254,
    62,48,186,32,33,71,149,197,182,81,49,88,193,140,231,41,
    189,145,220,204,207,127,190,73,126,234,203,152,52,75,203,196,
    41,231,232,204,78,99,133,202,122,187,154,17,23,120,92,100,
    226,82,70,188,195,227,50,19,107,25,113,133,199,85,38,174,
    101,55,67,235,76,220,128,246,38,93,163,16,101,139,202,193,
    220,255,90,14,56,163,102,147,75,234,255,90,5,236,199,179,
    87,220,254,16,210,46,224,117,21,64,20,173,90,210,239,246,
    69,118,232,40,154,196,215,26,27,55,128,232,116,98,233,42,
    169,99,179,51,11,19,185,130,232,85,207,199,57,125,179,61,
    254,36,183,230,138,59,159,209,58,135,76,159,180,56,100,226,
    89,120,31,251,100,139,251,228,3,234,147,47,216,116,199,208,
    173,242,24,136,165,220,3,116,221,18,202,115,103,210,11,186,
    19,38,197,220,225,80,134,158,253,8,138,205,45,63,190,253,
    216,83,189,250,19,20,122,16,83,172,99,55,123,51,223,168,
    24,23,172,227,216,149,242,12,155,73,20,25,168,127,201,128,
    218,224,83,104,94,145,237,3,26,184,6,231,229,215,254,117,
    30,131,250,20,20,246,34,191,35,63,85,241,161,175,18,58,
    61,253,48,19,118,69,124,224,154,160,170,183,95,247,218,211,
    88,210,109,79,20,183,252,239,36,175,240,134,172,180,206,90,
    190,206,196,179,105,74,118,131,232,216,13,126,192,146,73,166,
    204,146,9,234,52,245,52,195,27,89,50,141,53,179,100,202,
    51,198,30,151,7,79,6,82,201,107,249,161,40,150,233,237,
    131,39,177,77,136,70,120,32,228,51,22,254,14,28,103,70,
    155,235,175,82,192,37,235,192,155,171,40,227,246,186,33,248,
    207,168,148,43,130,59,150,107,215,236,90,45,58,91,232,19,
    197,136,181,3,123,57,7,36,223,7,103,173,3,97,151,79,
    190,71,238,64,95,230,241,93,149,253,19,72,239,22,236,119,
    114,96,211,149,11,31,227,244,81,25,11,11,119,83,220,60,
    217,191,36,58,221,176,12,246,247,50,155,246,82,155,98,55,
    236,244,242,8,240,189,244,96,95,109,79,101,110,249,3,125,
    1,170,86,174,61,247,98,23,231,27,215,168,137,140,125,55,
    32,120,146,93,25,153,217,166,175,78,186,31,210,207,175,176,
    211,226,6,102,98,163,224,176,199,178,235,39,40,73,63,153,
    20,144,214,80,10,14,155,112,13,138,197,215,103,131,19,125,
    24,208,55,21,79,232,70,44,249,24,7,186,194,172,44,87,
    16,51,84,89,77,81,197,218,106,153,139,181,138,181,184,80,
    177,42,115,38,223,61,45,225,169,175,106,85,22,22,133,254,
    219,69,84,85,141,221,181,138,248,30,70,64,47,2,
};

EmbeddedPython embedded_m5_internal_param_BiModeBP(
    "m5/internal/param_BiModeBP.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_BiModeBP.py",
    "m5.internal.param_BiModeBP",
    data_m5_internal_param_BiModeBP,
    2302,
    7014);

} // anonymous namespace