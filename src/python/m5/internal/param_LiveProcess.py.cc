#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_LiveProcess[] = {
    120,156,197,89,95,115,219,198,17,223,3,64,74,164,36,75,
    178,44,201,182,100,139,254,35,155,113,99,49,77,227,56,51,
    113,221,186,137,59,147,76,163,184,80,82,59,74,166,40,68,
    156,40,80,36,192,1,142,146,152,145,218,153,200,211,246,181,
    15,157,233,23,232,67,191,77,63,77,95,219,221,61,28,5,
    73,164,146,153,214,172,69,158,23,139,197,222,237,238,111,247,
    22,199,58,100,255,10,248,253,121,5,32,253,151,0,8,240,
    35,160,5,208,22,176,41,64,72,1,193,21,216,45,64,242,
    30,4,5,120,13,176,105,129,180,224,24,9,27,190,182,32,
    154,228,103,138,208,178,153,35,160,87,6,233,192,102,1,94,
    70,179,224,200,34,236,150,33,249,29,8,33,34,1,175,130,
    49,8,198,225,53,106,71,162,196,10,199,129,152,101,102,150,
    32,152,96,102,25,130,73,38,38,160,55,3,114,18,54,167,
    72,108,243,18,170,125,128,106,167,89,237,63,73,109,128,119,
    230,33,184,68,226,184,174,175,72,210,33,73,158,111,154,181,
    204,152,85,206,194,230,101,67,207,229,232,43,57,122,62,71,
    47,228,232,197,28,125,53,71,95,203,209,215,115,244,82,142,
    94,206,209,55,114,244,205,28,189,146,163,43,76,163,229,151,
    161,121,11,154,183,161,121,7,182,49,24,179,125,43,239,130,
    180,161,185,10,155,171,32,241,115,23,142,49,94,193,229,220,
    19,247,248,137,185,254,19,247,249,137,42,108,86,65,226,231,
    190,126,162,8,27,213,5,196,64,248,111,252,87,69,12,128,
    154,196,97,79,38,105,24,71,94,24,109,199,161,69,247,139,
    52,16,98,234,52,140,101,208,249,136,160,243,15,96,220,4,
    86,6,157,35,64,197,130,108,105,89,112,196,196,145,5,189,
    42,28,10,104,58,16,216,112,136,211,20,104,1,13,1,199,
    22,124,99,147,192,17,142,14,6,248,38,56,74,227,166,201,
    1,214,154,198,224,168,0,135,5,216,120,117,104,17,99,183,
    4,201,223,225,219,101,86,58,206,74,45,56,196,209,129,99,
    7,142,138,240,18,133,144,213,44,145,249,226,213,33,90,138,
    156,141,170,131,171,93,207,153,75,166,4,97,18,249,109,169,
    46,35,237,117,252,196,111,123,191,10,247,228,139,36,174,203,
    52,173,150,141,92,156,174,117,124,181,227,242,131,54,121,164,
    221,81,172,48,142,164,154,64,98,59,140,2,175,29,7,221,
    150,84,227,164,205,219,14,91,210,243,248,230,39,237,78,156,
    168,231,73,18,39,46,57,149,153,173,216,239,63,65,46,173,
    183,226,84,86,105,54,158,198,37,245,138,164,183,59,172,145,
    22,192,139,165,135,3,153,214,147,176,163,48,86,90,35,73,
    147,182,42,69,137,135,244,75,28,106,59,113,91,214,226,22,
    26,149,28,244,122,181,78,18,55,208,202,90,67,182,31,61,
    76,149,191,213,146,181,173,110,216,10,106,175,62,120,191,214,
    233,169,157,56,170,181,31,213,194,72,73,116,77,171,118,206,
    41,107,40,68,238,74,247,195,134,23,178,97,222,142,108,117,
    100,50,69,220,235,52,181,152,17,147,162,40,108,81,21,83,
    72,21,240,107,139,101,107,66,172,135,100,90,157,204,37,92,
    57,121,36,81,120,5,236,90,144,44,19,78,154,248,17,20,
    88,68,203,6,221,179,248,222,175,201,39,154,219,180,41,250,
    154,121,200,216,66,144,161,228,19,10,119,4,12,144,2,52,
    139,160,129,131,120,211,72,74,122,52,162,56,169,177,80,185,
    3,233,95,1,125,140,144,57,132,12,78,199,54,136,104,6,
    84,153,170,10,114,23,112,194,239,24,145,27,85,90,254,58,
    227,66,237,132,105,188,31,177,247,137,230,28,218,64,207,188,
    232,125,190,213,148,117,149,174,32,227,171,184,91,169,251,81,
    20,171,138,31,4,21,95,169,36,220,234,42,153,86,84,92,
    89,77,171,20,80,119,214,64,171,175,175,215,49,80,162,176,
    35,148,244,69,16,214,21,94,204,241,5,71,33,149,10,97,
    177,19,7,41,242,73,69,67,42,151,22,169,200,201,49,47,
    132,81,227,145,40,77,143,114,151,240,250,153,89,9,67,179,
    90,52,64,74,101,107,91,149,25,147,126,154,122,188,18,226,
    51,252,72,241,158,223,234,74,214,142,24,82,184,32,34,245,
    26,70,2,192,171,100,140,177,157,13,138,226,40,232,225,250,
    194,250,125,154,250,42,195,112,146,129,56,143,32,28,195,177,
    136,255,23,197,130,85,119,50,232,21,13,252,168,16,42,224,
    224,139,44,254,8,197,99,44,58,85,139,171,6,219,196,89,
    121,155,40,122,216,93,166,225,6,13,55,105,88,49,102,191,
    105,219,167,206,218,254,152,230,179,216,96,54,141,2,100,27,
    211,130,83,153,117,237,36,179,176,56,110,80,134,88,148,71,
    39,25,226,80,33,77,158,210,136,162,156,123,54,164,95,80,
    217,166,76,98,101,148,52,8,127,162,78,146,130,29,229,206,
    144,3,198,13,158,93,2,105,30,169,141,28,82,93,138,17,
    195,212,189,102,106,162,71,18,26,160,238,18,169,42,12,240,
    116,133,134,91,163,114,247,9,212,26,231,160,246,33,77,61,
    147,65,109,138,33,86,198,239,140,85,183,179,24,244,247,202,
    185,51,16,35,124,57,3,240,117,143,40,251,188,213,35,134,
    86,102,235,47,115,208,162,229,89,121,147,214,145,232,45,146,
    37,121,80,45,226,198,255,50,90,196,189,220,226,189,252,29,
    222,203,185,31,224,206,76,23,106,155,107,181,38,10,228,146,
    109,27,22,178,61,58,45,225,136,6,29,244,42,241,118,69,
    177,205,84,87,159,172,166,107,171,233,135,88,49,43,79,185,
    86,233,154,169,171,98,34,59,84,213,232,209,231,7,117,201,
    59,35,95,121,158,46,98,30,23,52,47,219,113,17,95,243,
    228,80,203,120,154,203,121,170,18,170,226,35,241,117,185,239,
    107,90,250,167,52,89,153,29,109,139,69,196,82,89,240,138,
    60,93,189,185,247,226,187,248,253,5,57,159,172,150,64,221,
    186,187,161,215,203,166,144,81,238,219,167,240,242,134,13,113,
    107,168,249,75,131,147,226,9,78,232,107,27,232,255,9,184,
    43,21,240,71,32,36,96,192,51,232,247,51,133,66,63,71,
    226,191,5,206,145,1,189,0,215,155,13,218,255,89,2,203,
    80,250,152,69,117,107,240,41,252,57,151,96,102,3,183,179,
    190,51,191,129,59,253,90,197,16,250,65,155,180,115,186,168,
    81,112,118,252,148,196,116,165,58,201,217,147,237,160,223,46,
    98,165,126,211,120,26,215,211,120,180,162,111,78,208,68,91,
    224,146,152,179,114,24,249,49,13,239,246,225,33,12,239,13,
    46,110,5,134,239,215,158,222,17,190,166,21,56,188,230,233,
    49,222,12,114,58,250,248,47,24,252,191,219,199,191,228,189,
    235,53,191,138,208,104,81,192,143,45,129,239,165,216,186,209,
    107,160,3,178,0,155,69,202,20,110,180,69,150,72,194,84,
    50,170,123,167,54,70,118,202,186,118,87,63,230,58,156,52,
    28,140,164,66,80,68,159,180,252,246,86,224,63,221,163,169,
    104,190,186,73,45,203,44,126,38,191,120,74,11,49,108,253,
    124,249,200,24,177,55,146,234,240,62,205,100,22,207,185,16,
    196,117,46,9,95,236,200,74,91,182,183,240,141,115,39,236,
    84,182,91,126,131,35,99,103,198,125,110,140,83,28,218,179,
    173,70,250,128,198,184,82,143,35,44,219,221,186,138,147,74,
    32,241,53,76,6,149,135,21,174,249,149,48,173,248,91,120,
    215,175,43,13,243,211,153,202,61,173,159,52,82,110,95,119,
    247,137,28,89,100,61,124,199,14,177,145,63,128,254,22,171,
    95,254,250,37,156,91,116,157,53,56,45,190,96,169,158,174,
    89,212,118,184,107,52,188,5,163,172,244,239,161,230,46,77,
    65,238,42,138,37,171,100,169,217,211,137,250,130,158,27,144,
    174,159,136,31,144,174,250,0,41,75,218,34,73,202,49,122,
    199,167,177,68,229,126,179,108,152,19,60,78,50,115,202,48,
    47,241,56,205,204,25,195,156,229,241,50,51,231,12,243,10,
    143,243,204,92,48,204,69,30,175,50,243,154,97,94,231,113,
    137,153,203,134,121,131,199,155,204,92,49,204,10,143,183,152,
    121,219,48,239,240,120,151,153,171,134,121,143,199,251,204,172,
    26,230,91,60,62,96,230,143,204,169,219,219,204,124,8,155,
    107,116,60,68,156,26,85,177,177,255,182,138,113,21,24,89,
    254,255,225,127,90,188,220,199,255,151,181,187,31,64,214,203,
    12,43,92,34,111,216,148,46,92,77,97,94,163,242,86,241,
    137,205,213,65,153,227,213,19,233,43,169,131,180,60,34,67,
    185,252,233,137,191,59,169,70,231,27,254,103,125,155,142,185,
    139,235,93,225,216,233,55,72,142,157,120,25,93,199,206,223,
    225,206,255,9,117,254,135,236,0,207,210,205,255,9,40,11,
    125,63,80,189,139,228,190,119,206,23,186,189,167,181,249,157,
    142,140,2,247,1,228,59,118,190,61,18,28,80,189,253,11,
    228,90,42,91,92,193,22,253,124,6,210,126,146,179,145,131,
    88,232,231,220,168,194,201,184,253,155,193,109,245,29,200,111,
    42,238,19,26,120,27,233,239,32,238,207,250,193,184,54,24,
    148,237,128,222,8,47,184,139,109,29,111,238,72,15,147,218,
    191,80,199,126,78,199,126,160,110,12,148,10,18,114,84,202,
    122,46,150,32,93,124,192,171,175,213,245,129,210,178,17,234,
    69,93,112,155,52,81,92,233,98,200,226,101,180,119,129,105,
    116,215,152,134,244,176,185,186,23,47,165,155,95,10,94,168,
    202,96,185,3,89,239,50,110,88,217,247,10,145,74,66,245,
    9,107,136,21,198,83,195,239,26,27,135,251,169,115,161,142,
    78,78,7,210,67,28,209,233,92,232,167,78,39,231,39,186,
    80,55,7,202,165,116,114,141,89,196,170,190,71,132,212,241,
    219,85,198,24,178,254,238,133,214,117,115,214,33,205,53,133,
    183,128,64,162,207,229,249,234,167,40,71,179,99,179,64,98,
    19,27,247,60,79,159,9,224,117,203,243,70,215,247,253,20,
    53,255,158,166,184,11,220,247,137,34,118,126,243,98,224,159,
    85,42,150,4,183,216,103,126,62,210,43,165,114,163,223,136,
    123,169,75,28,119,186,95,123,248,71,14,211,232,82,153,226,
    243,155,117,191,173,143,167,249,232,213,189,3,217,33,153,123,
    191,95,195,200,227,124,12,161,15,124,112,51,225,246,159,187,
    125,247,39,166,172,181,31,173,25,51,215,54,240,229,62,106,
    120,123,146,94,27,248,71,150,246,35,117,235,140,208,243,118,
    183,133,91,98,240,49,215,16,35,92,57,35,165,61,118,90,
    86,45,13,20,218,8,219,250,87,1,238,154,243,247,131,196,
    71,122,254,12,55,149,73,232,183,194,111,229,185,245,107,125,
    89,136,24,70,3,102,235,165,74,182,207,45,69,70,221,182,
    247,153,108,199,73,239,179,56,144,106,249,204,253,103,65,144,
    184,126,212,144,198,224,179,110,121,150,189,81,105,29,23,187,
    229,180,236,16,183,224,205,204,45,43,3,239,127,212,138,235,
    187,50,200,100,110,12,151,249,56,110,147,27,9,71,230,54,
    31,105,158,246,22,165,80,70,243,27,205,217,70,140,83,46,
    193,114,143,238,75,248,249,211,93,9,101,195,176,146,145,123,
    114,100,233,169,207,9,244,177,230,83,58,55,73,127,131,3,
    253,240,81,154,46,97,170,82,187,98,139,50,54,44,142,61,
    57,83,114,38,39,74,78,105,204,230,179,234,41,49,103,149,
    157,210,196,164,184,232,111,5,83,186,108,173,44,149,196,127,
    0,93,31,166,126,
};

EmbeddedPython embedded_m5_internal_param_LiveProcess(
    "m5/internal/param_LiveProcess.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_LiveProcess.py",
    "m5.internal.param_LiveProcess",
    data_m5_internal_param_LiveProcess,
    2597,
    8255);

} // anonymous namespace
