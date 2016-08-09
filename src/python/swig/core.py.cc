#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_core[] = {
    120,156,181,89,235,111,219,200,17,31,62,36,91,178,125,182,
    227,216,206,195,137,149,228,146,40,185,92,220,199,61,10,92,
    154,54,118,124,105,174,23,39,71,231,144,156,238,80,150,230,
    174,37,218,20,169,35,87,137,117,176,139,162,14,218,162,40,
    250,161,64,251,161,31,250,161,64,11,180,127,77,255,163,118,
    102,150,164,228,60,138,43,44,89,214,122,56,92,206,206,227,
    183,51,179,180,15,217,79,9,191,63,174,1,164,210,4,16,
    248,107,64,8,208,54,160,97,128,33,13,16,231,97,183,4,
    201,123,32,74,240,2,160,97,2,78,60,68,194,130,47,77,
    136,38,249,153,50,132,22,115,12,232,85,65,218,208,40,193,
    147,104,22,108,89,134,221,42,36,63,7,195,48,34,3,158,
    138,49,16,227,240,2,165,35,81,97,129,227,64,204,42,51,
    43,32,38,152,89,5,49,201,196,4,244,102,64,78,66,99,
    138,166,53,222,66,177,215,81,236,52,139,253,55,137,21,120,
    103,25,196,91,52,29,245,250,130,102,218,52,147,215,155,102,
    41,51,32,88,202,54,218,51,91,76,156,5,105,193,206,9,
    104,156,0,137,191,179,112,72,38,35,107,14,26,115,32,231,
    96,231,36,52,78,130,56,193,50,230,121,246,60,17,98,142,
    57,11,204,89,32,66,156,100,206,34,115,22,115,226,20,141,
    185,208,211,208,56,205,220,51,131,220,179,208,56,11,98,158,
    159,94,226,219,75,68,136,5,230,156,99,206,57,34,196,34,
    115,206,51,231,60,17,226,20,115,150,153,179,76,132,56,205,
    156,26,115,106,68,136,51,204,185,192,156,11,68,136,179,204,
    185,200,156,139,68,136,37,230,92,98,206,37,34,196,57,230,
    188,205,156,183,153,64,103,92,134,198,101,38,174,64,227,10,
    19,87,161,113,149,137,58,52,234,76,92,131,198,53,38,174,
    67,227,58,19,239,64,227,29,38,110,64,227,6,33,101,179,
    142,104,131,224,63,248,83,55,144,82,147,56,60,147,73,26,
    196,145,27,68,219,113,96,210,253,50,13,4,80,159,134,177,
    12,169,107,132,212,127,2,195,84,152,25,82,15,0,12,186,
    6,8,77,56,96,226,192,132,94,29,246,13,216,177,65,88,
    176,143,203,148,200,231,77,3,14,77,248,202,162,9,7,56,
    218,136,167,243,96,43,13,211,29,198,147,150,52,6,7,37,
    216,47,193,230,211,125,147,24,187,21,72,254,14,223,44,177,
    208,113,22,106,194,62,142,54,28,218,112,80,134,39,56,9,
    89,59,21,2,151,241,116,31,45,69,206,102,221,70,109,55,
    6,204,37,83,68,144,68,94,91,42,50,201,245,227,68,214,
    171,249,173,56,189,217,241,84,203,225,185,22,57,161,221,81,
    44,35,142,164,154,64,98,59,136,132,219,142,69,55,148,106,
    156,4,184,219,65,40,93,151,111,222,111,119,226,68,173,39,
    73,156,56,228,71,102,134,177,87,60,65,75,250,97,156,202,
    58,173,198,203,56,36,94,209,236,237,14,75,36,5,88,63,
    122,88,200,212,79,130,142,194,240,104,137,52,155,164,213,41,
    48,60,164,119,112,88,105,197,109,185,18,135,1,198,114,175,
    215,91,233,36,113,51,241,218,43,77,217,126,255,221,84,121,
    91,161,92,217,234,6,161,88,121,250,131,15,86,58,61,213,
    138,163,149,244,121,208,92,33,7,220,68,198,9,18,133,12,
    55,96,35,220,150,12,59,50,153,34,238,25,90,198,152,49,
    38,141,178,97,25,117,99,10,169,18,126,45,99,201,156,48,
    54,2,50,195,39,211,8,54,246,32,80,40,122,6,236,154,
    144,44,17,12,118,240,215,160,184,33,24,54,233,158,201,247,
    62,35,251,53,119,199,162,224,106,230,62,67,7,49,132,51,
    111,81,52,35,224,248,151,96,167,12,26,23,8,39,13,148,
    164,71,35,78,39,49,38,10,183,33,253,19,160,63,17,17,
    251,144,161,229,208,2,35,154,1,85,165,28,133,220,5,92,
    240,87,12,184,205,58,169,191,193,24,80,173,32,141,159,71,
    236,105,162,121,139,108,162,103,30,245,30,110,237,72,95,165,
    203,200,248,34,238,214,124,47,138,98,85,243,132,168,121,74,
    37,193,86,87,201,180,166,226,218,229,180,78,193,115,102,115,
    24,21,242,122,157,28,54,20,98,132,141,190,16,129,175,240,
    98,142,47,56,10,169,84,8,129,86,44,82,228,147,136,166,
    84,14,41,169,200,201,49,43,194,8,113,105,42,45,143,243,
    222,194,235,59,185,38,12,195,122,57,7,77,42,195,109,85,
    101,252,121,105,234,178,38,196,47,182,194,51,47,236,74,150,
    142,120,81,168,16,145,90,135,161,131,237,20,41,158,219,201,
    202,71,113,36,122,168,75,224,95,165,101,78,49,228,38,25,
    116,243,8,184,49,28,203,248,183,108,44,152,190,157,193,172,
    156,67,109,129,140,4,14,180,145,197,26,97,119,136,249,163,
    110,114,2,96,253,121,183,93,36,138,30,118,150,104,56,71,
    195,121,26,150,115,19,135,105,231,212,203,118,126,72,178,77,
    54,142,205,32,199,91,185,25,226,200,142,57,221,223,49,152,
    211,54,9,249,38,237,143,62,242,109,202,127,201,109,26,113,
    42,239,41,11,210,199,148,109,105,135,176,48,218,12,8,107,
    162,250,96,103,167,56,51,100,236,120,142,83,135,192,55,136,
    192,230,0,2,29,138,7,195,207,57,157,231,53,151,102,104,
    224,57,103,73,84,233,53,94,173,209,112,97,20,174,237,67,
    168,249,10,132,62,162,101,102,50,8,77,49,116,170,248,157,
    49,125,43,243,119,81,206,230,94,130,14,225,198,126,13,110,
    174,16,101,189,106,225,8,33,147,217,245,241,0,100,72,21,
    115,80,253,13,36,122,139,164,245,32,88,22,177,14,63,137,
    22,177,180,154,92,90,191,195,165,149,203,51,247,101,58,177,
    90,156,91,53,81,34,243,183,45,88,200,74,102,90,193,17,
    149,223,235,213,226,237,154,98,251,40,15,222,186,156,222,188,
    156,126,132,25,174,118,155,115,139,206,113,58,139,37,178,67,
    89,136,30,93,223,243,37,87,45,190,114,93,157,116,92,78,
    64,110,86,13,17,55,243,228,60,51,247,42,167,223,84,37,
    148,117,135,238,215,106,225,87,82,243,19,18,92,101,167,90,
    198,34,98,164,106,240,234,174,206,172,220,246,240,93,252,174,
    146,163,201,66,9,212,151,59,155,90,55,86,155,12,112,110,
    28,193,193,16,149,118,86,80,202,231,121,252,203,253,248,211,
    215,202,225,251,27,224,214,218,128,95,3,69,24,3,153,193,
    183,64,59,133,116,142,166,255,12,24,231,175,169,201,156,31,
    54,169,14,243,12,76,27,233,135,60,85,151,232,79,224,183,
    3,155,36,47,164,86,214,222,13,22,82,187,200,45,12,141,
    111,85,44,237,163,73,136,2,209,242,82,154,166,51,75,127,
    223,245,83,117,209,162,97,102,29,38,78,198,181,72,151,86,
    255,170,143,18,42,69,103,141,57,115,32,246,223,165,225,123,
    69,216,141,156,55,36,69,150,225,205,53,210,213,153,249,75,
    90,205,102,253,166,199,184,94,175,245,252,16,91,198,28,190,
    165,28,190,119,11,248,74,46,31,47,184,137,167,209,164,24,
    30,154,6,30,32,245,209,17,123,35,58,127,148,65,142,81,
    175,77,199,194,82,118,44,68,232,211,34,12,195,252,203,105,
    136,146,214,145,106,197,222,216,208,126,42,2,171,99,70,195,
    222,208,183,55,133,237,86,232,181,183,132,119,187,73,98,73,
    182,159,239,21,51,87,116,102,80,81,194,185,241,38,93,249,
    242,253,92,225,103,67,223,218,31,160,148,66,81,6,178,136,
    125,222,207,143,91,178,214,150,237,45,60,149,181,130,78,109,
    59,244,154,236,241,188,110,221,203,3,170,114,43,126,175,207,
    92,39,217,180,188,73,160,67,238,147,232,12,22,1,155,139,
    192,143,168,8,236,243,6,119,77,93,7,250,129,226,218,205,
    7,12,130,123,36,159,187,26,73,58,189,83,208,189,78,71,
    70,194,185,126,100,63,114,7,233,37,77,61,111,36,65,117,
    241,88,26,96,115,188,211,223,139,211,152,179,79,98,206,126,
    21,135,164,206,128,85,188,37,75,197,230,92,26,129,142,28,
    199,175,243,56,234,182,191,40,10,220,144,235,61,140,130,241,
    56,165,122,58,51,222,164,225,26,13,183,10,199,211,30,18,
    18,15,137,50,243,189,154,129,162,25,195,115,160,74,226,30,
    86,80,174,82,120,29,186,238,104,106,206,123,40,101,59,119,
    118,25,147,94,217,172,148,43,125,103,23,123,105,170,15,65,
    120,101,35,21,88,74,226,246,131,192,79,226,71,107,253,148,
    221,237,248,67,87,155,164,236,193,209,86,233,255,84,151,102,
    68,113,210,246,194,65,141,157,213,81,120,121,77,39,162,227,
    168,75,253,88,144,58,71,253,59,26,109,41,219,252,242,248,
    218,98,33,123,216,85,157,174,186,27,36,125,52,8,188,24,
    182,194,235,40,229,119,3,10,67,246,83,100,138,106,161,240,
    203,229,170,191,25,227,245,189,64,173,133,210,139,186,157,209,
    236,181,123,40,229,15,199,80,115,142,221,151,210,74,119,194,
    240,211,32,85,50,194,178,49,26,101,127,130,82,254,120,60,
    16,84,25,4,82,56,94,36,226,182,134,128,157,241,134,174,
    239,125,148,242,231,227,233,59,171,65,187,22,198,254,238,199,
    137,252,186,43,35,191,167,213,166,212,174,2,127,55,125,36,
    147,77,233,99,107,54,116,3,126,138,82,254,114,12,116,80,
    169,240,187,201,99,212,114,52,136,120,128,82,254,122,60,15,
    235,180,144,4,94,24,124,67,24,238,191,166,245,59,202,29,
    69,106,160,35,243,223,142,167,52,205,192,211,249,90,75,250,
    187,157,56,136,148,78,189,15,71,225,227,71,40,229,31,199,
    83,151,94,237,118,163,194,203,247,194,120,203,11,83,237,105,
    147,61,61,116,173,63,67,41,255,234,107,205,29,227,75,255,
    90,208,107,190,11,249,49,174,151,58,196,113,166,11,189,249,
    109,120,222,55,253,144,56,244,50,97,195,107,235,119,155,252,
    46,207,185,4,217,219,25,135,94,28,234,206,138,222,81,241,
    217,89,191,125,192,118,150,223,63,240,97,210,249,62,13,212,
    228,112,246,212,173,22,55,89,137,108,82,2,77,56,35,249,
    207,188,132,97,248,192,219,163,13,196,253,5,87,109,46,134,
    124,58,91,13,154,235,145,8,188,104,181,167,228,195,68,200,
    132,229,96,46,64,31,69,138,253,254,234,28,69,106,127,26,
    40,21,202,255,249,244,252,155,166,113,113,227,210,193,41,153,
    243,28,231,10,222,143,140,111,70,13,7,129,177,154,53,8,
    216,42,172,162,192,204,170,59,66,36,138,156,125,63,122,134,
    184,16,143,91,137,244,196,253,187,252,68,198,123,20,39,10,
    57,212,213,250,113,187,19,132,242,174,167,36,167,113,58,150,
    184,119,215,87,63,191,167,255,47,66,151,27,250,122,58,191,
    126,236,220,89,187,191,113,207,125,184,49,146,252,163,15,125,
    250,101,210,109,82,34,253,5,14,244,202,184,50,93,193,182,
    149,254,67,97,25,85,60,41,216,214,228,76,197,158,156,168,
    216,149,49,139,223,6,78,225,89,190,106,87,38,22,38,43,
    120,85,49,42,86,197,236,127,166,12,253,253,86,31,99,240,
    243,95,38,222,67,189,
};

EmbeddedPython embedded_m5_internal_core(
    "m5/internal/core.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/swig/core.py",
    "m5.internal.core",
    data_m5_internal_core,
    2518,
    7643);

} // anonymous namespace
