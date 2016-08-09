#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_FUDesc[] = {
    120,156,189,88,91,115,219,198,21,62,11,128,148,72,81,150,
    100,89,146,109,41,22,210,142,19,198,211,136,109,26,197,153,
    137,226,54,205,101,38,153,142,236,130,201,216,97,50,69,33,
    98,73,129,2,1,14,176,146,204,140,212,135,202,189,76,223,
    251,19,250,208,127,211,127,212,158,115,22,0,33,202,154,201,
    76,75,57,226,102,177,151,179,231,242,157,203,110,23,178,127,
    21,252,253,218,6,72,255,46,0,124,252,19,16,2,12,5,
    116,4,8,41,192,191,3,71,21,72,222,7,191,2,175,0,
    58,6,72,3,46,176,99,194,119,6,68,13,222,83,133,208,
    228,17,1,227,58,72,11,58,21,120,30,173,128,37,171,112,
    84,135,228,15,32,132,136,4,188,240,231,192,159,135,87,72,
    29,59,53,38,56,15,52,88,231,193,26,248,11,60,88,7,
    191,193,157,5,24,47,131,108,64,103,145,150,117,110,33,217,
    71,72,118,137,201,254,155,200,250,56,179,6,254,45,90,142,
    124,125,75,43,45,90,201,231,45,49,149,229,156,203,21,232,
    220,206,251,171,165,254,157,82,127,173,212,95,47,245,55,184,
    143,220,220,134,193,93,24,220,131,193,125,232,161,130,86,138,
    147,55,65,154,48,216,130,206,22,72,252,219,132,11,212,161,
    127,187,180,227,13,222,177,90,236,120,192,59,182,161,179,13,
    18,255,30,232,29,85,104,55,215,209,46,193,127,240,95,19,
    237,2,170,129,205,137,76,210,32,142,220,32,234,197,129,65,
    243,85,106,200,138,93,106,230,50,115,126,74,230,252,23,176,
    45,125,35,51,231,57,32,97,65,178,132,6,156,115,231,220,
    128,113,19,206,4,12,44,240,77,56,195,99,42,196,64,95,
    192,133,1,223,155,180,224,28,91,11,149,254,0,44,165,109,
    57,96,165,107,74,115,112,94,129,179,10,180,95,156,25,52,
    112,84,131,228,159,240,195,22,19,157,103,162,6,156,97,107,
    193,133,5,231,85,120,142,139,112,104,80,35,241,197,139,51,
    148,20,71,218,77,11,185,221,47,137,75,162,248,65,18,121,
    67,169,22,177,239,142,188,196,27,186,95,124,243,153,76,187,
    205,122,190,36,78,119,70,158,58,116,120,143,73,202,24,142,
    20,211,138,35,169,22,176,211,11,34,223,29,198,254,113,40,
    213,60,17,114,123,65,40,93,151,39,191,28,142,226,68,125,
    158,36,113,226,144,62,121,48,140,189,98,7,105,179,27,198,
    169,108,210,105,124,140,67,228,21,173,238,141,152,34,49,192,
    124,210,102,31,217,75,130,145,66,51,105,138,180,154,168,53,
    201,64,220,164,207,176,105,29,198,67,217,138,195,0,109,250,
    114,60,110,141,146,184,143,2,182,250,114,184,251,110,170,188,
    131,80,182,14,142,131,208,111,189,248,240,131,214,104,172,14,
    227,168,53,220,109,5,145,146,168,149,176,85,214,199,14,206,
    223,38,202,167,65,223,13,88,38,247,80,134,35,153,144,234,
    210,251,116,170,88,22,13,81,21,166,104,138,69,236,85,240,
    103,138,45,99,65,236,7,36,85,151,36,37,52,89,101,252,
    144,81,5,28,25,144,108,17,58,6,248,39,200,156,136,145,
    54,205,25,60,247,59,82,135,30,29,152,100,115,61,120,198,
    136,66,104,225,202,61,50,114,4,12,139,10,12,170,160,225,
    130,40,211,248,73,198,212,226,114,34,99,32,113,11,210,127,
    0,170,23,129,114,6,25,136,46,76,16,209,50,168,58,249,
    55,142,174,227,129,127,98,28,182,155,196,254,62,67,66,29,
    6,105,124,26,177,226,169,207,158,211,70,205,60,27,63,61,
    24,200,174,74,183,113,224,219,248,216,238,122,81,20,43,219,
    243,125,219,83,42,9,14,142,149,76,109,21,219,15,211,38,
    217,210,89,201,81,85,208,27,143,114,20,145,197,17,69,250,
    195,15,186,10,63,86,249,131,173,144,74,133,136,56,140,253,
    20,199,137,68,95,42,135,152,84,164,228,152,25,97,192,184,
    180,148,142,199,117,183,240,251,147,156,19,70,101,179,154,99,
    40,149,97,79,213,25,142,94,154,186,204,9,141,51,242,136,
    240,137,23,30,75,166,142,240,81,200,16,117,53,15,179,198,
    222,93,146,35,23,155,101,137,226,200,31,35,107,65,247,109,
    58,245,46,35,176,193,24,92,67,252,205,97,91,197,255,87,
    197,186,209,181,50,212,85,115,228,81,228,83,192,118,23,153,
    233,17,133,23,24,101,154,6,135,9,22,135,125,241,39,212,
    163,205,206,22,53,111,80,243,128,154,237,92,226,25,138,189,
    56,45,246,99,58,202,96,89,89,42,50,139,153,75,229,95,
    242,167,123,19,127,194,64,216,38,191,48,200,123,38,126,97,
    81,208,76,158,80,139,75,217,227,76,72,191,166,16,77,254,
    195,196,200,85,16,244,212,155,184,2,235,200,89,38,217,231,
    115,20,59,4,205,50,62,251,37,124,58,100,30,6,167,115,
    47,15,130,46,173,208,176,116,54,137,84,229,53,74,182,169,
    121,243,6,52,61,1,88,255,10,192,62,162,83,151,51,128,
    45,50,176,234,248,91,54,186,102,166,254,34,37,174,78,1,
    139,80,101,189,6,85,111,81,207,188,42,240,205,1,42,19,
    243,139,18,160,136,51,163,44,205,62,118,198,27,36,68,25,
    74,27,152,218,159,71,27,152,173,13,206,214,63,231,108,205,
    25,159,235,33,29,148,77,142,203,186,83,33,109,244,76,88,
    207,178,112,90,195,22,101,121,57,182,227,158,173,88,92,138,
    161,123,15,211,157,135,233,71,24,29,237,39,28,151,116,124,
    212,17,48,145,35,138,96,180,245,243,151,93,201,9,144,191,
    92,87,7,44,151,131,151,155,37,86,68,213,26,233,210,200,
    149,204,161,59,85,9,69,236,89,171,185,94,168,153,184,254,
    138,206,169,179,142,77,177,129,8,170,11,102,198,213,65,154,
    11,43,158,197,223,111,72,239,36,176,4,42,143,157,182,102,
    149,165,32,121,156,159,93,66,201,236,100,112,90,72,244,155,
    28,29,213,9,58,232,103,230,88,255,43,112,181,41,224,47,
    64,246,71,51,103,88,47,92,131,12,190,74,203,127,15,236,
    20,175,201,246,28,91,218,148,225,121,5,134,156,244,49,47,
    213,201,255,43,248,91,201,163,242,20,109,102,245,100,57,69,
    91,69,92,98,224,252,168,52,108,93,14,96,100,151,67,47,
    165,101,58,42,77,156,116,18,245,139,90,16,163,242,12,81,
    52,175,79,112,137,153,239,39,24,162,36,183,41,86,141,18,
    50,126,65,205,123,5,40,68,62,54,27,190,182,225,250,100,
    236,234,152,255,29,29,110,49,187,75,115,92,39,232,237,5,
    214,43,57,214,223,43,176,46,57,49,189,226,59,5,181,6,
    89,248,194,16,120,233,195,106,140,238,88,22,200,10,116,170,
    228,21,92,54,139,204,105,68,30,176,40,188,93,202,122,172,
    138,125,173,164,194,200,218,126,212,188,156,117,32,32,19,238,
    133,222,240,192,247,158,12,233,20,58,170,155,187,145,145,243,
    189,92,230,155,92,64,92,199,58,127,238,230,252,159,204,58,
    8,124,128,68,11,190,25,242,126,220,101,207,255,250,80,218,
    67,57,60,192,11,227,97,48,178,123,161,215,103,123,152,153,
    92,79,115,185,20,27,116,186,122,72,31,81,27,219,221,56,
    194,152,124,220,85,113,98,251,18,175,82,210,183,223,181,57,
    160,219,65,106,123,7,56,235,117,149,134,244,101,135,228,226,
    212,75,250,41,227,235,232,148,186,55,97,79,23,111,199,1,
    22,227,49,20,169,83,223,221,138,248,204,101,182,246,16,60,
    17,47,73,106,172,163,18,85,18,206,14,53,239,192,13,133,
    241,247,145,104,72,212,73,73,85,177,105,212,12,190,175,232,
    21,207,104,117,122,213,43,79,127,140,87,234,71,152,204,55,
    171,180,82,206,209,157,156,218,26,133,241,78,61,31,92,224,
    182,193,131,139,249,59,207,45,30,92,226,183,147,42,143,172,
    144,107,207,253,175,174,205,254,113,19,158,113,242,127,245,104,
    231,241,77,179,237,124,8,89,30,191,206,155,69,89,166,69,
    237,205,3,145,95,23,202,2,241,83,196,234,20,176,220,110,
    34,61,37,181,85,182,102,47,30,71,2,125,230,120,226,157,
    87,11,219,79,10,73,46,184,110,25,223,97,99,233,251,17,
    27,75,60,143,238,99,133,107,113,133,187,71,21,238,25,139,
    237,26,186,200,157,0,176,82,72,79,54,143,228,169,91,214,
    128,174,96,137,45,111,52,146,145,239,60,130,114,81,202,211,
    179,182,57,69,157,63,66,169,126,48,197,29,172,66,175,250,
    24,5,212,146,100,108,181,74,225,85,55,96,63,134,231,159,
    115,120,54,233,197,105,18,85,157,61,106,56,142,22,33,212,
    249,85,161,253,245,43,216,139,143,35,69,247,156,235,166,176,
    122,209,207,111,244,165,54,166,23,197,163,223,6,169,38,112,
    221,28,81,224,215,22,254,100,69,179,15,248,50,148,74,94,
    2,2,163,35,187,26,251,18,179,90,60,198,27,11,95,3,
    240,59,116,221,27,201,6,31,35,209,99,200,226,19,102,3,
    81,197,124,176,38,214,140,90,181,38,56,181,78,189,250,106,
    134,108,200,11,222,113,234,112,228,88,42,244,206,15,148,121,
    150,35,19,241,165,108,223,27,234,247,37,126,64,113,126,10,
    217,165,215,121,187,176,31,189,4,240,45,67,223,226,208,115,
    56,237,115,150,119,126,73,227,180,98,184,187,147,75,179,243,
    116,68,114,184,39,146,202,5,126,32,29,238,242,205,189,188,
    72,139,172,151,94,153,148,209,49,205,125,74,7,169,205,215,
    238,108,7,67,253,138,167,86,166,230,253,196,195,254,218,212,
    104,42,147,192,11,131,31,244,179,93,62,172,72,65,211,4,
    73,206,226,139,51,115,41,98,50,52,18,217,71,32,161,42,
    151,202,75,179,80,242,113,174,147,75,80,44,111,187,9,12,
    233,154,86,95,178,159,208,67,78,250,25,54,244,238,86,91,
    170,33,158,40,188,152,120,197,93,20,150,217,88,174,89,141,
    133,154,85,155,51,249,209,100,17,175,45,117,171,182,208,16,
    147,255,182,17,121,117,99,123,165,38,254,11,101,225,194,86,
};

EmbeddedPython embedded_m5_internal_param_FUDesc(
    "m5/internal/param_FUDesc.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_FUDesc.py",
    "m5.internal.param_FUDesc",
    data_m5_internal_param_FUDesc,
    2256,
    6705);

} // anonymous namespace