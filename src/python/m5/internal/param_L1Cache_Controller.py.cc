#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_L1Cache_Controller[] = {
    120,156,197,89,109,115,219,198,17,222,3,64,74,164,36,75,
    178,94,109,203,22,253,34,155,86,27,169,73,227,56,51,113,
    221,58,78,50,147,76,163,184,144,59,118,24,79,81,136,56,
    73,160,64,128,5,78,182,233,145,102,218,202,211,246,99,191,
    116,250,11,250,161,255,166,255,168,217,221,3,64,232,133,178,
    210,184,180,37,157,151,139,197,222,238,237,179,123,123,199,38,
    164,255,74,248,247,171,26,64,114,206,0,240,240,87,64,0,
    208,22,208,16,32,164,0,111,26,118,74,16,127,8,94,9,
    94,3,52,12,144,6,28,32,97,194,119,6,132,163,252,78,
    25,2,147,57,2,186,85,144,22,52,74,240,36,156,4,75,
    150,97,167,10,241,239,65,8,17,10,120,234,13,129,55,12,
    175,81,59,18,21,86,56,12,196,172,50,179,2,222,8,51,
    171,224,141,50,49,2,221,9,144,163,208,24,35,177,198,57,
    84,187,140,106,199,89,237,127,72,173,135,79,102,192,59,71,
    226,104,215,183,36,105,145,36,207,55,206,90,38,50,43,39,
    161,113,62,163,167,10,244,116,129,158,41,208,179,5,122,174,
    64,207,23,232,11,5,250,98,129,190,84,160,23,10,244,229,
    2,125,165,64,47,22,232,90,129,190,90,160,175,49,141,43,
    114,30,90,215,161,117,3,90,75,176,137,65,154,204,189,191,
    9,210,132,214,45,104,220,2,137,191,55,225,0,227,232,157,
    47,188,81,231,55,166,242,55,110,243,27,203,208,88,6,137,
    191,183,245,27,101,88,175,207,34,54,252,255,226,191,186,64,
    74,141,226,240,92,198,137,31,133,142,31,110,70,190,65,207,
    203,52,16,146,154,52,12,165,144,122,72,144,250,55,48,158,
    60,35,133,212,62,160,98,65,190,4,6,236,51,177,111,64,
    183,14,123,2,90,22,120,38,236,225,52,37,50,96,75,192,
    129,1,207,76,18,216,199,209,194,192,95,1,75,105,60,181,
    56,240,90,211,16,236,151,96,175,4,235,79,247,12,98,236,
    84,32,254,23,188,90,96,165,195,172,212,128,61,28,45,56,
    176,96,191,12,79,80,8,89,173,10,185,47,158,238,161,167,
    200,89,175,91,104,237,90,193,93,114,197,243,227,208,109,75,
    117,1,105,167,227,198,110,219,249,245,251,15,221,230,182,116,
    30,70,161,138,163,32,144,113,189,154,137,71,201,74,199,85,
    219,54,191,111,210,194,180,59,138,245,70,161,84,35,72,108,
    250,161,231,180,35,111,55,144,106,152,148,58,155,126,32,29,
    135,31,126,217,238,68,177,250,60,142,163,216,166,181,101,102,
    16,185,249,27,180,178,205,32,74,100,157,102,227,105,108,82,
    175,72,122,179,195,26,201,0,182,153,94,246,100,210,140,253,
    142,194,144,105,141,36,77,218,234,20,44,30,146,103,56,172,
    110,71,109,185,26,5,62,198,247,101,183,187,218,137,163,45,
    116,118,117,75,182,239,188,151,40,119,35,144,171,27,187,126,
    224,173,62,253,248,163,213,78,87,109,71,225,106,251,206,170,
    31,42,137,43,20,172,246,91,155,21,148,61,79,179,188,240,
    183,28,159,253,115,182,101,208,145,241,24,113,47,146,5,98,
    66,140,138,178,48,69,93,140,33,85,194,63,83,44,24,35,
    98,205,39,15,155,228,53,161,204,42,226,138,130,45,96,199,
    128,120,129,80,211,194,95,65,97,70,236,172,211,51,131,159,
    253,134,150,70,115,91,38,97,65,51,247,24,105,8,57,148,
    188,71,193,15,129,225,82,130,86,25,52,140,16,125,26,87,
    113,151,70,20,39,53,6,42,183,32,249,7,224,82,35,128,
    246,32,5,215,129,9,34,156,0,85,165,218,131,220,89,156,
    240,207,140,207,245,58,153,191,198,240,80,219,126,18,189,8,
    57,8,68,115,70,173,227,202,60,234,126,179,209,146,77,149,
    44,34,227,219,104,183,214,116,195,48,82,53,215,243,106,174,
    82,177,191,177,171,100,82,83,81,109,41,169,83,92,237,201,
    12,97,185,190,110,39,67,20,69,31,17,165,63,120,126,83,
    225,135,41,254,192,81,72,164,66,116,108,71,94,130,124,82,
    177,37,149,77,70,42,90,228,136,13,97,240,56,36,74,211,
    163,220,57,252,252,32,179,132,17,90,47,103,120,74,100,176,
    169,170,12,77,55,73,28,182,132,248,140,66,82,252,220,13,
    118,37,107,71,40,41,52,136,72,109,195,32,113,56,79,62,
    101,75,192,126,133,81,232,117,209,76,191,121,139,44,152,103,
    52,142,50,30,103,16,139,67,56,150,241,255,178,152,53,154,
    86,138,192,114,134,66,170,142,10,24,3,34,133,1,34,242,
    0,43,81,221,224,82,194,174,113,142,94,35,138,94,182,23,
    104,184,76,195,21,26,22,51,239,7,180,4,99,71,151,224,
    46,77,107,176,223,236,33,133,203,204,60,244,14,229,217,133,
    94,158,97,225,92,167,124,49,40,171,122,249,98,81,145,141,
    239,211,136,162,156,137,38,36,143,169,164,83,94,177,50,74,
    33,76,6,162,122,41,194,235,101,79,208,58,12,103,232,182,
    9,178,69,220,110,21,112,107,83,168,24,180,246,133,172,80,
    58,36,161,225,106,95,34,85,165,19,22,188,70,195,213,1,
    175,122,15,120,91,199,128,247,9,89,48,145,2,111,140,1,
    87,197,191,9,163,105,166,161,200,183,211,169,35,128,35,180,
    89,39,160,237,38,81,230,113,231,223,13,208,82,151,191,40,
    0,141,172,52,138,158,173,33,209,157,35,135,138,16,155,195,
    22,225,73,56,135,187,190,193,187,254,207,120,215,231,206,129,
    123,59,93,196,77,174,227,154,40,209,202,108,154,48,155,238,
    230,73,5,71,244,235,101,183,22,109,214,20,187,78,53,247,
    222,82,178,178,148,124,130,213,180,118,159,235,152,174,167,186,
    98,198,178,67,21,143,94,253,252,101,83,242,230,201,159,28,
    71,23,56,135,139,157,147,110,202,136,182,25,90,87,35,91,
    112,46,245,137,138,169,194,15,114,201,171,249,146,147,7,95,
    209,156,85,94,111,83,204,33,178,170,130,13,115,116,129,231,
    102,141,159,226,223,167,20,3,114,94,2,181,253,246,186,54,
    155,61,34,223,236,159,30,66,207,96,252,177,87,113,130,223,
    102,168,41,247,80,67,127,102,150,15,127,5,238,102,5,252,
    5,8,23,24,254,52,31,242,244,33,32,76,145,248,239,128,
    19,231,132,174,129,107,209,58,117,10,44,129,37,42,185,203,
    162,186,137,248,10,254,86,200,186,108,171,55,211,126,181,184,
    213,91,121,29,99,64,157,105,59,183,14,23,60,138,209,182,
    155,144,152,174,98,189,68,238,237,24,121,127,137,85,124,64,
    232,26,214,179,57,100,216,179,30,182,104,179,188,36,166,140,
    2,98,222,167,225,131,28,44,34,227,253,255,109,92,132,254,
    27,188,163,247,142,239,200,16,139,77,31,31,226,206,244,184,
    170,60,55,74,89,110,124,144,231,134,228,205,238,53,159,107,
    104,52,8,5,7,134,192,195,47,118,126,116,214,180,64,150,
    160,81,166,44,226,118,93,164,73,38,178,98,71,165,241,208,
    78,202,75,180,166,23,47,7,130,142,49,13,47,7,89,68,
    40,204,247,2,183,189,225,185,247,95,210,140,52,109,51,75,
    59,35,243,97,162,232,3,165,140,232,231,6,127,188,147,249,
    242,124,144,5,228,35,90,188,204,7,78,23,47,106,114,213,
    120,188,45,107,109,217,222,192,195,236,182,223,169,109,6,238,
    22,199,201,76,125,252,38,243,81,113,160,143,118,42,201,50,
    141,81,173,25,133,88,231,119,155,42,138,107,158,196,163,157,
    244,106,239,213,120,147,168,249,73,205,221,192,167,110,83,233,
    20,56,156,204,220,32,187,241,86,194,189,240,206,11,34,7,
    29,103,7,79,241,62,30,14,94,65,190,53,235,115,101,94,
    243,185,237,215,25,133,179,227,161,77,117,117,117,163,174,197,
    94,161,225,54,188,131,173,225,67,156,224,5,205,68,139,87,
    22,151,140,138,193,77,214,113,233,71,164,37,57,158,209,85,
    113,134,140,214,23,89,105,94,151,73,82,14,209,157,2,141,
    21,218,38,26,213,140,57,194,227,40,51,199,50,230,57,30,
    199,153,57,145,49,39,121,60,207,204,169,140,57,205,227,12,
    51,103,51,230,28,143,243,204,188,144,49,47,242,120,137,153,
    11,25,243,50,143,87,152,185,152,49,107,60,94,101,230,181,
    236,250,238,58,51,111,64,99,137,238,147,136,115,147,42,213,
    208,143,173,84,156,226,131,78,238,63,189,213,2,101,223,125,
    151,46,216,31,67,218,210,244,43,78,162,232,223,152,46,78,
    45,145,157,180,138,206,241,77,207,226,41,249,224,52,99,233,
    42,169,35,183,48,88,183,185,224,233,249,95,247,10,207,241,
    51,193,131,220,195,3,110,237,186,211,28,80,125,228,228,128,
    138,39,225,69,60,28,88,124,56,184,71,135,131,61,94,14,
    199,208,231,131,30,96,75,249,170,144,179,161,124,113,130,105,
    122,101,244,65,128,76,116,59,29,25,122,246,50,20,123,123,
    126,60,72,140,80,161,253,59,20,218,45,83,76,99,51,127,
    60,87,105,63,41,120,204,145,45,229,217,57,224,24,51,180,
    255,153,65,187,78,151,43,189,77,197,190,71,3,111,35,249,
    14,98,255,50,143,80,253,84,220,18,251,107,217,142,226,46,
    29,44,207,46,140,13,33,95,18,20,120,234,231,111,124,25,
    79,82,73,7,55,121,233,4,8,215,176,169,39,253,95,222,
    163,249,103,179,249,143,61,86,203,167,169,244,147,100,247,176,
    5,63,64,156,38,166,98,113,136,171,126,114,154,130,4,113,
    239,200,231,126,147,78,190,9,79,248,67,228,105,70,234,27,
    14,179,213,205,211,85,252,97,23,13,147,49,207,118,70,81,
    154,168,194,19,165,28,245,217,105,47,242,221,115,182,242,95,
    196,81,91,203,97,19,23,74,182,17,201,221,80,177,5,111,
    69,17,217,71,208,63,163,188,122,120,134,73,209,211,68,189,
    193,248,183,161,39,73,195,112,54,113,245,224,141,83,110,70,
    49,54,185,222,227,232,20,195,127,188,22,50,251,70,102,246,
    27,132,213,167,103,14,242,169,86,191,5,53,100,246,210,81,
    164,244,147,230,61,129,183,119,79,6,82,201,190,123,153,162,
    26,155,94,158,122,24,191,56,234,58,142,190,4,194,207,129,
    227,12,188,111,255,5,78,240,71,154,137,174,50,176,111,23,
    101,236,220,103,196,145,31,163,82,174,8,62,42,29,249,134,
    81,155,74,181,72,95,126,116,19,155,91,167,241,124,243,224,
    47,192,178,147,10,237,51,92,32,214,220,182,254,206,130,47,
    226,237,235,144,94,146,218,183,242,77,136,110,145,249,198,73,
    223,244,97,139,192,199,56,62,181,217,84,240,21,81,237,59,
    43,153,159,43,218,79,123,119,163,203,158,242,87,112,237,59,
    125,196,214,253,182,254,134,71,77,30,121,238,197,46,210,51,
    71,184,137,140,125,55,240,95,73,142,243,201,211,174,231,101,
    239,98,95,153,71,8,40,238,121,250,168,232,38,74,182,213,
    149,126,54,119,130,108,175,172,157,40,242,32,61,225,166,66,
    39,251,142,15,83,223,79,246,229,97,16,53,119,164,151,202,
    92,238,47,243,89,212,166,181,154,63,217,90,237,202,81,19,
    100,184,203,22,160,121,95,71,158,60,182,22,15,60,47,182,
    221,112,75,58,207,37,157,227,213,213,163,2,135,92,204,164,
    78,94,14,6,67,14,120,6,99,38,161,166,9,153,39,73,
    209,29,84,54,73,143,205,71,220,62,61,60,103,116,44,183,
    124,244,56,102,197,135,85,166,157,44,37,27,87,149,254,91,
    105,65,207,160,107,129,190,105,210,87,232,247,169,51,75,26,
    56,208,55,111,149,241,10,214,5,234,116,77,81,197,94,215,
    50,71,39,42,214,232,72,197,170,12,153,252,245,200,152,152,
    50,170,86,101,100,84,188,233,103,17,43,73,213,88,156,175,
    136,239,1,248,221,195,7,
};

EmbeddedPython embedded_m5_internal_param_L1Cache_Controller(
    "m5/internal/param_L1Cache_Controller.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_L1Cache_Controller.py",
    "m5.internal.param_L1Cache_Controller",
    data_m5_internal_param_L1Cache_Controller,
    2647,
    8945);

} // anonymous namespace