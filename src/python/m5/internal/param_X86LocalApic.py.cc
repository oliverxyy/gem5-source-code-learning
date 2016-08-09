#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_X86LocalApic[] = {
    120,156,197,88,109,115,219,198,17,222,3,64,74,164,36,235,
    221,175,114,68,219,149,195,122,26,177,77,99,59,51,113,221,
    58,78,58,227,76,34,167,96,90,59,76,166,40,4,156,40,
    80,32,192,1,78,178,153,145,102,58,149,39,237,135,246,99,
    127,66,62,244,223,244,23,244,175,180,187,123,0,8,73,212,
    196,147,214,172,69,158,143,119,123,123,251,242,236,222,222,121,
    144,253,171,224,247,87,13,128,244,95,2,192,199,143,128,16,
    160,47,160,35,64,72,1,254,10,236,85,32,121,15,252,10,
    188,2,232,24,32,13,56,198,142,9,95,25,16,205,242,154,
    42,132,38,143,8,24,214,65,90,208,169,192,179,104,17,44,
    89,133,189,58,36,127,0,33,68,36,224,185,63,5,254,52,
    188,66,238,216,169,49,195,105,160,193,58,15,214,192,159,225,
    193,58,248,179,220,153,129,225,2,200,89,232,204,17,89,231,
    2,178,189,131,108,231,153,237,63,137,173,143,51,171,224,95,
    32,114,148,235,75,162,180,136,146,247,155,103,46,11,185,148,
    139,208,89,202,251,203,165,254,74,169,191,90,234,95,44,245,
    47,149,250,151,75,253,43,165,254,213,82,255,90,169,191,86,
    234,95,47,245,223,226,62,106,184,4,189,117,232,53,160,119,
    3,118,208,232,139,133,54,55,65,154,208,187,5,157,91,32,
    241,115,19,142,209,47,254,82,105,197,143,120,197,114,177,98,
    131,87,220,134,206,109,144,248,217,208,43,170,208,110,94,68,
    95,7,255,198,127,77,244,53,168,89,108,14,100,146,6,113,
    228,4,209,78,28,24,52,95,165,134,144,225,81,51,149,65,
    228,49,65,228,31,192,248,240,141,12,34,71,128,140,5,233,
    18,26,112,196,157,35,3,134,77,56,20,208,179,192,55,225,
    16,183,169,144,0,93,1,199,6,124,109,18,193,17,182,22,
    58,242,45,176,148,198,71,143,29,169,57,77,193,81,5,14,
    43,208,126,126,104,208,192,94,13,146,239,224,155,53,102,58,
    205,76,13,56,196,214,130,99,11,142,170,240,12,137,112,168,
    87,35,245,197,243,67,212,20,71,218,77,11,165,221,42,169,
    75,170,248,65,18,185,125,169,150,177,239,12,220,196,237,59,
    207,223,191,247,105,236,185,225,163,65,224,53,235,57,97,156,
    110,14,92,181,107,243,74,147,76,210,31,40,230,24,71,82,
    205,96,103,39,136,124,167,31,251,251,161,84,211,196,206,217,
    9,66,233,56,60,249,164,63,136,19,245,113,146,196,137,77,
    86,229,193,48,118,139,21,100,83,47,140,83,217,164,221,120,
    27,155,216,43,162,222,25,48,71,18,128,165,165,197,190,76,
    189,36,24,40,116,150,230,72,212,196,173,73,110,226,38,253,
    29,54,173,221,184,47,91,113,24,160,103,95,14,135,173,65,
    18,119,81,205,86,87,246,239,190,147,42,119,59,148,173,237,
    253,32,244,91,168,120,107,48,84,187,113,212,234,223,109,5,
    145,146,104,155,176,117,214,42,155,72,181,68,252,95,4,93,
    39,96,205,156,93,25,14,100,50,71,163,87,105,111,177,32,
    102,69,85,152,162,41,230,176,87,193,175,41,214,140,25,177,
    21,144,110,30,233,75,200,178,202,88,34,7,11,216,51,32,
    89,35,164,244,240,35,200,181,136,151,54,205,25,60,247,27,
    50,138,30,237,153,228,127,61,120,200,232,66,152,33,229,3,
    114,120,4,12,145,10,244,170,160,161,131,136,211,88,74,134,
    212,34,57,177,49,144,185,5,233,223,1,141,140,160,57,132,
    12,80,199,38,136,104,1,84,157,242,7,142,94,196,13,255,
    196,152,108,55,73,252,45,6,134,218,13,210,248,69,196,230,
    167,62,71,81,27,45,243,249,240,233,118,79,122,42,93,199,
    129,47,227,253,134,231,70,81,172,26,174,239,55,92,165,146,
    96,123,95,201,180,161,226,198,70,218,36,143,218,139,57,182,
    10,126,195,65,142,37,242,59,98,73,255,240,3,79,225,15,
    6,173,195,94,72,165,66,92,236,198,126,138,227,196,162,43,
    149,77,66,42,50,114,204,130,48,108,28,34,165,237,145,238,
    2,254,126,148,75,194,216,108,86,115,36,165,50,220,81,117,
    6,165,155,166,14,75,66,227,140,63,98,124,224,134,251,146,
    185,35,136,20,10,68,93,45,195,100,16,120,153,180,201,149,
    103,141,162,56,242,135,40,96,224,189,77,123,95,102,28,206,
    50,18,87,17,133,83,216,86,241,255,170,184,104,120,86,134,
    189,106,142,63,202,133,10,216,251,34,3,0,98,241,24,243,
    78,211,224,196,193,74,113,92,222,164,30,45,182,215,168,185,
    78,205,91,212,172,231,122,191,113,229,231,78,43,127,159,54,
    52,88,99,214,141,92,100,230,186,249,39,98,235,202,40,182,
    48,65,182,41,70,12,138,164,81,140,88,148,76,147,135,212,
    34,41,71,159,9,233,23,148,186,41,150,152,25,133,13,6,
    0,245,70,97,193,150,178,23,200,2,211,57,162,109,130,105,
    25,171,221,18,86,109,114,18,3,213,190,146,167,69,135,40,
    52,68,237,107,196,170,50,198,212,13,106,110,76,204,222,35,
    176,117,207,128,237,3,218,123,33,3,219,28,131,172,142,223,
    5,195,51,51,39,20,7,230,242,41,144,17,194,172,49,8,
    187,77,61,243,172,218,147,6,87,166,236,175,75,224,34,249,
    140,178,78,91,216,25,94,34,85,202,176,186,132,199,255,179,
    232,18,158,232,6,159,232,63,229,19,157,171,2,174,195,116,
    178,54,57,95,235,78,133,108,178,99,194,197,236,164,78,107,
    216,162,70,47,135,141,120,167,161,88,105,202,173,15,54,210,
    205,141,244,3,204,154,141,135,156,175,116,222,212,153,49,145,
    3,202,108,180,244,227,151,158,228,227,145,127,57,142,78,100,
    14,39,53,39,59,118,17,97,171,100,81,35,55,53,167,244,
    84,37,148,201,39,99,236,122,97,108,146,253,19,218,173,206,
    150,54,197,37,68,83,93,176,72,142,78,225,92,130,241,44,
    126,63,36,235,147,218,18,168,56,183,219,90,96,214,133,180,
    178,127,114,2,49,111,90,19,187,133,172,127,155,35,165,58,
    66,10,125,205,28,253,127,6,174,78,5,124,11,132,5,116,
    121,134,254,34,88,200,249,203,68,254,123,224,48,25,83,17,
    112,206,105,83,21,192,20,152,138,210,251,76,170,11,132,79,
    224,47,165,24,203,143,113,51,171,63,203,199,184,85,228,43,
    6,209,107,29,213,214,201,196,70,222,217,117,83,34,211,217,
    106,20,182,163,51,161,168,26,49,91,191,113,68,77,235,125,
    28,18,233,235,17,158,232,32,188,38,150,141,18,74,126,70,
    205,187,5,64,68,62,246,38,165,91,135,243,143,109,71,159,
    11,95,145,8,22,11,61,63,197,169,23,153,60,105,63,114,
    30,63,253,244,233,86,219,121,66,59,36,251,3,149,22,241,
    80,201,227,225,221,34,30,36,31,103,175,248,134,66,173,65,
    254,63,54,4,94,75,177,158,163,91,160,5,178,2,157,42,
    69,14,151,223,34,11,44,145,167,54,74,132,39,206,74,54,
    209,150,54,94,1,1,237,93,106,94,78,38,101,144,131,31,
    132,110,127,219,119,31,82,94,76,105,67,47,15,53,35,151,
    126,161,44,61,133,137,56,79,1,254,121,55,215,226,96,50,
    233,226,30,240,89,168,165,231,224,240,99,143,115,196,23,187,
    178,209,151,253,109,188,138,238,6,131,198,78,232,118,217,55,
    102,166,221,211,92,59,197,206,61,93,127,164,119,168,141,27,
    94,28,97,38,223,247,84,156,52,124,137,215,51,233,55,222,
    105,240,49,208,8,210,134,187,141,179,174,167,52,236,79,134,
    46,151,186,110,210,77,185,170,221,123,65,221,201,249,214,193,
    219,119,128,5,254,1,20,199,174,190,21,22,89,157,75,119,
    29,69,184,47,94,188,212,80,103,49,170,69,236,77,106,126,
    12,19,77,254,239,101,158,76,201,96,85,113,205,168,25,124,
    71,44,211,125,78,43,199,196,236,95,95,39,102,245,35,82,
    22,185,85,162,148,83,116,255,167,182,70,71,64,167,158,15,
    206,112,59,203,131,115,249,224,5,110,231,121,112,33,127,188,
    90,228,193,37,232,44,211,235,11,141,172,80,54,152,250,111,
    179,1,7,211,228,194,232,232,127,154,4,236,251,255,31,225,
    237,247,33,43,18,206,75,0,162,172,217,156,78,0,61,145,
    223,81,202,106,241,139,200,149,177,248,115,188,68,186,74,106,
    63,173,77,74,85,78,36,122,231,63,142,194,250,108,53,253,
    168,208,234,152,11,164,225,10,187,79,95,208,216,125,226,89,
    116,21,203,106,139,203,234,7,84,86,31,178,9,28,67,87,
    214,35,96,86,10,75,208,229,54,146,47,156,179,214,208,197,
    51,9,231,14,6,50,242,237,59,80,174,135,121,122,50,88,
    160,212,245,45,148,202,21,83,172,96,1,124,54,14,41,55,
    151,180,100,63,86,138,200,155,152,71,25,188,127,203,193,219,
    228,220,92,36,104,251,1,53,156,146,139,108,108,255,178,240,
    199,205,241,200,196,13,157,16,17,18,121,67,186,122,189,14,
    25,22,81,124,105,46,141,169,123,227,151,241,59,29,209,165,
    161,123,32,29,60,30,35,188,82,208,99,175,23,239,227,48,
    237,248,3,151,146,20,84,216,125,15,157,186,255,61,236,251,
    110,138,38,31,47,218,15,93,75,178,53,202,178,157,67,200,
    0,228,156,225,203,80,42,57,38,88,20,57,52,123,197,240,
    37,150,15,241,16,47,148,124,63,195,223,161,227,76,240,192,
    253,133,78,236,144,210,41,139,7,174,168,226,145,187,42,240,
    207,168,85,107,130,107,153,83,15,248,90,48,122,27,211,119,
    145,97,106,115,222,157,47,112,201,175,204,121,65,65,16,230,
    187,243,150,219,215,207,131,252,242,101,223,130,236,133,194,126,
    187,192,55,153,141,47,128,250,178,141,185,134,235,44,46,171,
    236,159,211,56,185,160,127,119,51,215,106,83,107,245,161,155,
    162,113,131,248,35,121,16,120,146,31,187,251,119,213,181,177,
    180,35,178,203,99,231,219,67,244,107,255,204,98,25,237,247,
    157,207,100,63,78,134,159,197,190,84,107,167,230,31,249,126,
    98,187,81,87,58,7,146,106,69,117,227,52,65,86,40,106,
    30,57,213,120,125,78,210,158,163,8,78,234,167,97,142,153,
    179,243,143,195,216,219,147,126,70,115,253,124,154,143,226,190,
    139,227,227,119,105,7,249,46,139,167,230,253,132,86,173,158,
    26,77,101,18,184,97,240,141,126,113,206,135,213,10,65,98,
    156,175,168,4,61,57,196,69,160,218,128,243,239,107,28,58,
    137,236,6,20,129,204,250,36,135,236,76,34,108,179,109,198,
    196,123,153,195,228,194,77,223,185,244,163,209,67,202,183,28,
    58,244,178,92,155,175,97,232,209,105,101,138,58,158,87,150,
    57,187,80,179,102,103,106,86,109,202,228,167,192,57,188,116,
    215,173,218,204,172,56,239,111,29,3,182,110,172,47,213,196,
    127,0,235,100,223,99,
};

EmbeddedPython embedded_m5_internal_param_X86LocalApic(
    "m5/internal/param_X86LocalApic.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_X86LocalApic.py",
    "m5.internal.param_X86LocalApic",
    data_m5_internal_param_X86LocalApic,
    2438,
    7559);

} // anonymous namespace
