#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_Prefetcher[] = {
    120,156,197,89,221,115,219,198,17,223,3,64,74,164,36,139,
    178,190,108,75,182,32,127,52,172,167,17,219,52,142,51,19,
    215,173,155,164,51,205,76,20,23,76,199,14,147,41,10,17,
    71,10,20,9,176,192,201,54,93,169,15,149,167,31,111,125,
    233,159,144,135,254,35,125,238,63,211,231,102,119,143,0,65,
    138,26,123,166,53,43,145,167,195,98,111,111,247,246,183,123,
    123,167,38,12,127,10,248,253,153,13,144,252,83,0,248,248,
    17,208,5,232,9,104,8,16,82,128,191,6,71,5,136,223,
    7,191,0,175,0,26,6,72,3,206,176,99,194,215,6,132,
    139,60,166,8,93,147,41,2,6,101,144,22,52,10,240,36,
    92,1,75,22,225,168,12,241,111,65,8,17,10,120,234,207,
    129,63,15,175,80,58,118,74,44,112,30,136,88,102,98,9,
    252,5,38,150,193,95,228,206,2,12,42,32,23,161,177,68,
    108,141,75,40,246,46,138,93,102,177,255,34,177,62,190,89,
    7,255,18,177,163,94,95,17,167,69,156,60,223,50,75,169,
    164,90,174,64,227,114,218,95,205,245,215,114,253,245,92,127,
    35,215,223,204,245,175,228,250,87,115,253,107,185,254,86,174,
    191,205,125,180,228,50,116,174,67,231,6,116,118,160,133,139,
    187,146,105,109,131,52,161,179,11,141,93,144,248,177,225,12,
    215,223,191,156,27,113,147,71,172,102,35,110,241,136,219,208,
    184,13,18,63,183,244,136,34,212,171,27,232,211,224,63,248,
    83,69,159,130,90,196,230,153,140,147,32,10,221,32,108,69,
    129,65,239,139,212,16,2,154,212,204,13,161,240,49,65,225,
    31,192,56,240,141,33,20,78,1,5,11,178,165,107,192,41,
    119,78,13,24,84,225,68,64,199,2,223,132,19,156,166,64,
    10,180,5,156,25,240,141,73,12,167,216,90,232,176,27,96,
    41,141,131,14,59,76,75,154,131,211,2,156,20,160,254,244,
    196,32,194,81,9,226,111,225,229,54,11,157,103,161,6,156,
    96,107,193,153,5,167,69,120,130,76,72,234,148,200,124,241,
    244,4,45,69,74,189,106,161,182,251,57,115,201,20,63,136,
    67,175,39,213,10,246,221,190,23,123,61,247,113,44,91,82,
    53,15,101,92,45,167,108,81,178,215,247,212,161,195,227,76,
    90,144,94,95,177,188,40,148,106,1,59,173,32,244,221,94,
    228,31,119,165,154,39,97,110,43,232,74,215,229,151,191,236,
    245,163,88,125,26,199,81,236,208,154,50,177,27,121,217,8,
    90,209,102,55,74,100,149,102,227,105,28,18,175,136,187,213,
    103,137,164,0,235,74,131,125,153,52,227,160,175,208,85,90,
    34,113,147,180,42,57,137,155,228,75,108,106,135,81,79,214,
    162,110,128,126,125,49,24,212,250,113,212,70,35,107,109,217,
    187,247,110,162,188,131,174,172,29,28,7,93,191,246,244,195,
    15,106,253,129,58,140,194,90,239,94,45,8,149,196,149,233,
    214,38,215,100,15,121,46,147,244,231,65,219,13,216,46,247,
    80,118,251,50,94,34,234,53,154,89,84,196,162,40,10,83,
    84,197,18,246,10,248,53,197,182,177,32,246,3,178,172,73,
    214,18,170,172,60,142,200,185,2,142,12,136,183,9,37,29,
    252,8,114,43,98,165,78,239,12,126,247,43,90,18,77,237,
    152,228,123,77,60,97,100,33,196,144,243,1,57,59,4,134,
    71,1,58,69,208,176,65,180,105,28,197,3,106,145,157,196,
    24,40,220,130,228,239,128,75,140,128,57,129,33,152,206,76,
    16,97,5,84,153,114,4,82,55,112,194,63,50,30,235,85,
    82,127,159,97,161,14,131,36,122,30,242,226,83,159,35,168,
    142,43,243,120,240,197,65,71,54,85,178,131,132,175,162,99,
    187,233,133,97,164,108,207,247,109,79,169,56,56,56,86,50,
    177,85,100,223,73,170,228,79,103,37,69,86,38,111,208,79,
    145,68,94,71,36,233,7,63,104,42,124,88,229,7,246,66,
    34,21,162,226,48,242,19,164,147,136,182,84,14,41,169,104,
    145,35,86,132,65,227,18,43,77,143,124,151,240,249,81,170,
    9,35,179,90,76,113,148,200,110,75,149,25,146,94,146,184,
    172,9,209,25,125,36,248,153,215,61,150,44,29,33,164,80,
    33,234,106,29,102,129,191,43,100,75,106,58,219,19,70,161,
    63,64,245,130,230,59,52,243,21,70,225,34,227,112,29,49,
    56,135,109,17,255,22,197,134,209,180,134,200,43,166,232,163,
    44,168,128,125,47,134,238,71,36,158,97,198,169,26,156,50,
    216,36,142,201,155,212,163,193,206,54,53,215,169,185,65,205,
    78,106,245,91,54,125,105,210,244,251,52,157,193,246,178,101,
    228,30,51,181,204,31,139,171,171,163,184,194,196,88,167,248,
    48,40,138,70,241,97,81,18,141,31,82,139,172,28,121,38,
    25,163,116,28,177,48,10,25,4,63,245,70,33,193,235,228,
    84,200,254,249,20,205,14,65,52,143,211,118,14,167,14,185,
    136,65,234,92,77,19,162,75,28,26,158,206,22,137,42,76,
    89,104,155,154,221,25,173,246,8,104,237,115,64,251,136,102,
    174,12,129,182,196,0,43,227,183,98,52,205,161,11,178,109,
    114,117,2,96,132,46,107,10,186,190,71,61,243,188,209,179,
    5,214,208,212,95,228,128,69,218,25,121,139,246,177,51,216,
    36,67,242,144,218,196,45,255,73,184,137,187,184,193,187,248,
    15,121,23,231,74,128,107,44,157,164,77,206,211,186,83,160,
    21,105,153,176,49,220,157,147,18,182,104,207,139,129,29,181,
    108,197,38,83,78,125,112,39,217,187,147,124,132,217,210,126,
    200,121,74,231,75,157,17,99,217,167,140,70,67,63,125,209,
    148,188,41,242,147,235,234,4,230,114,50,115,135,155,45,162,
    107,157,214,211,72,23,154,83,121,162,98,202,224,179,88,234,
    114,182,212,164,249,103,52,87,153,215,217,20,155,136,164,178,
    96,133,92,157,184,185,232,226,183,248,253,57,173,61,25,45,
    129,202,110,167,174,213,101,75,200,38,231,7,99,104,121,187,
    118,56,53,20,252,235,20,37,197,17,74,232,107,166,184,255,
    51,112,53,42,224,79,64,56,64,119,15,113,159,133,9,57,
    126,149,216,127,3,28,32,83,170,0,206,53,117,218,249,153,
    3,83,80,114,159,89,117,81,240,25,252,37,23,93,233,214,
    109,14,235,205,252,214,109,101,121,138,1,244,70,219,179,53,
    158,208,200,55,135,94,66,108,58,75,141,2,118,180,19,100,
    117,34,102,233,183,140,166,121,61,139,75,10,125,51,194,18,
    109,126,91,98,213,200,33,228,71,212,188,151,129,67,164,180,
    183,167,219,14,92,188,81,187,122,47,248,154,20,176,88,229,
    229,57,14,141,145,136,12,251,133,20,251,239,101,216,151,188,
    105,189,226,243,7,181,6,121,251,204,16,120,184,196,138,141,
    206,114,22,200,2,52,138,20,37,92,94,139,97,16,137,52,
    137,81,202,27,219,17,121,73,246,245,98,101,14,215,190,164,
    230,197,44,146,3,185,243,65,215,235,29,248,222,195,152,102,
    162,233,154,105,88,25,169,238,149,188,238,20,18,226,34,245,
    249,241,94,106,195,179,89,36,134,15,80,112,166,59,135,129,
    31,53,57,27,124,121,40,237,158,236,29,224,33,243,48,232,
    219,173,174,215,102,191,152,67,219,190,72,109,83,236,216,201,
    10,35,185,75,109,100,55,163,16,243,245,113,83,69,177,237,
    75,60,122,73,223,126,215,230,100,111,7,137,237,29,224,91,
    175,169,52,196,199,131,148,11,89,47,110,39,92,179,30,61,
    167,238,172,252,234,226,169,58,160,226,29,178,173,85,159,247,
    178,220,205,101,185,142,24,156,21,15,85,106,160,179,21,85,
    27,206,30,53,223,135,25,166,248,247,81,240,239,104,6,90,
    172,162,216,50,74,134,170,140,197,232,99,26,149,156,143,212,
    127,191,73,164,234,11,160,97,188,22,137,83,206,209,153,158,
    218,18,165,249,70,57,37,46,112,187,200,196,165,148,120,137,
    219,101,38,86,82,226,10,183,151,153,184,154,18,215,184,93,
    103,226,70,74,220,228,246,10,19,175,166,196,107,220,110,49,
    113,59,37,94,231,246,6,19,119,210,27,46,155,137,187,208,
    184,73,87,55,68,185,69,201,102,238,191,77,54,28,173,179,
    138,211,223,255,79,115,140,115,255,255,161,186,243,33,12,171,
    141,139,242,139,200,219,181,164,243,75,71,164,135,156,188,81,
    124,153,178,57,5,226,110,51,150,158,146,218,67,219,179,49,
    147,115,148,158,247,15,163,156,113,190,28,127,148,89,116,198,
    85,214,96,141,29,167,79,119,236,56,241,36,188,134,117,185,
    197,117,249,3,170,203,79,216,124,215,208,165,249,8,144,133,
    108,21,214,176,9,229,115,119,114,37,116,237,77,170,121,253,
    190,12,125,231,46,228,203,105,126,61,11,12,80,78,252,43,
    228,42,30,83,172,97,253,124,62,246,40,229,231,44,100,15,
    22,178,104,155,145,47,25,178,127,75,33,91,189,13,249,188,
    239,60,160,134,51,125,150,228,157,159,102,158,216,153,142,199,
    8,207,54,125,175,45,233,200,246,90,30,172,192,244,157,78,
    70,82,183,166,13,193,10,237,152,118,169,86,208,69,99,88,
    244,155,240,145,120,218,188,198,201,234,246,212,161,199,61,23,
    87,49,86,199,125,183,223,74,120,142,55,98,164,73,150,105,
    146,113,186,178,47,30,140,193,211,211,51,188,150,137,164,47,
    100,210,153,166,110,78,27,212,111,185,125,50,153,121,88,246,
    27,176,145,116,202,60,99,84,190,97,56,55,48,25,104,141,
    47,124,73,178,248,116,49,72,212,238,52,38,44,127,130,208,
    237,5,73,34,181,168,215,115,145,76,58,106,231,137,211,215,
    108,18,30,175,101,74,23,54,71,227,224,229,60,235,203,174,
    84,242,92,146,225,50,99,120,113,228,163,38,113,52,192,194,
    137,15,198,248,220,117,221,153,213,64,63,65,193,47,105,6,
    74,20,88,3,137,34,86,65,235,98,236,215,40,21,75,130,
    139,204,137,255,153,104,5,239,80,36,215,169,225,237,106,57,
    11,108,190,215,79,11,61,202,1,124,111,177,239,245,244,149,
    44,223,55,58,20,126,124,55,228,188,147,37,8,186,52,227,
    3,184,190,232,192,52,205,213,47,23,187,206,143,137,78,224,
    233,221,219,75,77,219,211,166,213,17,49,178,199,255,86,232,
    221,83,91,19,60,146,160,255,185,236,69,241,224,243,200,151,
    106,123,226,253,35,223,143,29,47,196,108,242,76,82,241,205,
    176,26,99,24,86,222,90,70,202,101,79,85,101,156,247,156,
    46,154,9,95,234,155,116,206,111,231,223,127,220,141,154,71,
    210,31,242,92,191,152,231,147,168,135,184,190,96,150,122,144,
    206,178,50,241,222,167,104,80,235,19,212,68,198,129,215,13,
    94,234,11,250,148,204,169,105,82,32,185,39,123,226,122,122,
    162,186,96,136,199,178,29,36,148,47,151,243,236,195,237,150,
    0,200,166,157,207,5,185,161,179,138,7,125,66,213,87,105,
    15,41,170,249,50,144,110,218,75,203,37,140,13,218,134,77,
    81,198,141,216,50,23,43,37,107,113,161,100,149,230,76,190,
    30,93,18,171,70,217,42,45,44,138,105,191,59,24,67,101,
    99,103,179,36,190,3,9,183,48,161,
};

EmbeddedPython embedded_m5_internal_param_Prefetcher(
    "m5/internal/param_Prefetcher.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_Prefetcher.py",
    "m5.internal.param_Prefetcher",
    data_m5_internal_param_Prefetcher,
    2506,
    7797);

} // anonymous namespace
