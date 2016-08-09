#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_X86NativeTrace[] = {
    120,156,197,88,91,115,219,198,21,62,11,128,148,72,73,150,
    100,221,108,75,177,144,116,220,176,158,70,108,211,40,206,140,
    85,183,105,234,206,36,211,200,41,232,142,21,37,83,20,2,
    86,20,40,16,224,0,43,89,204,72,47,165,167,237,244,189,
    63,161,15,253,55,253,71,205,57,103,1,16,186,117,50,211,
    134,149,137,245,98,177,123,246,92,190,115,217,245,33,255,171,
    225,243,75,27,32,251,155,0,8,240,39,32,2,232,11,216,
    23,32,164,128,96,25,142,107,144,126,0,65,13,222,0,236,
    27,32,13,24,97,199,132,175,12,136,103,121,77,29,34,147,
    71,4,12,155,32,45,216,175,193,171,120,17,44,89,135,227,
    38,164,127,4,33,68,44,96,47,152,130,96,26,222,32,117,
    236,52,152,224,52,208,96,147,7,27,16,204,240,96,19,130,
    89,238,204,192,112,1,228,44,236,207,209,180,253,59,72,246,
    49,146,157,103,178,255,34,178,1,126,89,129,224,14,77,71,
    190,190,164,153,22,205,228,253,230,153,202,66,193,229,34,236,
    223,45,250,75,149,254,114,165,191,82,233,175,86,250,107,220,
    71,110,238,66,239,30,244,238,67,239,1,28,162,130,22,203,
    157,215,65,154,208,219,128,253,13,144,248,91,135,17,234,48,
    184,91,89,241,22,175,88,42,87,60,228,21,155,176,191,9,
    18,127,15,245,138,58,116,90,171,104,151,240,223,248,215,66,
    187,128,154,197,230,84,166,89,152,196,110,24,31,38,161,65,
    223,235,212,144,21,125,106,166,114,115,126,66,230,252,39,176,
    45,3,35,55,231,5,32,97,65,178,68,6,92,112,231,194,
    128,97,11,206,5,244,44,8,76,56,199,109,106,196,64,87,
    192,200,128,175,77,154,112,129,173,133,74,127,8,150,210,182,
    236,177,210,53,165,41,184,168,193,121,13,58,123,231,6,13,
    28,55,32,253,7,124,179,193,68,167,153,168,1,231,216,90,
    48,178,224,162,14,175,112,18,14,245,26,36,190,216,59,71,
    73,113,164,211,178,144,219,221,138,184,36,74,16,166,177,215,
    151,106,5,251,238,192,75,189,190,187,247,209,135,187,158,10,
    79,229,203,212,243,101,171,89,76,77,178,173,129,167,142,28,
    94,107,146,82,250,3,197,52,147,88,170,25,236,28,134,113,
    224,246,147,224,36,146,106,154,8,186,135,97,36,93,151,63,
    126,218,31,36,169,122,158,166,73,234,144,94,121,48,74,188,
    114,5,105,213,143,146,76,182,104,55,222,198,33,242,138,102,
    31,14,152,34,49,192,252,210,226,64,102,126,26,14,20,154,
    75,83,164,217,68,173,69,134,226,38,219,195,166,125,148,244,
    101,59,137,80,164,244,108,56,108,15,210,164,139,130,182,187,
    178,191,253,94,166,188,131,72,182,15,78,194,40,104,163,232,
    237,193,80,29,37,113,187,191,221,14,99,37,81,59,81,251,
    38,189,108,225,188,187,180,195,235,176,235,134,44,155,123,36,
    163,129,76,231,104,244,1,237,46,22,196,172,168,11,83,180,
    196,28,246,106,248,152,98,195,152,17,187,33,73,231,147,196,
    132,46,171,138,39,50,178,128,99,3,210,13,66,75,15,127,
    130,204,139,152,233,208,55,131,191,253,142,212,162,71,123,38,
    97,64,15,158,51,194,16,106,56,115,135,140,30,3,195,164,
    6,189,58,104,248,32,234,52,158,210,33,181,56,157,200,24,
    72,220,130,236,239,128,106,70,224,156,67,14,170,145,9,34,
    94,0,213,36,127,199,209,85,220,240,79,140,203,78,139,216,
    223,101,104,168,163,48,75,94,199,108,0,234,179,39,117,80,
    51,95,12,95,28,244,164,175,178,77,28,248,50,57,177,125,
    47,142,19,101,123,65,96,123,74,165,225,193,137,146,153,173,
    18,251,81,214,34,155,58,139,5,186,74,122,195,65,129,38,
    178,60,162,73,191,4,161,175,240,101,137,95,216,10,153,84,
    136,140,163,36,200,112,156,72,116,165,114,136,73,69,74,78,
    152,17,6,142,75,83,105,123,156,119,7,223,63,46,56,97,
    116,182,234,5,150,50,25,29,170,38,195,210,203,50,151,57,
    161,113,70,32,17,62,245,162,19,201,212,17,70,10,25,162,
    174,230,97,82,24,188,71,242,20,226,179,76,113,18,7,67,
    100,49,244,223,165,221,239,49,18,103,25,139,43,136,195,41,
    108,235,248,127,93,172,26,190,149,163,175,94,32,144,34,162,
    2,182,191,200,33,128,104,28,97,244,105,25,28,62,88,44,
    246,205,119,168,71,139,157,13,106,222,162,230,33,53,155,133,
    228,19,16,127,238,170,248,79,104,75,131,101,102,233,200,76,
    102,33,93,112,201,191,238,143,253,11,3,101,135,252,196,32,
    111,26,251,137,69,65,53,125,70,45,78,101,15,52,33,123,
    73,33,156,252,137,137,145,235,160,19,80,111,236,26,172,43,
    103,129,116,48,93,160,218,33,168,86,241,218,173,224,213,33,
    51,49,88,157,251,69,112,116,105,134,134,169,179,78,164,106,
    55,40,219,166,230,237,9,106,124,12,184,238,53,192,61,165,
    221,23,114,192,205,49,208,154,248,44,24,190,153,155,161,76,
    157,75,87,128,70,40,179,110,64,217,15,169,103,94,23,124,
    242,0,203,197,253,77,5,96,196,161,81,149,106,23,59,195,
    53,18,166,10,173,53,44,5,94,197,107,152,221,13,206,238,
    63,225,236,206,21,2,215,79,58,104,155,28,183,117,167,70,
    90,57,52,97,53,207,218,89,3,91,148,233,108,104,39,135,
    182,98,177,41,198,238,60,202,182,30,101,79,49,122,218,207,
    56,110,233,248,169,35,100,42,7,20,225,104,233,243,51,95,
    114,162,228,55,215,213,1,205,229,224,230,230,9,24,81,70,
    85,0,91,128,149,205,161,61,83,41,69,244,73,169,187,89,
    170,155,184,255,140,246,107,178,174,77,177,134,136,106,10,102,
    202,213,193,156,11,50,254,138,207,175,72,255,36,184,4,42,
    171,157,142,102,153,165,33,185,156,31,95,66,205,247,47,139,
    211,70,226,191,47,208,82,31,163,133,30,179,240,129,191,0,
    87,171,2,254,12,132,7,52,123,238,3,165,203,16,0,150,
    104,250,31,128,157,229,134,234,128,99,79,135,42,2,158,129,
    33,41,123,194,83,117,177,240,25,252,181,226,105,69,74,55,
    243,122,180,154,210,173,50,110,49,144,190,83,218,182,46,7,
    56,178,207,145,151,209,52,29,181,198,206,59,206,14,101,13,
    137,81,123,2,168,154,214,59,185,196,212,215,99,76,81,82,
    92,23,75,70,5,41,63,165,230,253,18,36,162,24,251,126,
    249,219,132,219,147,184,171,115,196,87,196,132,197,108,207,79,
    41,82,44,175,118,63,121,241,219,23,187,157,43,52,75,199,
    168,21,142,241,126,233,24,146,179,219,27,62,184,80,107,16,
    12,70,134,192,147,37,150,120,116,144,179,64,214,96,191,78,
    46,196,53,185,200,61,76,20,81,142,98,226,165,212,201,122,
    218,213,26,44,145,160,141,76,205,217,164,162,7,217,121,39,
    242,250,7,129,247,172,79,187,209,150,126,225,115,70,193,255,
    66,149,127,242,23,113,155,8,252,186,93,200,113,58,169,200,
    241,33,18,47,249,103,63,9,18,159,195,197,203,35,105,247,
    101,255,0,79,169,71,225,192,62,140,188,46,219,199,204,229,
    123,81,200,167,216,192,87,75,146,236,49,181,137,237,39,49,
    6,246,19,95,37,169,29,72,60,183,201,192,126,207,230,172,
    96,135,153,237,29,224,87,207,87,26,255,151,189,152,43,96,
    47,237,102,92,236,30,191,166,238,36,237,235,226,209,60,196,
    202,63,129,50,15,235,3,99,25,228,185,166,215,238,132,59,
    227,137,76,13,117,72,163,242,196,217,162,230,71,48,225,92,
    240,1,18,143,104,23,82,90,93,172,27,13,67,45,99,247,
    242,204,47,104,117,118,221,123,63,255,46,222,171,111,132,114,
    31,174,23,87,73,83,32,249,68,71,183,61,245,252,182,7,
    29,123,234,191,117,108,246,138,73,250,195,233,255,212,159,157,
    39,255,47,246,157,143,32,79,253,183,249,178,168,202,54,167,
    125,185,39,138,19,72,85,48,190,245,88,191,5,70,174,159,
    74,79,73,109,173,141,201,137,203,113,65,239,61,28,251,232,
    245,90,249,227,82,178,17,151,62,195,101,54,162,62,130,177,
    17,197,171,248,1,22,205,22,23,205,59,84,52,159,179,26,
    92,67,215,205,99,128,214,74,109,208,241,36,150,175,221,155,
    52,162,139,99,98,207,27,12,100,28,56,143,161,90,239,242,
    231,73,97,226,169,198,236,184,20,49,197,50,22,184,215,125,
    146,194,109,69,82,182,102,173,244,194,9,218,149,97,60,42,
    96,220,162,227,199,56,230,58,59,212,112,148,45,3,172,243,
    11,40,130,110,9,212,64,70,82,201,27,173,163,104,117,126,
    52,14,36,38,160,100,136,39,20,46,247,241,61,114,221,137,
    6,236,159,35,241,19,200,143,89,24,176,69,221,104,152,141,
    122,67,112,38,188,114,51,172,153,178,161,40,106,135,153,195,
    174,62,95,202,206,151,151,69,50,34,93,241,65,108,215,235,
    235,59,39,190,76,113,126,0,249,129,215,121,183,84,36,221,
    6,240,73,66,159,220,16,218,156,165,57,41,59,63,163,113,
    58,18,247,183,183,10,137,182,180,68,21,113,248,2,181,191,
    205,250,191,62,241,249,153,158,149,170,141,27,191,127,138,149,
    66,62,225,102,2,157,176,175,47,249,212,226,149,239,65,234,
    97,127,229,202,104,38,211,208,139,194,111,244,173,94,49,204,
    139,175,179,78,238,124,169,208,173,124,228,164,202,199,135,255,
    80,10,51,158,82,217,13,51,220,135,55,185,30,22,200,214,
    234,237,219,194,104,149,192,36,33,168,171,89,125,54,127,70,
    247,65,217,175,177,161,107,188,198,124,3,225,72,33,195,196,
    19,241,156,176,204,217,133,134,53,59,211,176,26,83,38,223,
    185,204,225,169,166,105,53,102,102,197,248,223,38,130,183,105,
    108,226,218,111,1,150,188,250,104,
};

EmbeddedPython embedded_m5_internal_param_X86NativeTrace(
    "m5/internal/param_X86NativeTrace.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_X86NativeTrace.py",
    "m5.internal.param_X86NativeTrace",
    data_m5_internal_param_X86NativeTrace,
    2217,
    6808);

} // anonymous namespace
