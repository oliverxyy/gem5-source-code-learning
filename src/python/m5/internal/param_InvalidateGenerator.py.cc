#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_InvalidateGenerator[] = {
    120,156,197,88,255,111,219,198,21,127,71,82,178,37,219,177,
    28,199,118,18,59,49,147,214,173,22,172,214,218,213,77,129,
    122,217,178,182,43,90,160,110,70,117,72,234,118,227,104,241,
    44,83,150,72,129,60,39,81,38,99,192,28,108,195,126,223,
    159,176,31,246,223,12,251,135,182,247,222,145,20,109,201,65,
    209,13,138,35,93,78,247,229,221,251,242,121,95,238,90,144,
    254,149,240,251,11,27,32,249,183,0,240,241,35,160,11,208,
    19,176,47,64,72,1,254,53,56,46,65,252,62,248,37,120,
    9,176,111,128,52,224,12,59,38,124,107,64,56,207,123,202,
    208,53,121,68,192,160,10,210,130,253,18,60,14,151,192,146,
    101,56,174,66,252,123,16,66,132,2,158,248,51,224,207,194,
    75,164,142,157,10,19,156,5,26,172,242,96,5,252,57,30,
    172,130,63,207,157,57,24,212,64,206,195,254,2,45,219,191,
    130,100,239,33,217,69,38,251,47,34,235,227,204,10,248,87,
    104,57,242,245,13,173,180,104,37,159,183,200,84,106,25,151,
    75,176,127,53,235,47,23,250,215,10,253,149,66,127,181,208,
    95,43,244,175,23,250,55,10,253,155,133,254,122,161,191,81,
    232,223,226,62,74,117,21,58,183,161,179,9,29,27,14,81,
    209,75,185,4,119,64,154,208,185,11,251,119,65,226,231,14,
    156,161,45,252,171,133,29,111,240,142,229,124,199,155,188,99,
    11,246,183,64,226,231,77,189,163,12,205,250,42,218,55,248,
    15,254,213,209,190,160,230,177,121,42,227,36,136,66,55,8,
    15,163,192,160,249,50,53,132,134,22,53,51,41,44,62,38,
    88,252,19,24,19,190,145,194,226,20,144,176,32,89,186,6,
    156,114,231,212,128,65,29,134,2,58,22,248,38,12,241,152,
    18,49,208,22,112,102,192,119,38,45,56,197,214,66,227,221,
    6,75,105,76,116,216,120,154,210,12,156,150,96,88,130,230,
    147,161,65,3,199,21,136,255,1,47,54,152,232,44,19,53,
    96,136,173,5,103,22,156,150,225,49,46,194,161,78,133,196,
    23,79,134,40,41,142,52,235,22,114,187,87,16,151,68,241,
    131,56,244,122,82,221,196,190,219,247,98,175,231,126,30,62,
    245,186,129,239,41,249,153,12,101,236,169,40,174,87,179,245,
    81,178,221,247,212,145,195,4,76,210,76,175,175,152,112,20,
    74,53,135,157,195,32,244,221,94,228,159,116,165,154,37,170,
    238,97,208,149,174,203,147,159,247,250,81,172,62,141,227,40,
    118,72,185,60,216,141,188,124,7,169,182,213,141,18,89,167,
    211,248,24,135,200,43,90,125,216,103,138,196,0,51,77,155,
    125,153,180,226,160,175,208,102,154,34,173,38,106,117,178,22,
    55,201,111,177,105,28,69,61,217,136,186,1,26,248,249,96,
    208,232,199,81,27,165,109,180,101,111,231,157,68,121,7,93,
    217,56,56,9,186,126,227,201,135,31,52,250,3,117,20,133,
    141,222,78,35,8,149,68,21,117,27,151,42,103,27,23,95,
    165,99,158,5,109,55,96,1,221,35,217,237,203,120,129,70,
    73,179,32,106,98,94,148,133,41,234,98,1,123,37,252,154,
    98,195,152,19,123,1,137,216,34,177,9,103,86,17,89,100,
    110,1,199,6,196,27,132,155,14,126,4,25,26,209,211,164,
    57,131,231,126,77,186,209,163,29,147,208,160,7,135,140,53,
    4,29,174,220,37,243,135,192,128,41,65,167,12,26,72,136,
    63,141,172,120,64,45,46,39,50,6,18,183,32,249,59,160,
    174,17,66,67,72,225,117,102,130,8,107,160,170,20,65,112,
    116,21,15,252,19,35,180,89,39,246,247,24,31,234,40,72,
    162,103,33,91,129,250,236,83,77,212,204,163,193,87,7,29,
    217,82,201,38,14,124,19,157,216,45,47,12,35,101,123,190,
    111,123,74,197,193,193,137,146,137,173,34,123,43,169,147,97,
    157,165,12,98,57,189,65,63,131,20,153,31,33,165,127,248,
    65,75,225,143,101,254,193,86,72,164,66,120,28,69,126,130,
    227,68,162,45,149,67,76,42,82,114,196,140,48,122,92,90,
    74,199,227,186,43,248,251,97,198,9,67,180,94,206,0,149,
    200,238,161,170,50,54,189,36,113,153,19,26,103,24,18,97,
    68,197,137,100,234,136,37,133,12,81,87,243,48,85,32,94,
    39,161,50,29,176,96,97,20,250,3,228,51,104,189,77,44,
    92,103,56,206,51,32,87,16,140,51,216,150,241,255,178,88,
    53,90,86,10,193,114,6,67,10,144,10,24,4,34,197,1,
    66,242,12,131,81,221,224,104,194,178,177,151,222,165,30,109,
    118,54,168,185,69,205,109,106,54,51,241,167,165,131,133,139,
    58,184,79,231,26,44,56,139,72,6,51,51,17,253,115,158,
    118,99,228,105,24,60,155,228,49,6,249,213,200,99,44,10,
    180,241,3,106,113,41,251,162,9,201,215,20,214,201,179,152,
    24,57,17,186,3,245,70,78,194,10,115,106,164,136,217,12,
    223,14,129,182,136,220,118,1,185,14,217,138,97,235,220,200,
    98,165,75,43,52,96,157,117,34,85,154,160,113,155,154,59,
    211,86,251,8,122,237,49,232,125,68,44,212,82,232,45,48,
    228,170,248,173,25,45,51,181,69,158,83,151,47,64,142,240,
    102,77,192,219,91,212,51,199,165,127,77,80,75,101,254,85,
    1,106,196,166,81,20,109,15,59,131,53,146,168,8,178,53,
    44,20,30,135,107,152,251,13,206,253,63,225,220,207,245,3,
    87,105,58,144,155,28,203,117,167,68,170,57,52,97,53,205,
    233,73,5,91,20,236,249,192,142,14,109,197,178,83,220,221,
    221,74,182,183,146,143,48,162,218,15,56,150,233,152,170,163,
    102,44,251,20,245,104,235,167,207,91,146,51,40,255,114,93,
    29,228,92,14,120,110,154,153,17,111,43,164,88,35,211,56,
    135,251,68,197,20,229,167,170,243,106,174,115,18,225,11,58,
    180,202,10,55,197,26,98,171,42,152,51,87,71,121,174,217,
    120,22,191,191,36,35,144,244,18,168,130,119,154,154,111,22,
    137,132,115,126,124,14,63,83,18,200,105,224,9,191,201,112,
    83,30,225,134,190,102,230,18,127,1,174,106,5,252,25,8,
    25,8,128,212,37,114,15,34,40,44,211,242,223,1,251,206,
    132,218,129,227,81,147,234,5,94,129,97,42,185,207,75,117,
    41,241,5,252,181,224,120,89,194,55,211,186,181,152,240,173,
    60,150,49,164,190,87,82,183,206,7,61,50,210,145,151,208,
    50,29,201,70,190,60,74,27,121,153,137,145,124,90,248,154,
    213,199,185,196,217,119,35,116,81,202,92,23,203,70,1,51,
    239,82,243,94,14,23,145,141,77,129,201,77,184,60,207,187,
    58,131,124,75,156,88,204,251,226,12,231,152,9,180,114,255,
    40,101,254,241,94,238,31,146,115,222,75,190,226,80,107,16,
    16,206,12,129,119,89,44,1,233,234,104,129,44,193,126,153,
    60,137,11,119,145,58,154,200,34,30,197,199,115,9,149,149,
    180,167,213,151,99,65,155,153,154,231,83,141,36,100,233,221,
    174,215,59,240,189,7,9,29,73,231,182,50,215,51,50,33,
    106,69,33,200,109,196,101,114,240,207,157,76,152,167,83,141,
    34,31,164,2,176,16,236,51,126,212,226,208,241,245,145,180,
    123,178,119,128,55,219,163,160,111,31,118,189,54,91,202,76,
    133,252,42,19,82,177,169,47,150,44,201,61,106,35,187,21,
    133,24,238,79,90,120,158,237,75,188,230,73,223,126,199,230,
    92,97,7,137,237,29,224,172,215,82,218,13,206,123,52,215,
    202,94,220,78,184,44,62,126,70,221,169,91,218,197,59,125,
    128,23,133,19,200,83,180,190,100,230,161,159,175,0,218,171,
    240,120,188,192,169,129,142,113,84,190,56,219,212,252,8,94,
    71,134,120,31,79,136,233,40,82,95,89,172,27,21,67,221,
    152,236,208,143,136,78,50,238,214,7,223,199,173,245,227,84,
    234,220,101,90,41,103,232,141,129,218,10,165,11,122,123,42,
    165,111,79,52,168,223,158,202,60,114,133,194,192,204,255,26,
    6,216,125,166,238,56,195,255,171,247,59,247,95,171,12,206,
    135,144,22,13,151,121,190,40,10,184,160,61,191,35,178,251,
    76,81,58,126,82,177,95,5,53,183,21,75,28,213,198,219,
    152,178,224,28,79,52,3,127,28,185,245,120,229,253,48,151,
    241,140,203,167,193,53,182,169,190,218,177,77,197,227,240,38,
    150,224,22,151,224,187,84,130,15,89,33,174,161,171,240,17,
    104,75,185,94,232,206,17,202,103,147,120,211,186,209,245,54,
    241,232,245,251,50,244,157,123,80,44,161,121,122,170,56,161,
    64,118,6,133,154,198,20,215,176,102,30,247,88,10,216,5,
    153,217,184,165,220,71,167,109,102,198,247,223,50,124,215,23,
    161,24,181,157,93,106,56,78,231,33,218,249,121,110,164,119,
    95,9,94,172,84,99,204,11,8,161,158,12,149,155,4,47,
    36,93,229,126,192,46,172,195,184,200,154,48,199,106,103,47,
    242,101,87,42,121,57,96,20,137,145,62,5,248,18,19,106,
    52,192,123,24,223,103,240,119,215,117,167,159,123,126,134,39,
    252,129,142,162,228,136,185,71,148,49,251,172,24,149,114,69,
    112,130,191,240,72,174,121,163,203,185,174,219,7,137,195,49,
    105,49,55,8,63,225,102,233,149,108,199,183,206,61,175,167,
    31,221,248,33,201,121,3,210,43,190,243,118,110,88,202,120,
    124,89,210,215,84,244,60,46,62,184,214,112,126,74,227,180,
    171,183,179,157,9,182,173,5,251,36,136,241,38,40,253,92,
    44,126,76,238,237,240,139,197,248,242,230,32,81,178,167,214,
    47,76,202,240,164,231,126,41,123,81,60,248,50,242,165,218,
    184,48,255,16,237,238,120,97,91,186,79,37,213,72,234,206,
    197,5,105,129,164,105,100,171,236,137,60,156,95,59,198,139,
    94,132,147,250,77,149,111,3,227,243,31,119,163,214,177,244,
    211,53,183,46,95,243,73,212,243,112,124,242,41,205,32,59,
    101,233,194,188,31,211,174,149,11,163,137,140,3,196,209,11,
    253,84,155,13,171,53,178,250,37,230,32,194,99,163,92,244,
    92,150,122,216,67,98,217,14,208,84,49,211,30,219,159,70,
    96,130,175,122,235,149,222,92,164,53,117,255,210,151,16,253,
    198,242,128,94,248,146,71,216,208,235,108,101,177,130,190,70,
    33,218,20,85,12,210,150,57,95,171,88,243,115,21,171,50,
    99,242,3,218,2,94,71,171,86,101,110,94,76,254,183,137,
    62,90,53,54,107,21,241,95,133,39,198,145,
};

EmbeddedPython embedded_m5_internal_param_InvalidateGenerator(
    "m5/internal/param_InvalidateGenerator.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_InvalidateGenerator.py",
    "m5.internal.param_InvalidateGenerator",
    data_m5_internal_param_InvalidateGenerator,
    2348,
    7378);

} // anonymous namespace
