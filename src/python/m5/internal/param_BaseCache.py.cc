#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BaseCache[] = {
    120,156,197,89,123,115,219,198,17,223,3,31,50,41,201,146,
    45,75,242,67,182,232,135,108,198,141,165,196,241,171,245,35,
    177,29,119,166,153,137,226,66,238,216,81,50,69,33,224,72,
    66,34,1,22,56,201,102,70,154,233,84,158,182,95,160,253,
    175,127,246,143,126,155,126,128,78,191,74,187,187,199,131,64,
    138,180,53,105,205,90,210,250,176,183,216,187,221,253,237,222,
    3,30,116,255,21,240,239,139,10,64,242,47,1,224,227,175,
    128,38,64,75,192,186,0,33,5,248,167,96,171,0,241,45,
    240,11,240,22,96,221,2,105,193,62,54,114,240,157,5,225,
    4,191,83,132,102,142,57,2,58,101,144,121,88,47,192,203,
    240,4,228,101,17,182,202,16,255,6,132,16,161,128,87,254,
    24,248,199,224,45,106,199,70,137,21,30,3,98,150,153,89,
    2,127,156,153,101,240,39,184,49,14,157,105,144,19,176,62,
    73,98,235,199,81,237,117,84,59,197,106,255,65,106,125,236,
    153,5,255,56,137,227,188,190,37,201,60,73,242,120,83,172,
    101,218,204,242,4,172,159,52,237,153,76,251,84,166,61,155,
    105,207,101,218,243,153,246,233,76,251,76,166,125,54,211,62,
    151,105,47,100,218,231,51,237,11,220,70,11,79,194,230,34,
    108,86,96,243,34,212,208,233,39,82,107,46,129,204,193,230,
    101,88,191,12,18,127,47,193,62,198,197,63,153,121,227,10,
    191,49,147,190,177,196,111,92,133,245,171,32,241,119,73,191,
    81,132,181,234,28,198,58,248,55,254,171,98,172,65,77,32,
    217,145,113,18,68,161,19,132,181,40,176,168,191,72,132,144,
    225,17,25,235,66,228,41,65,228,239,192,248,240,173,46,68,
    246,0,21,11,178,165,105,193,30,55,246,44,232,84,97,87,
    192,102,30,252,28,236,226,48,5,154,64,93,192,190,5,223,
    231,72,96,15,105,30,3,121,1,242,74,227,99,147,3,169,
    53,141,193,94,1,118,11,176,246,106,215,34,198,86,9,226,
    191,193,15,11,172,244,24,43,181,96,23,105,30,246,243,176,
    87,132,151,40,132,172,205,18,153,47,94,237,162,165,200,89,
    171,230,113,182,171,25,115,201,20,63,136,67,183,37,213,52,
    182,157,182,27,187,45,231,137,155,200,167,174,215,144,213,178,
    145,138,146,229,182,171,26,54,191,150,35,127,180,218,138,213,
    69,161,84,227,216,168,5,161,239,180,34,127,187,41,213,49,
    210,229,212,130,166,116,28,238,252,69,171,29,197,234,89,28,
    71,177,77,46,101,102,51,114,211,55,200,161,94,51,74,100,
    149,70,227,97,108,82,175,72,186,214,102,141,52,1,158,42,
    189,236,203,196,139,131,182,194,72,105,141,36,77,218,170,20,
    35,38,201,26,146,149,70,212,146,43,81,51,192,176,190,233,
    116,86,218,113,84,71,27,87,234,178,117,251,70,162,220,141,
    166,92,217,216,14,154,254,202,171,123,119,86,218,29,213,136,
    194,149,214,237,149,32,84,18,29,211,92,233,115,201,50,138,
    156,36,229,175,131,186,19,176,89,78,67,54,219,50,158,36,
    238,89,26,88,76,139,9,81,20,57,81,21,147,216,42,224,
    95,78,44,88,227,98,53,32,195,60,50,150,48,149,207,162,
    136,66,43,96,203,130,120,129,48,178,137,191,130,130,138,72,
    89,163,62,139,251,126,73,30,209,220,205,28,69,94,51,119,
    25,87,8,48,148,124,64,161,14,129,193,81,128,205,34,104,
    208,32,214,52,138,226,14,81,20,39,53,22,42,207,67,242,
    103,64,15,35,92,118,161,11,165,253,28,136,112,26,84,153,
    42,7,114,231,112,192,223,51,26,215,170,52,253,85,70,133,
    106,4,73,244,58,100,223,83,155,243,103,13,61,243,188,243,
    205,198,166,244,84,178,136,140,111,163,237,138,231,134,97,164,
    42,174,239,87,92,165,226,96,99,91,201,164,162,162,202,82,
    82,165,112,218,39,12,176,82,125,157,182,1,18,5,29,129,
    164,31,252,192,83,248,48,195,15,28,133,68,42,4,69,35,
    242,19,228,147,138,186,84,54,77,82,145,147,35,158,8,99,
    198,33,81,26,30,229,142,227,243,99,51,19,6,102,181,104,
    96,148,200,102,77,149,25,145,110,146,56,60,19,226,51,248,
    72,241,142,219,220,150,172,29,17,164,112,66,212,212,115,24,
    1,252,78,147,41,198,114,54,39,140,66,191,131,179,11,188,
    107,52,240,105,6,225,4,195,112,22,33,56,134,180,136,255,
    23,197,156,229,229,187,192,43,26,240,81,9,84,192,161,23,
    221,232,35,16,247,177,220,84,45,174,23,108,17,103,228,37,
    106,209,203,246,2,145,243,68,46,16,89,52,70,127,88,203,
    39,251,45,191,75,163,89,108,46,27,70,193,201,25,195,252,
    158,172,58,115,144,85,88,20,215,40,59,44,202,161,131,236,
    200,83,1,141,31,17,69,81,206,187,28,36,47,168,92,83,
    22,177,50,74,24,132,62,181,14,18,130,221,100,83,1,173,
    30,51,88,182,9,160,89,148,214,51,40,181,41,66,12,81,
    251,140,169,134,14,73,104,112,218,231,72,85,97,128,159,43,
    68,46,142,198,217,7,48,171,31,130,217,125,26,120,186,11,
    179,73,134,87,25,255,166,45,47,215,141,64,186,66,206,244,
    193,139,176,149,31,128,173,171,212,202,29,182,121,164,176,234,
    90,250,243,12,172,104,114,86,214,160,85,108,116,230,201,142,
    44,160,230,113,177,127,25,206,227,250,109,241,250,253,9,175,
    223,188,7,224,93,151,46,208,57,174,209,186,81,32,135,212,
    114,48,215,93,151,147,18,82,52,231,77,167,18,213,42,138,
    45,166,122,250,96,41,89,94,74,238,99,165,172,60,226,26,
    165,107,165,174,134,177,108,83,53,163,87,159,189,241,36,175,
    135,252,228,56,186,120,57,92,200,156,238,58,139,216,154,37,
    119,90,198,207,92,198,19,21,83,245,30,129,167,203,169,167,
    105,226,95,209,80,101,118,115,78,204,35,142,202,130,231,227,
    232,154,205,187,45,238,197,191,39,228,122,178,89,2,237,195,
    237,53,61,91,54,132,76,178,63,238,193,202,7,53,195,94,
    65,189,191,50,24,41,30,96,132,254,114,6,244,127,4,222,
    133,10,248,3,16,10,48,216,93,208,167,57,66,97,159,33,
    241,95,3,103,199,128,245,159,235,204,26,173,249,44,129,229,
    39,185,203,162,122,59,240,21,252,41,147,90,102,209,206,117,
    247,153,217,69,59,159,214,40,134,207,145,22,230,124,111,49,
    163,208,52,220,132,196,116,133,58,200,214,131,69,32,221,32,
    98,133,254,176,88,58,166,7,113,104,62,223,31,32,137,150,
    189,115,98,198,202,224,227,83,34,55,83,104,8,195,251,96,
    83,91,132,225,43,180,163,87,129,239,104,252,60,207,120,106,
    140,51,54,213,144,226,190,96,112,127,51,197,189,228,213,234,
    45,31,58,136,90,20,234,125,75,224,73,19,55,106,116,176,
    203,131,44,192,122,145,50,132,55,213,162,155,64,194,212,47,
    170,118,61,75,33,59,100,85,187,42,141,182,14,36,145,55,
    35,168,11,20,203,7,77,183,181,225,187,143,168,242,37,52,
    154,103,82,202,50,83,159,206,78,157,210,65,12,155,61,63,
    222,54,38,236,140,160,38,220,1,94,231,244,212,57,3,252,
    200,227,66,240,162,33,43,45,217,218,192,115,101,35,104,87,
    106,77,183,206,81,201,117,77,251,198,152,166,56,172,253,27,
    139,228,58,209,168,226,69,33,22,234,109,79,69,113,197,151,
    120,220,146,126,229,70,133,171,124,37,72,42,238,6,246,186,
    158,210,240,238,205,79,222,189,186,113,61,225,141,234,214,107,
    106,142,40,170,14,158,163,3,220,176,239,64,186,164,234,35,
    94,90,180,121,43,174,179,5,7,197,131,148,234,232,58,69,
    155,12,123,153,200,71,48,186,218,126,171,27,195,132,92,85,
    20,231,172,146,165,166,178,233,249,156,222,73,14,39,233,63,
    197,17,146,84,95,4,117,83,181,72,146,114,140,206,240,68,
    75,84,222,215,203,134,57,206,116,130,153,147,134,121,156,233,
    20,51,167,13,243,4,211,147,204,156,49,204,83,76,103,153,
    57,103,152,243,76,79,51,243,140,97,158,101,122,142,153,11,
    134,121,158,233,5,102,46,26,102,133,233,69,102,94,50,204,
    203,76,175,48,115,201,48,175,50,189,198,204,170,97,126,196,
    244,58,51,127,98,152,31,51,189,193,204,101,195,92,97,250,
    9,51,63,53,204,155,76,63,99,230,45,195,188,205,244,14,
    51,239,26,230,61,166,63,101,230,207,12,243,62,211,7,204,
    124,104,152,143,152,126,206,204,47,204,133,222,99,102,62,129,
    245,167,116,35,69,156,47,169,156,142,253,183,229,148,11,210,
    136,74,209,222,255,180,138,218,119,255,15,51,183,239,65,119,
    43,53,172,130,138,172,89,147,186,130,110,10,115,122,203,218,
    196,87,68,115,135,211,216,241,98,233,42,169,195,179,48,18,
    35,185,6,235,97,127,119,80,20,15,159,51,30,167,246,236,
    243,6,178,115,138,163,166,15,173,28,53,241,50,60,139,7,
    142,60,31,56,30,208,129,99,151,141,119,44,125,230,56,0,
    99,33,245,1,185,51,148,175,157,62,63,232,51,5,205,204,
    109,183,101,232,219,215,33,123,76,224,238,17,196,159,74,254,
    95,32,179,151,203,137,83,120,46,56,156,117,180,160,101,236,
    227,240,21,210,60,27,77,32,25,173,127,53,104,173,62,131,
    236,170,102,63,32,194,235,88,186,132,217,159,167,97,88,28,
    0,69,220,131,199,78,236,134,117,153,208,65,244,189,50,184,
    183,228,155,131,12,79,157,25,244,78,146,68,30,107,28,222,
    75,186,200,127,252,164,174,13,144,243,101,203,165,27,222,164,
    129,131,201,68,198,59,146,117,30,85,150,70,32,240,13,232,
    83,151,6,232,168,69,49,238,83,124,39,9,163,168,173,29,
    114,4,49,26,133,246,19,189,236,129,174,108,224,182,164,137,
    73,24,122,157,161,238,206,202,24,119,103,120,170,50,224,157,
    32,113,84,212,118,154,114,71,54,89,241,123,133,72,51,157,
    244,179,204,129,198,182,220,55,78,43,192,83,189,23,109,135,
    106,168,79,250,196,140,79,122,217,3,193,64,81,73,134,66,
    69,247,26,168,240,147,186,58,64,174,29,203,154,84,94,195,
    137,66,199,245,60,153,104,149,71,20,37,253,116,195,126,184,
    75,93,120,135,6,25,243,32,239,17,33,229,229,140,114,220,
    109,94,25,240,6,2,179,141,219,109,217,3,144,35,9,210,
    0,148,245,253,29,106,105,192,219,137,252,237,182,12,85,224,
    54,179,126,58,154,36,13,196,39,249,254,30,190,184,59,244,
    126,240,131,78,215,161,157,164,143,47,191,241,65,157,29,36,
    213,73,148,108,177,146,119,116,147,26,190,24,231,199,129,195,
    41,183,158,12,157,11,119,154,185,208,131,186,56,72,170,174,
    48,172,24,79,46,35,164,235,253,82,164,148,247,8,89,238,
    192,247,94,199,129,146,206,198,118,173,38,187,217,240,126,41,
    163,189,135,171,110,14,66,35,125,50,106,145,171,2,95,98,
    46,134,161,244,232,218,46,147,212,63,226,181,164,139,253,119,
    139,13,215,236,181,183,127,204,132,134,191,214,51,161,161,98,
    188,226,243,198,204,151,77,137,158,235,27,69,127,150,212,167,
    67,31,215,183,56,234,56,142,190,36,196,231,166,227,140,234,
    88,248,16,120,211,172,239,98,240,88,40,138,120,48,156,21,
    71,248,177,74,197,146,224,179,120,223,215,100,61,107,90,63,
    245,133,89,39,177,137,99,79,165,187,4,254,234,105,206,196,
    180,161,224,139,162,85,183,165,191,88,241,247,24,251,50,116,
    111,207,237,107,233,110,131,170,55,223,82,234,187,96,220,240,
    241,61,1,95,11,216,159,17,159,182,72,173,219,203,198,228,
    229,199,184,137,176,105,15,225,236,72,186,98,224,15,175,173,
    219,188,120,101,229,14,92,243,252,160,132,158,30,40,180,166,
    75,192,185,190,78,25,110,183,156,175,101,43,138,59,95,71,
    190,228,220,234,153,73,247,34,67,139,152,233,12,158,71,175,
    236,161,161,180,16,118,234,175,145,188,196,31,238,127,218,140,
    188,45,233,119,101,206,15,151,249,50,106,185,200,31,60,202,
    90,96,70,57,209,215,239,199,244,214,108,31,23,119,62,88,
    179,77,177,29,236,226,23,84,255,8,4,166,143,111,34,250,
    173,162,16,167,79,124,127,209,123,210,225,236,137,101,61,192,
    112,196,172,33,149,238,110,254,31,26,68,28,42,234,153,55,
    71,148,105,250,46,80,127,174,120,68,251,45,206,10,250,148,
    89,154,42,97,214,209,153,32,39,202,120,42,200,231,38,166,
    75,249,137,241,82,190,52,150,227,47,80,147,98,198,42,231,
    75,227,19,98,216,207,34,230,98,217,90,188,84,18,255,1,
    196,101,220,243,
};

EmbeddedPython embedded_m5_internal_param_BaseCache(
    "m5/internal/param_BaseCache.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_BaseCache.py",
    "m5.internal.param_BaseCache",
    data_m5_internal_param_BaseCache,
    2852,
    9199);

} // anonymous namespace
