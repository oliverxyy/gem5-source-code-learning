#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_PciConfigAll[] = {
    120,156,197,88,95,115,219,198,17,223,3,64,74,164,36,235,
    191,255,202,22,157,196,49,235,105,196,54,141,227,204,196,117,
    235,56,233,76,50,19,197,133,210,218,97,50,69,33,224,72,
    129,2,1,14,112,146,205,140,212,206,68,158,54,15,157,233,
    83,63,66,30,250,109,250,105,250,218,238,238,1,32,36,82,
    78,102,82,179,50,121,94,222,237,237,221,238,254,118,111,239,
    60,200,254,42,248,253,117,3,32,253,183,0,240,241,35,32,
    4,232,11,104,11,16,82,128,191,6,251,21,72,222,1,191,
    2,47,0,218,6,72,3,78,144,48,225,75,3,162,121,158,
    83,133,208,228,30,1,195,58,72,11,218,21,120,18,45,131,
    37,171,176,95,135,228,143,32,132,136,4,60,245,103,192,159,
    133,23,40,29,137,26,11,156,5,234,172,115,103,13,252,57,
    238,172,131,63,207,196,28,12,151,64,206,67,123,129,216,218,
    23,80,236,29,20,187,200,98,255,69,98,125,28,89,7,255,
    2,177,227,190,190,32,78,139,56,121,189,69,150,178,148,239,
    114,25,218,43,57,189,90,162,215,74,244,122,137,190,88,162,
    47,149,232,203,37,250,74,137,190,90,162,175,149,232,141,18,
    125,189,68,223,40,209,155,37,186,193,52,106,190,2,189,155,
    208,123,13,122,175,67,7,157,177,92,104,249,6,72,19,122,
    183,160,125,11,36,126,222,128,19,244,151,191,82,154,241,38,
    207,88,45,102,220,230,25,77,104,55,65,226,231,182,158,81,
    133,157,230,69,196,64,240,31,252,107,34,6,64,205,99,115,
    40,147,52,136,35,39,136,58,113,96,208,120,149,26,66,140,
    71,205,76,6,157,71,4,157,127,2,227,198,55,50,232,28,
    3,10,22,164,75,104,192,49,19,199,6,12,155,112,36,160,
    103,129,111,194,17,46,83,161,13,116,5,156,24,240,149,73,
    12,199,216,90,232,224,27,96,41,141,155,30,59,88,75,154,
    129,227,10,28,85,96,231,233,145,65,29,251,53,72,190,131,
    175,55,88,232,44,11,53,224,8,91,11,78,44,56,174,194,
    19,100,194,174,94,141,212,23,79,143,80,83,236,217,105,90,
    184,219,237,146,186,164,138,31,36,145,219,151,106,21,105,103,
    224,38,110,223,121,236,5,143,226,168,19,116,31,134,97,179,
    158,51,198,233,214,192,85,123,54,207,52,201,36,253,129,98,
    137,113,36,213,28,18,157,32,242,157,126,236,31,132,82,205,
    146,56,167,19,132,210,113,120,240,227,254,32,78,212,71,73,
    18,39,54,89,149,59,195,216,45,102,144,77,189,48,78,101,
    147,86,227,101,108,18,175,136,187,51,96,137,180,1,222,45,
    77,246,101,234,37,193,64,161,179,180,68,226,38,105,77,114,
    19,55,233,239,177,105,237,197,125,217,138,195,0,61,251,124,
    56,108,13,146,184,139,106,182,186,178,127,247,173,84,185,187,
    161,108,237,30,4,161,223,122,250,222,187,173,193,80,237,197,
    81,171,127,183,21,68,74,162,109,194,214,184,85,182,144,107,
    133,228,63,11,186,78,192,154,57,123,50,28,200,100,129,122,
    175,210,218,98,73,204,139,170,48,69,83,44,32,85,193,175,
    41,54,140,57,177,29,144,110,30,233,75,200,178,202,88,34,
    7,11,216,55,32,217,32,164,244,240,35,200,181,136,151,29,
    26,51,120,236,183,100,20,221,219,51,201,255,186,243,136,209,
    133,48,67,206,251,228,240,8,24,34,21,232,85,65,67,7,
    17,167,177,148,12,169,69,118,18,99,160,112,11,210,127,0,
    26,25,65,115,4,25,160,78,76,16,209,18,168,58,229,21,
    236,189,136,11,126,195,152,220,105,210,246,183,25,24,106,47,
    72,227,103,17,155,159,104,142,162,29,180,204,227,225,103,187,
    61,233,169,116,19,59,190,136,15,26,158,27,69,177,106,184,
    190,223,112,149,74,130,221,3,37,211,134,138,27,183,210,38,
    121,212,94,206,177,85,200,27,14,114,44,145,223,17,75,250,
    135,31,120,10,127,48,104,29,246,66,42,21,226,98,47,246,
    83,236,39,17,93,169,108,218,164,34,35,199,188,17,134,141,
    67,172,180,60,242,93,192,223,15,243,157,48,54,155,213,28,
    73,169,12,59,170,206,160,116,211,212,225,157,80,63,227,143,
    4,31,186,225,129,100,233,8,34,133,27,34,82,239,97,58,
    8,188,76,218,228,202,179,70,81,28,249,67,220,96,224,221,
    166,181,47,51,14,231,25,137,235,136,194,25,108,171,248,127,
    85,92,52,60,43,195,94,53,199,31,229,66,5,236,125,145,
    1,0,177,120,130,121,167,105,112,226,96,165,56,46,95,35,
    138,38,219,27,212,92,167,230,6,53,155,185,222,175,92,249,
    133,179,202,223,163,5,13,214,152,117,35,23,153,185,110,254,
    169,216,186,50,138,45,76,144,59,20,35,6,69,210,40,70,
    44,74,166,201,3,106,145,149,163,207,132,244,115,74,221,20,
    75,44,140,194,6,3,128,168,81,88,176,165,236,37,178,192,
    108,142,104,155,96,90,198,106,183,132,85,155,156,196,64,181,
    175,228,105,209,33,14,13,81,251,26,137,170,76,48,117,131,
    154,155,83,179,247,8,108,221,49,176,189,79,107,47,101,96,
    91,96,144,213,241,187,100,120,102,230,132,226,192,92,61,3,
    50,66,152,53,1,97,111,18,101,142,171,61,109,112,101,202,
    254,166,4,46,218,159,81,214,105,27,137,225,37,82,165,12,
    171,75,120,252,63,137,46,225,137,110,240,137,254,51,62,209,
    185,42,224,250,76,39,107,147,243,181,38,42,100,147,142,9,
    23,179,147,58,173,97,139,26,61,31,54,226,78,67,177,210,
    148,91,239,223,74,183,110,165,239,99,214,108,60,224,124,165,
    243,166,206,140,137,28,80,102,163,169,31,61,247,36,31,143,
    252,203,113,116,34,115,56,169,57,217,177,139,8,91,39,139,
    26,185,169,57,165,167,42,161,76,62,29,99,215,11,99,211,
    222,63,161,213,234,108,105,83,92,66,52,213,5,111,201,209,
    41,156,75,48,30,197,239,7,100,125,82,91,2,21,237,246,
    142,222,48,235,66,90,217,63,61,133,152,87,173,137,221,66,
    209,191,203,145,82,29,33,133,190,102,142,254,191,2,87,167,
    2,254,2,132,5,116,121,134,254,34,88,200,249,171,196,254,
    7,224,48,153,80,17,112,206,217,161,42,128,57,48,21,165,
    247,152,85,23,8,159,192,183,165,24,203,143,113,51,171,63,
    203,199,184,85,228,43,6,209,15,58,170,173,211,137,141,188,
    179,231,166,196,166,179,213,40,108,71,103,66,81,53,98,182,
    126,229,136,154,213,235,56,180,165,175,70,120,162,131,240,154,
    88,53,74,40,249,57,53,111,23,0,17,121,223,171,220,221,
    38,156,127,108,59,250,92,248,146,182,96,241,166,23,103,184,
    158,42,11,41,98,160,146,199,192,219,69,12,72,62,194,94,
    240,173,132,90,131,124,126,98,8,188,162,98,13,71,55,66,
    11,100,5,218,85,138,22,46,185,69,22,76,34,79,103,148,
    252,78,157,143,108,150,109,109,176,194,237,218,163,212,60,159,
    78,154,32,167,222,15,221,254,174,239,62,56,164,181,104,65,
    47,15,47,35,223,253,82,121,247,20,26,226,60,5,248,231,
    221,92,139,195,233,164,136,119,105,169,124,247,28,16,126,236,
    113,94,248,124,79,54,250,178,191,139,215,207,189,96,208,232,
    132,110,151,125,99,102,218,125,150,107,167,216,185,103,107,142,
    244,14,181,113,195,139,35,204,222,7,158,138,147,134,47,241,
    74,38,253,198,91,13,78,253,141,32,109,184,187,56,234,122,
    74,67,253,116,184,114,121,235,38,221,148,43,217,253,103,68,
    78,207,183,14,222,184,3,44,234,159,67,113,212,234,155,96,
    145,201,185,92,215,145,131,235,226,101,75,13,117,230,162,250,
    195,222,162,230,39,48,213,132,255,14,138,62,160,53,200,96,
    85,113,205,168,25,124,47,44,243,61,166,153,233,120,204,254,
    237,135,196,172,126,80,202,34,183,74,156,114,134,238,252,212,
    214,40,237,183,235,121,231,28,183,243,220,185,144,119,94,224,
    118,145,59,151,242,135,172,101,238,92,129,246,42,189,184,80,
    207,26,101,131,153,31,155,13,56,152,166,23,70,127,254,159,
    38,1,251,222,255,103,243,246,123,144,21,6,231,37,0,81,
    214,108,65,39,128,158,200,239,37,101,181,248,21,228,202,68,
    252,57,94,34,93,37,181,159,54,166,165,42,39,18,189,242,
    55,163,176,30,175,160,31,22,90,157,112,81,52,92,99,247,
    233,75,25,187,79,60,137,174,98,41,109,113,41,125,159,74,
    233,35,54,129,99,232,106,122,4,204,74,97,9,186,208,70,
    242,153,51,110,13,93,48,211,230,220,193,64,70,190,125,7,
    202,53,48,15,79,7,11,148,186,190,133,82,137,98,138,53,
    44,122,199,227,144,114,115,73,75,246,99,165,136,188,169,121,
    148,193,251,247,28,188,77,206,205,69,130,182,239,83,195,41,
    185,200,198,246,175,10,127,92,157,140,204,221,131,148,174,89,
    47,27,198,66,137,207,74,164,185,124,154,192,54,8,93,213,
    137,147,62,139,250,62,30,146,199,239,137,89,135,186,54,121,
    66,26,124,45,89,224,203,198,73,24,63,24,225,15,246,39,
    135,160,47,67,169,228,4,236,41,178,79,246,16,224,75,60,
    141,227,33,222,201,248,138,131,191,67,199,153,226,249,245,75,
    20,253,39,90,131,14,45,60,191,68,21,79,176,117,188,192,
    175,27,181,106,77,112,105,112,230,13,92,111,140,60,171,203,
    249,97,106,115,26,91,44,220,204,134,205,207,103,66,4,95,
    63,183,221,190,126,97,227,199,35,251,117,200,46,249,246,237,
    2,46,100,54,190,67,233,251,42,134,46,151,45,92,165,216,
    191,200,17,212,191,187,149,107,181,149,105,149,123,209,224,97,
    117,125,34,215,199,145,74,80,121,180,119,200,143,25,227,28,
    59,195,84,73,141,133,242,160,140,14,250,206,167,178,31,39,
    195,79,99,95,170,141,51,227,15,125,63,177,221,168,43,157,
    67,73,101,151,186,121,150,33,171,185,180,140,156,171,49,113,
    15,167,121,199,246,162,153,112,80,191,172,50,208,199,199,31,
    133,177,183,47,253,140,103,178,57,152,231,195,184,239,98,255,
    228,85,118,130,124,149,229,51,227,126,66,179,214,207,244,166,
    50,9,220,16,131,224,28,213,62,112,211,192,123,28,196,31,
    202,195,192,147,231,44,58,26,39,24,229,131,106,141,48,53,
    73,10,149,132,167,187,184,40,27,59,5,57,222,18,217,13,
    208,197,9,139,59,61,43,59,23,40,32,206,203,30,101,9,
    211,139,81,125,239,209,143,53,15,230,242,181,232,69,183,182,
    88,195,120,165,19,195,20,117,60,51,44,115,126,169,102,205,
    207,213,172,218,140,201,79,112,11,120,217,173,91,181,185,121,
    241,178,127,155,24,233,117,99,115,165,38,254,11,185,2,178,
    254,
};

EmbeddedPython embedded_m5_internal_param_PciConfigAll(
    "m5/internal/param_PciConfigAll.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_PciConfigAll.py",
    "m5.internal.param_PciConfigAll",
    data_m5_internal_param_PciConfigAll,
    2417,
    7451);

} // anonymous namespace