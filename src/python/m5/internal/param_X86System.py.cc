#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_X86System[] = {
    120,156,197,89,221,111,27,199,17,159,189,35,41,145,146,44,
    201,250,242,135,18,209,78,108,51,142,45,198,110,28,7,136,
    235,214,95,1,28,192,138,122,116,97,71,9,122,61,241,86,
    212,81,228,29,113,183,178,205,64,70,139,200,104,83,160,111,
    69,95,250,214,135,62,244,191,233,127,212,206,204,222,157,86,
    20,109,185,31,102,37,113,61,220,157,157,157,153,253,205,236,
    236,186,9,233,79,17,63,63,175,2,36,161,5,224,227,159,
    128,14,64,87,192,134,0,33,5,248,243,176,83,132,248,83,
    240,139,240,10,96,195,2,105,193,62,18,54,124,107,65,56,
    201,115,74,208,177,185,71,64,191,2,178,0,27,69,120,18,
    206,66,65,150,96,167,2,241,175,65,8,17,10,120,234,143,
    129,63,14,175,80,58,18,101,22,56,14,212,89,225,206,50,
    248,19,220,89,1,127,146,137,9,232,207,128,156,132,141,41,
    98,219,56,129,98,47,163,216,105,22,251,15,18,235,227,200,
    2,248,39,136,29,245,250,134,56,11,196,201,235,77,179,148,
    153,76,203,89,216,56,153,209,115,6,61,111,208,11,6,189,
    104,208,75,6,125,202,160,79,27,244,25,131,62,107,208,203,
    6,253,158,65,191,111,208,43,6,93,53,232,115,6,125,222,
    160,63,48,232,15,13,250,130,65,95,52,232,75,6,93,51,
    232,143,12,250,50,211,232,241,147,208,254,24,218,87,160,125,
    21,182,16,4,179,185,119,87,65,218,208,174,195,70,29,36,
    254,173,194,62,226,196,63,105,204,248,132,103,204,229,51,174,
    241,140,235,176,113,29,36,254,93,211,51,74,208,168,45,34,
    246,130,127,226,79,77,32,165,38,177,121,38,227,36,136,66,
    55,8,183,162,192,162,241,18,53,132,212,38,53,99,41,100,
    239,17,100,255,14,140,87,223,74,33,251,18,80,176,32,91,
    58,22,188,100,226,165,5,253,26,236,9,104,23,192,183,97,
    15,151,41,146,2,45,1,251,22,124,103,19,195,75,108,11,
    8,172,247,161,160,52,94,219,12,44,45,105,12,94,22,97,
    175,8,141,167,123,22,117,236,148,33,254,27,124,191,204,66,
    199,89,168,5,123,216,22,96,191,0,47,75,240,4,153,176,
    171,93,38,243,197,211,61,180,20,123,26,181,2,106,187,102,
    152,75,166,248,65,28,122,93,169,102,144,118,123,94,236,117,
    221,167,159,127,214,232,39,74,118,107,149,140,43,74,86,123,
    158,218,118,120,154,77,254,232,246,20,139,139,66,169,38,144,
    216,10,66,223,237,70,254,110,71,170,113,146,229,110,5,29,
    233,186,60,248,176,219,139,98,245,32,142,163,216,33,151,114,
    103,39,242,242,25,228,208,102,39,74,100,141,86,227,101,28,
    18,175,136,123,171,199,18,73,1,86,149,38,251,50,105,198,
    65,79,225,78,105,137,196,77,210,106,180,71,220,36,13,108,
    234,219,81,87,214,163,78,128,219,250,162,223,175,247,226,168,
    133,54,214,91,178,123,227,106,162,188,205,142,172,111,238,6,
    29,191,142,86,215,123,125,181,29,133,245,238,141,122,16,42,
    137,142,233,212,7,92,178,138,44,39,73,248,243,160,229,6,
    108,150,187,45,59,61,25,79,81,239,25,90,88,204,136,73,
    81,18,182,168,137,41,164,138,248,177,197,178,53,33,214,2,
    50,172,73,198,18,166,10,38,138,104,107,5,236,88,16,47,
    19,70,218,248,39,104,83,17,41,13,26,179,120,236,23,228,
    17,221,219,182,105,231,117,231,30,227,10,1,134,156,183,104,
    171,67,96,112,20,161,93,2,13,26,196,154,70,81,220,167,
    22,217,73,140,133,194,11,144,252,25,208,195,8,151,61,72,
    161,180,111,131,8,103,64,85,40,147,97,239,34,46,248,3,
    163,177,81,35,245,215,24,21,106,59,72,162,231,33,251,158,
    104,142,159,6,122,102,189,255,245,102,91,54,85,178,130,29,
    223,68,187,213,166,23,134,145,170,122,190,95,245,148,138,131,
    205,93,37,147,170,138,170,23,146,26,109,167,51,155,1,43,
    151,215,239,101,64,162,77,71,32,233,47,126,208,84,248,101,
    142,191,240,46,36,82,33,40,182,35,63,193,126,18,209,146,
    202,33,37,21,57,57,98,69,24,51,46,177,210,242,200,119,
    2,191,223,201,52,97,96,214,74,25,140,18,217,217,82,21,
    70,164,151,36,46,107,66,253,12,62,18,252,204,235,236,74,
    150,142,8,82,168,16,145,90,135,17,192,239,20,153,146,89,
    206,230,132,81,232,247,81,187,160,121,137,22,62,197,32,156,
    100,24,46,32,4,199,176,45,225,191,37,177,104,53,11,41,
    240,74,25,248,40,5,42,224,173,23,233,238,35,16,247,49,
    221,212,44,206,23,108,17,71,228,121,162,104,178,179,76,205,
    123,212,188,79,205,74,102,244,187,181,124,106,208,242,155,180,
    154,197,230,178,97,180,57,118,102,152,127,40,170,78,31,68,
    21,38,197,6,69,135,69,49,116,16,29,5,74,160,241,109,
    106,145,149,227,206,134,228,49,165,107,138,34,22,70,1,131,
    208,39,234,32,32,216,77,14,37,208,218,120,134,101,135,0,
    106,162,180,101,160,212,161,29,98,136,58,167,179,108,232,18,
    135,6,167,115,150,68,21,135,248,185,74,205,185,209,56,251,
    0,102,173,35,48,251,130,22,158,73,97,54,197,240,170,224,
    103,198,106,218,233,14,228,39,228,220,0,188,8,91,133,33,
    216,186,72,148,125,212,230,145,194,42,181,244,75,3,86,164,
    156,101,26,180,134,68,127,137,236,48,1,181,132,135,253,147,
    112,9,207,111,139,207,239,79,248,252,230,26,128,171,64,157,
    160,109,206,209,154,40,146,67,182,108,88,76,207,229,164,140,
    45,154,243,162,95,141,182,170,138,45,166,124,122,235,66,178,
    122,33,249,2,51,101,245,54,231,40,157,43,117,54,140,101,
    143,178,25,77,125,240,162,41,249,60,228,111,174,171,147,151,
    203,137,204,77,207,89,196,214,2,185,211,202,252,204,105,60,
    81,49,101,239,17,120,186,146,123,154,20,255,138,150,170,176,
    155,109,177,132,56,170,8,214,199,213,57,155,171,45,30,197,
    207,93,114,61,217,44,129,238,5,78,67,107,203,134,144,73,
    206,149,67,88,121,167,102,56,117,148,251,203,12,35,165,3,
    140,208,199,206,64,255,123,224,42,84,192,239,128,80,128,155,
    157,130,62,143,17,218,246,57,98,255,21,112,116,12,57,255,
    57,207,52,232,204,103,14,76,63,201,77,102,213,229,192,87,
    240,163,17,90,217,161,109,167,117,166,121,104,23,242,28,197,
    240,121,171,131,185,112,56,153,209,214,108,123,9,177,233,12,
    117,16,173,7,135,64,94,32,98,134,126,183,88,26,215,139,
    184,164,207,119,7,72,162,99,239,172,152,179,12,124,92,163,
    230,122,14,13,145,245,189,51,213,86,224,245,39,180,171,79,
    129,111,105,253,2,107,60,61,198,17,155,75,200,113,95,204,
    112,127,61,199,189,228,211,234,21,95,58,168,181,104,171,247,
    45,129,55,95,44,212,232,162,89,0,89,132,141,18,69,8,
    23,213,34,13,32,145,229,47,202,118,135,142,66,118,200,154,
    118,85,190,219,122,35,169,121,49,130,188,64,123,121,171,227,
    117,55,125,239,246,111,105,33,90,173,153,133,148,149,169,62,
    99,170,78,225,32,94,167,61,127,189,145,153,240,108,4,57,
    225,51,148,155,171,206,17,224,71,77,78,4,143,183,101,181,
    43,187,155,120,175,220,14,122,213,173,142,215,226,93,177,83,
    211,190,206,76,83,188,173,131,133,69,114,153,218,168,218,140,
    66,76,212,187,77,21,197,85,95,226,117,75,250,213,171,85,
    206,242,213,32,169,122,155,56,234,53,149,134,247,225,248,228,
    234,213,139,91,9,23,170,59,207,137,28,209,174,186,120,143,
    14,176,96,255,1,242,35,85,95,241,242,164,205,165,184,142,
    22,92,20,47,82,170,175,243,20,21,25,206,42,53,31,193,
    232,114,251,167,40,247,55,180,0,185,170,36,206,90,101,75,
    77,155,225,185,78,115,146,163,65,250,215,183,9,82,253,48,
    149,134,106,137,56,229,24,221,225,169,45,83,122,223,168,100,
    157,19,220,78,114,231,84,214,121,130,219,105,238,156,201,58,
    103,185,61,201,157,115,217,43,217,60,119,46,192,198,34,61,
    171,80,207,18,229,132,177,255,54,39,112,84,141,40,158,254,
    240,63,77,5,206,205,255,131,230,206,231,144,214,3,175,75,
    3,194,52,107,74,167,129,182,200,174,32,166,77,252,206,177,
    120,20,139,110,51,150,158,146,122,123,150,71,98,36,39,18,
    189,236,31,15,34,251,104,177,124,39,183,103,159,171,160,254,
    60,239,154,190,121,241,174,137,39,225,25,172,154,11,92,53,
    223,162,170,121,143,141,119,45,93,56,31,128,177,152,251,128,
    220,25,202,231,238,128,31,116,97,76,154,121,189,158,12,125,
    231,50,152,181,46,15,143,96,255,41,111,253,9,140,130,196,
    22,243,88,220,30,141,58,202,202,134,125,188,125,197,60,206,
    70,179,145,140,214,191,100,104,173,241,197,52,79,205,206,45,
    106,56,25,231,121,216,249,89,190,13,215,135,64,209,107,246,
    2,215,120,156,115,89,43,183,23,177,14,116,193,250,79,166,
    97,25,165,168,198,124,51,155,250,112,136,100,26,233,184,221,
    222,33,21,222,138,145,22,37,219,7,7,212,249,55,205,214,
    26,209,34,111,193,70,75,156,48,151,224,110,85,29,50,51,
    233,110,6,81,98,136,63,150,137,132,211,5,210,236,100,112,
    114,14,241,37,46,36,7,67,72,63,3,235,211,24,253,172,
    226,168,143,87,72,190,148,225,247,142,235,142,234,24,254,41,
    202,253,145,22,152,7,62,134,69,9,15,226,5,193,191,86,
    185,84,22,92,221,12,188,207,107,189,126,2,217,21,164,159,
    56,156,131,167,115,200,242,59,114,86,101,16,186,185,244,94,
    243,186,250,13,144,95,184,156,15,32,125,143,112,46,229,208,
    167,119,26,190,247,233,219,53,102,31,174,188,184,208,114,120,
    69,122,177,232,222,88,205,140,90,205,141,186,115,111,253,161,
    211,184,191,206,111,217,221,27,199,50,62,86,23,7,56,210,
    49,244,206,125,196,254,99,222,221,103,146,42,65,117,225,77,
    194,204,9,234,236,80,206,70,208,213,79,181,106,118,96,220,
    143,61,164,23,6,122,19,25,7,94,39,248,94,190,217,140,
    167,100,198,199,175,227,120,72,112,127,180,254,101,39,242,84,
    16,182,214,211,184,186,116,12,255,189,40,220,10,90,218,154,
    43,71,93,148,114,221,245,18,169,57,31,132,42,238,103,142,
    58,78,153,129,105,71,248,15,56,31,188,80,67,228,95,62,
    70,254,225,89,71,182,248,32,4,30,221,197,104,213,173,54,
    117,136,38,38,83,131,47,5,187,177,60,86,147,161,179,248,
    157,111,8,46,56,18,143,128,70,134,187,93,247,145,236,70,
    113,255,81,228,75,181,60,48,126,199,247,99,199,11,91,185,
    50,231,6,25,210,107,138,150,145,113,85,135,234,112,152,247,
    53,0,198,193,20,192,43,67,199,239,117,162,230,142,244,83,
    158,225,160,101,158,251,81,151,0,63,158,166,99,26,230,244,
    121,200,31,148,106,52,201,23,133,195,213,24,167,205,88,182,
    130,132,208,60,153,179,166,213,9,165,52,118,216,145,172,109,
    76,27,81,126,213,55,110,253,40,120,155,158,159,19,172,164,
    129,254,195,160,60,93,198,92,75,69,139,45,42,88,182,20,
    236,201,153,114,97,114,162,92,40,143,217,252,206,59,37,230,
    172,74,161,60,49,41,254,221,223,21,204,219,21,107,101,174,
    44,254,5,236,166,142,73,
};

EmbeddedPython embedded_m5_internal_param_X86System(
    "m5/internal/param_X86System.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_X86System.py",
    "m5.internal.param_X86System",
    data_m5_internal_param_X86System,
    2583,
    8189);

} // anonymous namespace
