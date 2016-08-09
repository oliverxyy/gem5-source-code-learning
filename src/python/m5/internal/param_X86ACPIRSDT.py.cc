#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_X86ACPIRSDT[] = {
    120,156,197,88,253,114,219,198,17,223,3,64,74,164,36,139,
    178,44,201,182,100,137,109,199,9,235,54,98,155,70,113,102,
    172,186,117,156,116,198,153,142,236,130,206,216,81,50,69,33,
    224,72,130,2,1,14,112,178,197,140,212,63,42,79,219,23,
    232,35,244,143,190,77,223,168,221,221,3,64,232,195,105,102,
    90,179,50,113,62,28,238,246,246,227,183,31,119,30,100,127,
    21,124,126,221,4,72,255,40,0,124,252,9,8,1,134,2,
    246,5,8,41,192,191,1,135,21,72,62,2,191,2,111,0,
    246,13,144,6,156,97,199,132,175,13,136,230,121,77,21,66,
    147,71,4,140,235,32,45,216,175,192,139,104,9,44,89,133,
    195,58,36,127,0,33,68,36,224,165,63,3,254,44,188,65,
    234,216,169,49,193,89,160,193,58,15,214,192,159,227,193,58,
    248,243,220,153,131,113,3,228,60,236,47,208,180,253,107,72,
    246,30,146,93,100,178,255,36,178,62,126,89,1,255,26,77,
    71,190,190,162,153,22,205,228,253,22,153,74,35,231,114,9,
    246,175,231,253,229,82,255,70,169,191,82,234,175,114,31,57,
    184,14,131,53,24,220,132,193,45,232,162,82,150,138,221,110,
    131,52,97,176,14,251,235,32,241,119,27,206,80,111,254,245,
    210,138,13,94,177,92,172,184,195,43,54,97,127,19,36,254,
    238,232,21,85,232,180,86,209,22,193,191,240,175,133,182,0,
    53,143,205,43,153,164,65,28,57,65,212,141,3,131,190,87,
    169,33,203,121,212,204,100,38,124,76,38,252,7,176,253,124,
    35,51,225,41,32,97,65,178,132,6,156,114,231,212,128,113,
    11,78,4,12,44,240,77,56,193,109,42,196,64,79,192,153,
    1,223,152,52,225,20,91,11,21,189,9,150,210,246,27,176,
    162,53,165,25,56,173,192,73,5,58,47,79,12,26,56,172,
    65,242,119,248,118,131,137,206,50,81,3,78,176,181,224,204,
    130,211,42,188,192,73,56,52,168,145,248,226,229,9,74,138,
    35,157,150,133,220,238,149,196,37,81,252,32,137,220,161,84,
    215,177,239,140,220,196,29,58,47,63,249,248,209,227,103,79,
    236,206,103,207,91,245,124,94,156,110,143,92,213,183,121,161,
    73,26,25,142,20,19,140,35,169,230,176,211,13,34,223,25,
    198,254,81,40,213,44,81,115,186,65,40,29,135,63,62,25,
    142,226,68,125,158,36,113,98,147,82,121,48,140,221,98,5,
    169,212,11,227,84,182,104,55,222,198,38,242,138,102,119,71,
    76,145,24,96,102,105,177,47,83,47,9,70,10,109,165,41,
    210,108,162,214,34,43,113,147,126,137,77,187,31,15,101,59,
    14,3,52,236,241,120,220,30,37,113,15,165,108,247,228,112,
    231,131,84,185,7,161,108,31,28,5,161,223,70,185,219,163,
    177,234,199,81,123,184,211,14,34,37,81,53,97,251,146,82,
    182,113,18,169,43,125,29,244,156,128,5,115,250,50,28,201,
    100,129,70,111,211,214,162,33,230,69,85,152,162,37,22,176,
    87,193,199,20,27,198,156,216,11,72,52,143,196,37,92,89,
    101,36,145,121,5,28,26,144,108,16,78,6,248,19,100,88,
    68,75,135,190,25,252,237,119,164,19,61,58,48,201,250,122,
    240,132,177,133,32,195,153,187,100,238,8,24,32,21,24,84,
    65,3,7,241,166,145,148,140,169,197,233,68,198,64,226,22,
    164,127,3,212,49,66,230,4,50,56,157,153,32,162,6,168,
    58,121,55,142,174,226,134,127,98,68,118,90,196,254,30,227,
    66,245,131,52,126,29,177,246,169,207,62,212,65,205,60,27,
    63,61,24,72,79,165,91,56,240,85,124,212,244,220,40,138,
    85,211,245,253,166,171,84,18,28,28,41,153,54,85,220,188,
    155,182,200,160,246,82,14,173,130,222,120,148,67,137,204,142,
    80,210,47,126,224,41,124,89,230,23,182,66,42,21,194,162,
    31,251,41,142,19,137,158,84,54,49,169,72,201,49,51,194,
    168,113,104,42,109,143,243,174,225,251,163,156,19,134,102,171,
    154,3,41,149,97,87,213,25,147,110,154,58,204,9,141,51,
    252,136,240,43,55,60,146,76,29,49,164,144,33,234,106,30,
    166,2,192,155,36,76,46,59,11,20,197,145,63,70,254,2,
    239,125,218,250,38,195,112,158,129,184,130,32,156,193,182,138,
    255,87,197,170,225,89,25,244,170,57,252,40,16,42,96,227,
    139,204,254,8,197,51,12,58,45,131,163,6,203,196,94,249,
    67,234,209,98,123,131,154,59,212,108,82,179,149,139,253,174,
    101,95,184,40,251,125,218,207,96,129,89,52,50,144,153,139,
    230,159,243,172,91,19,207,194,224,216,33,15,49,200,143,38,
    30,98,81,32,77,30,82,139,83,217,247,76,72,159,83,216,
    38,79,98,98,228,52,8,127,234,77,156,130,21,101,55,72,
    1,179,57,158,109,2,105,25,169,189,18,82,109,178,17,195,
    212,190,149,199,68,135,102,104,128,218,235,68,170,114,133,166,
    155,212,252,96,90,234,158,64,173,119,9,106,15,104,235,70,
    6,181,5,134,88,29,159,134,225,153,153,13,138,92,185,124,
    1,98,132,47,235,10,124,189,71,61,243,178,212,83,134,86,
    38,235,111,74,208,34,246,140,178,72,123,216,25,175,145,36,
    101,80,173,97,226,127,17,173,97,46,55,56,151,255,140,115,
    57,215,3,92,33,233,64,109,114,172,214,157,10,169,164,107,
    194,106,150,163,211,26,182,40,208,241,184,25,119,155,138,101,
    166,184,186,123,55,221,190,155,62,192,136,217,124,200,177,74,
    199,76,29,21,19,57,162,168,70,75,63,63,246,36,103,70,
    126,115,28,29,196,28,14,104,78,150,113,17,95,43,164,80,
    35,215,52,135,243,84,37,20,197,167,162,235,122,161,107,98,
    253,11,218,172,206,138,54,197,26,98,169,46,152,35,71,71,
    111,174,189,248,43,62,159,146,242,73,106,9,84,53,219,29,
    205,47,139,66,66,217,63,61,135,151,119,44,136,221,70,202,
    95,230,56,169,78,112,66,143,153,67,255,47,192,85,169,128,
    63,3,33,1,13,158,65,191,240,20,50,253,50,77,255,61,
    176,143,92,81,11,112,188,233,80,254,231,25,24,134,210,251,
    60,85,151,6,95,192,95,75,14,150,39,112,51,171,59,203,
    9,220,42,98,21,67,232,123,37,105,235,124,80,35,227,244,
    221,148,166,233,72,53,241,217,73,58,40,202,69,140,212,239,
    26,79,179,122,27,135,56,250,102,130,38,74,129,235,98,217,
    40,97,228,231,212,124,88,192,67,228,99,239,144,185,45,120,
    123,190,118,116,70,248,154,56,176,152,231,197,25,69,58,68,
    26,79,58,143,156,199,79,127,251,116,175,227,16,185,188,79,
    100,11,151,168,228,46,241,97,225,18,146,211,217,27,62,157,
    80,107,16,6,206,12,129,71,70,172,230,232,132,102,129,172,
    192,126,149,156,135,107,111,145,249,150,200,131,27,133,194,115,
    185,146,245,180,167,53,88,192,64,91,152,154,227,169,4,13,
    50,242,110,232,14,15,124,247,97,72,91,209,126,94,238,109,
    70,206,124,163,204,60,121,138,120,27,255,252,186,147,11,241,
    106,42,1,227,99,164,92,48,207,238,225,199,30,71,137,231,
    125,217,28,202,225,1,30,66,251,193,168,217,13,221,30,91,
    198,204,132,123,154,11,167,216,180,23,171,143,244,30,181,113,
    211,139,35,140,228,71,158,138,147,166,47,241,100,38,253,230,
    7,77,78,3,205,32,109,186,7,248,213,245,148,70,254,121,
    231,229,50,215,77,122,41,87,180,135,175,169,59,53,203,58,
    120,236,14,176,182,143,160,200,186,250,60,88,68,117,174,218,
    181,35,225,182,120,230,82,99,29,198,168,18,177,183,169,249,
    49,76,51,248,127,68,90,162,45,72,93,85,177,110,212,12,
    181,164,125,55,159,246,140,214,165,151,221,245,224,251,184,171,
    190,219,201,156,182,74,51,229,12,29,251,169,173,81,6,160,
    171,155,74,118,117,67,131,250,234,166,202,35,215,200,189,103,
    254,91,247,102,247,152,154,99,28,253,79,189,218,190,255,127,
    225,221,254,4,178,188,255,54,143,22,101,193,22,180,71,15,
    68,126,228,40,75,197,183,27,55,175,130,148,227,37,210,85,
    82,27,105,99,74,130,114,92,208,27,31,79,220,244,114,113,
    252,168,144,233,140,43,158,241,13,182,157,62,109,177,237,196,
    139,232,54,86,201,22,87,201,187,84,37,159,176,2,28,67,
    23,202,19,80,86,10,61,80,32,136,228,107,231,146,46,116,
    41,76,188,185,163,145,140,124,251,30,148,171,91,254,60,21,
    28,60,208,32,157,148,31,166,184,129,229,236,101,15,164,64,
    91,146,145,141,88,41,124,110,90,230,100,220,158,229,184,109,
    45,66,57,218,218,187,212,112,124,45,66,171,253,171,194,24,
    119,174,4,165,140,176,112,148,41,157,160,254,195,12,44,133,
    184,140,204,222,89,117,140,116,95,134,82,201,203,70,86,196,
    74,118,146,246,37,38,177,120,140,199,26,62,38,224,123,232,
    56,211,139,251,191,212,240,134,148,18,18,198,125,81,197,200,
    191,98,212,170,53,193,201,244,194,221,177,230,137,170,64,93,
    14,143,83,155,227,196,98,161,76,190,225,204,83,26,233,157,
    15,111,123,238,80,223,77,241,189,139,253,35,200,78,200,246,
    251,133,81,232,238,128,207,32,250,180,135,222,193,137,158,243,
    186,253,11,26,167,5,195,157,237,92,160,237,76,148,206,56,
    253,76,166,222,115,210,128,243,74,82,181,192,215,173,195,29,
    117,247,194,138,115,42,40,175,83,235,87,206,236,4,67,125,
    253,199,185,176,252,221,79,92,236,175,92,24,77,101,18,184,
    97,240,173,190,239,203,135,21,137,246,246,173,223,131,239,42,
    145,203,83,57,75,171,159,124,231,124,50,44,35,43,145,189,
    32,69,6,120,247,43,246,205,226,13,3,96,243,74,124,151,
    169,76,13,145,186,52,214,135,251,135,116,149,148,126,138,13,
    93,255,213,22,107,136,78,10,68,38,158,170,23,132,101,206,
    55,106,214,252,92,205,170,205,152,124,99,179,128,231,163,186,
    85,155,155,23,249,191,45,196,113,221,216,106,212,196,191,1,
    16,151,227,29,
};

EmbeddedPython embedded_m5_internal_param_X86ACPIRSDT(
    "m5/internal/param_X86ACPIRSDT.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_X86ACPIRSDT.py",
    "m5.internal.param_X86ACPIRSDT",
    data_m5_internal_param_X86ACPIRSDT,
    2260,
    6841);

} // anonymous namespace
