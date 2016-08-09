#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_EtherTap[] = {
    120,156,189,88,91,115,219,198,21,62,11,128,148,72,81,22,
    117,183,45,217,98,219,113,195,122,26,177,77,227,56,51,81,
    220,38,173,59,211,60,40,9,232,142,29,38,83,20,34,150,
    36,40,16,224,0,43,91,244,72,121,168,220,203,67,95,251,
    19,250,208,127,211,127,212,156,115,22,0,33,209,154,201,76,
    35,202,196,122,177,187,56,123,46,223,185,236,118,33,253,43,
    225,243,155,6,64,242,173,0,240,240,39,32,0,24,9,232,
    8,16,82,128,183,1,199,37,136,223,7,175,4,111,0,58,
    6,72,3,46,176,99,194,215,6,132,53,254,166,12,129,201,
    35,2,38,85,144,22,116,74,240,60,92,5,75,150,225,184,
    10,241,159,65,8,17,10,120,225,45,128,183,8,111,144,58,
    118,42,76,112,17,104,176,202,131,21,240,150,120,176,10,94,
    141,59,75,48,169,131,172,65,103,153,150,117,110,33,217,135,
    72,118,133,201,254,151,200,122,56,179,9,222,45,90,142,124,
    125,69,43,45,90,201,251,173,48,149,122,198,229,42,116,214,
    178,254,122,161,191,81,232,111,22,250,91,220,71,14,214,96,
    184,13,195,219,48,188,3,61,84,202,106,190,219,93,144,38,
    12,119,160,179,3,18,127,119,225,2,245,230,173,21,190,216,
    229,47,214,243,47,238,241,23,247,161,115,31,36,254,238,233,
    47,202,208,110,110,161,45,252,255,225,95,19,109,1,170,134,
    205,75,25,39,126,20,58,126,216,139,124,131,230,203,212,144,
    229,186,212,44,164,38,252,45,153,240,63,192,246,243,140,212,
    132,231,128,132,5,201,18,24,112,206,157,115,3,38,77,56,
    19,48,180,192,51,225,12,183,41,17,3,125,1,23,6,124,
    99,210,130,115,108,45,84,244,125,176,148,182,223,144,21,173,
    41,45,192,121,9,206,74,208,126,113,102,208,192,113,5,226,
    127,195,235,93,38,186,200,68,13,56,195,214,130,11,11,206,
    203,240,28,23,225,208,176,66,226,139,23,103,40,41,142,180,
    155,22,114,123,88,16,151,68,241,252,56,116,71,82,173,96,
    223,25,187,177,59,114,158,170,129,140,159,185,227,102,53,91,
    20,37,251,99,87,13,108,254,202,36,117,140,198,138,169,69,
    161,84,75,216,233,249,161,231,140,34,239,36,144,106,145,72,
    57,61,63,144,142,195,147,127,24,141,163,88,61,141,227,40,
    182,73,163,60,24,68,110,254,5,233,179,27,68,137,108,210,
    110,188,141,77,228,21,173,238,141,153,34,49,192,156,210,199,
    158,76,186,177,63,86,104,40,77,145,86,19,181,38,153,136,
    155,196,198,166,53,136,70,178,21,5,62,90,245,116,50,105,
    141,227,168,143,34,182,250,114,244,232,221,68,185,71,129,108,
    29,157,248,129,215,122,241,225,7,173,241,68,13,162,176,53,
    122,212,242,67,37,81,47,65,235,178,70,246,113,197,26,209,
    126,229,247,29,159,165,114,6,50,24,203,120,153,70,239,210,
    190,162,46,106,162,44,76,209,20,203,216,43,225,99,138,93,
    99,73,28,250,36,87,151,100,37,68,89,69,12,145,97,5,
    28,27,16,239,18,66,134,248,19,100,82,196,73,155,230,12,
    158,251,146,20,162,71,135,38,217,93,15,158,49,170,16,94,
    184,242,128,12,29,2,67,163,4,195,50,104,200,32,210,52,
    134,226,9,181,184,156,200,24,72,220,130,228,95,128,10,70,
    176,156,65,10,164,11,19,68,88,7,85,37,191,198,209,45,
    220,240,47,140,197,118,147,216,63,100,80,168,129,159,68,175,
    66,86,61,245,217,123,218,168,153,47,38,159,31,13,101,87,
    37,123,56,240,85,116,210,232,186,97,24,169,134,235,121,13,
    87,169,216,63,58,81,50,105,168,168,241,32,105,146,53,237,
    213,12,87,57,189,201,56,195,17,217,28,113,164,95,60,191,
    171,240,101,157,95,216,10,137,84,136,137,65,228,37,56,78,
    36,250,82,217,196,164,34,37,71,204,8,67,198,161,165,180,
    61,174,187,133,239,159,100,156,48,46,155,229,12,69,137,12,
    122,170,202,128,116,147,196,97,78,104,156,177,71,132,95,186,
    193,137,100,234,8,32,133,12,81,87,243,112,243,232,187,77,
    146,100,130,179,52,97,20,122,19,100,206,239,190,67,251,222,
    102,12,214,24,133,155,136,192,5,108,203,248,127,89,108,25,
    93,43,197,93,57,195,30,197,63,5,108,121,145,26,31,113,
    120,129,177,166,105,112,176,96,129,216,31,127,76,61,250,216,
    222,165,230,30,53,247,169,217,203,100,190,81,193,151,175,10,
    254,152,54,51,88,90,150,139,76,99,102,114,121,151,124,234,
    206,212,167,48,32,182,201,55,12,242,160,169,111,88,20,60,
    227,39,212,226,82,246,58,19,146,103,20,170,201,135,152,24,
    185,11,2,159,122,83,119,96,45,217,117,146,126,49,67,178,
    77,240,44,98,180,95,192,168,77,6,98,128,218,119,178,80,
    232,208,10,13,77,123,135,72,149,222,162,230,6,53,63,154,
    139,174,167,32,235,207,128,236,35,218,183,158,130,108,153,193,
    85,197,167,110,116,205,212,0,121,114,92,191,2,46,66,150,
    245,22,100,253,148,122,230,172,200,243,4,85,42,232,239,11,
    160,34,222,140,162,60,135,216,153,108,147,24,69,56,109,99,
    154,127,30,110,99,230,54,56,115,255,130,51,55,103,127,174,
    135,116,112,54,57,62,235,78,137,244,209,51,97,43,205,200,
    73,5,91,148,230,116,210,136,122,13,197,2,83,44,61,120,
    144,236,63,72,62,194,40,217,120,194,241,73,199,73,29,9,
    99,57,166,72,70,159,62,61,237,74,78,133,252,230,56,58,
    112,57,28,196,156,52,197,34,178,54,73,155,70,166,102,14,
    225,137,138,41,114,223,188,162,171,185,162,137,239,207,104,167,
    42,107,217,20,219,136,162,170,96,118,28,29,174,185,204,226,
    89,124,62,37,205,147,200,18,168,64,182,219,154,89,150,131,
    36,178,127,126,9,41,55,41,133,221,66,178,127,204,16,82,
    158,34,132,30,51,67,252,223,129,171,79,1,127,3,194,0,
    154,58,69,124,238,32,100,244,117,90,254,39,96,215,120,75,
    230,231,24,211,166,108,207,43,48,244,36,143,121,169,46,4,
    62,131,127,20,252,42,75,215,102,90,95,22,211,181,149,199,
    39,6,207,247,74,201,214,229,64,70,150,25,184,9,45,211,
    209,105,234,170,211,248,159,87,134,24,157,111,20,73,139,122,
    15,135,216,249,102,138,35,74,120,59,98,221,40,160,227,151,
    212,188,151,3,67,100,99,55,197,217,30,92,159,154,29,29,
    255,191,166,237,45,102,120,101,129,69,201,8,228,152,47,101,
    152,127,47,199,188,228,52,245,134,79,26,212,26,100,231,11,
    67,224,241,15,235,51,58,109,89,32,75,208,41,147,119,112,
    41,45,82,231,17,89,232,162,64,119,41,7,178,58,14,181,
    162,114,83,107,43,82,115,122,243,33,129,164,63,8,220,209,
    145,231,62,9,104,31,218,172,155,185,147,145,113,94,47,114,
    78,174,32,174,99,158,95,31,101,18,188,188,249,112,240,1,
    146,205,57,103,240,123,81,151,99,192,179,129,108,140,228,232,
    8,143,146,3,127,220,232,5,110,159,109,98,166,146,125,158,
    73,166,216,168,87,235,137,228,33,181,81,163,27,133,24,161,
    79,186,42,138,27,158,196,35,150,244,26,239,54,56,188,55,
    252,164,225,30,225,172,219,85,26,218,151,93,147,75,86,55,
    238,39,92,157,30,191,162,238,124,108,234,224,201,217,199,34,
    61,132,60,149,234,83,93,30,173,185,252,214,158,130,123,226,
    225,73,77,116,132,162,218,194,222,167,230,103,48,183,160,254,
    62,233,135,232,147,162,202,98,199,168,24,204,96,182,230,11,
    250,34,153,245,207,127,126,31,255,212,23,51,169,151,150,105,
    165,92,160,51,59,181,21,10,235,157,106,54,184,196,109,141,
    7,151,179,193,91,220,174,240,96,61,187,16,90,229,193,53,
    232,172,211,141,9,141,108,144,231,47,252,191,158,207,206,51,
    31,183,57,249,65,29,222,126,60,127,198,237,15,33,77,248,
    215,57,187,40,74,181,172,157,125,40,178,243,69,81,36,190,
    193,216,156,193,156,211,141,165,171,164,182,205,238,60,68,228,
    96,161,119,61,157,186,239,108,37,252,73,46,205,5,23,57,
    147,13,54,153,62,84,177,201,196,243,240,46,150,196,22,151,
    196,7,84,18,159,177,232,142,161,171,226,41,16,75,185,6,
    232,102,37,148,175,156,203,90,208,69,47,49,230,142,199,50,
    244,236,135,80,172,99,121,250,230,109,79,161,233,91,40,148,
    27,166,216,192,194,117,214,223,40,238,22,164,99,219,149,114,
    15,155,139,21,25,168,127,205,128,218,228,184,155,7,95,251,
    128,26,14,183,121,164,181,127,157,219,224,246,44,10,143,78,
    122,201,107,58,32,93,63,137,5,15,95,141,240,155,218,158,
    93,230,157,140,198,76,226,186,57,162,192,170,195,151,183,45,
    226,171,182,107,8,240,92,70,128,94,216,90,236,80,158,12,
    164,146,87,16,165,72,250,244,104,238,73,204,161,209,4,79,
    75,124,0,193,247,192,113,230,148,121,62,214,94,4,9,1,
    31,51,143,40,99,238,217,196,35,245,166,81,41,87,4,39,
    243,43,55,208,154,41,170,52,117,177,61,97,222,192,94,201,
    13,200,87,165,89,86,37,91,243,161,240,208,29,233,123,46,
    190,198,177,127,2,233,177,219,126,39,7,2,221,70,240,9,
    71,159,34,209,17,185,208,224,186,194,254,21,141,83,111,244,
    104,63,147,104,191,32,209,239,200,108,6,207,95,179,172,237,
    143,244,173,160,90,189,50,239,197,46,246,55,175,140,38,50,
    246,221,192,127,45,213,189,235,183,77,41,146,204,217,52,147,
    159,93,178,148,193,70,191,115,214,191,20,114,25,14,177,236,
    251,9,210,97,34,133,229,105,36,250,56,211,194,21,0,22,
    63,157,15,118,116,245,172,143,246,79,72,182,228,83,108,232,
    206,175,178,82,65,28,81,124,50,241,88,189,44,44,179,86,
    175,88,181,165,138,85,89,48,249,178,102,25,143,73,85,171,
    178,84,19,217,191,61,68,91,213,216,91,171,136,239,0,176,
    142,243,159,
};

EmbeddedPython embedded_m5_internal_param_EtherTap(
    "m5/internal/param_EtherTap.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_EtherTap.py",
    "m5.internal.param_EtherTap",
    data_m5_internal_param_EtherTap,
    2259,
    6821);

} // anonymous namespace