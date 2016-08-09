#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_SouthBridge[] = {
    120,156,197,89,221,115,219,198,17,223,3,72,74,164,36,91,
    178,172,15,91,178,69,127,51,110,44,218,138,29,59,19,215,
    173,236,164,51,206,76,20,23,76,198,142,146,41,10,1,39,
    10,20,9,112,128,147,109,102,164,118,90,101,218,204,244,185,
    211,183,190,245,161,255,77,255,163,118,119,129,3,33,137,84,
    211,105,205,218,210,121,177,183,216,187,221,253,237,222,30,236,
    66,250,167,136,191,63,175,2,196,91,6,128,135,63,2,218,
    0,29,1,155,2,132,20,224,157,135,221,34,68,247,193,43,
    194,247,0,155,6,72,3,14,145,48,225,27,3,130,73,126,
    167,4,109,147,57,2,122,21,144,5,216,44,194,203,96,6,
    10,178,4,187,21,136,126,13,66,136,64,192,43,111,12,188,
    113,248,30,181,35,81,102,133,227,64,204,10,51,203,224,77,
    48,179,2,222,36,19,19,208,155,6,57,9,155,83,36,182,
    121,6,213,222,70,181,103,89,237,63,72,173,135,51,115,224,
    157,33,113,220,215,215,36,89,32,73,94,239,44,107,153,214,
    187,156,129,205,115,154,158,205,209,231,115,244,92,142,158,207,
    209,11,57,122,49,71,95,200,209,23,115,244,82,142,94,206,
    209,151,114,244,229,28,189,146,163,171,57,250,74,142,190,154,
    163,175,229,232,235,57,250,70,142,190,153,163,111,229,232,90,
    142,126,143,105,244,242,57,104,221,134,214,79,160,245,62,108,
    99,224,103,50,143,222,1,105,66,107,21,54,87,65,226,207,
    29,56,68,108,120,231,114,111,212,249,141,217,236,141,187,252,
    198,61,216,188,7,18,127,238,38,111,148,160,81,155,71,188,
    249,255,196,63,53,129,148,154,196,225,181,140,98,63,12,108,
    63,216,14,125,131,230,75,52,16,58,93,26,198,82,152,62,
    35,152,254,29,24,163,158,145,194,244,0,80,177,32,91,218,
    6,28,48,113,96,64,175,6,251,2,90,5,240,76,216,199,
    101,138,180,129,166,128,67,3,190,53,73,224,0,199,2,130,
    233,50,20,84,130,209,22,131,41,209,52,6,7,69,216,47,
    66,227,213,190,65,140,221,50,68,127,131,239,150,89,233,56,
    43,53,96,31,199,2,28,22,224,160,4,47,81,8,89,173,
    50,153,47,94,237,163,165,200,105,212,10,184,219,141,156,185,
    100,138,231,71,129,211,145,234,28,210,118,215,137,156,142,221,
    8,247,212,206,211,200,247,154,178,86,209,114,97,188,218,117,
    212,142,197,47,154,228,145,78,87,177,194,48,144,106,2,137,
    109,63,240,236,78,232,237,181,165,26,39,109,246,182,223,150,
    182,205,147,207,59,221,48,82,159,70,81,24,89,228,84,102,
    182,67,39,123,131,92,234,182,195,88,214,104,53,94,198,34,
    245,138,164,183,187,172,145,54,192,155,165,151,61,25,187,145,
    223,85,24,171,68,35,73,147,182,26,69,137,135,248,43,28,
    234,59,97,71,214,195,182,143,129,125,219,235,213,187,81,216,
    68,43,235,77,217,121,112,39,86,206,86,91,214,183,246,252,
    182,87,127,245,232,195,122,183,167,118,194,160,222,121,80,247,
    3,37,209,53,237,250,9,167,172,162,16,185,43,126,227,55,
    109,159,13,179,119,100,187,43,163,41,226,94,164,165,197,180,
    152,20,37,97,138,154,152,66,170,136,191,166,88,54,38,196,
    134,79,166,185,100,46,225,170,144,71,18,133,87,192,174,1,
    209,50,225,164,133,63,130,2,139,104,105,208,156,193,115,191,
    36,159,36,220,150,73,209,79,152,251,140,45,4,25,74,62,
    166,112,7,192,0,41,66,171,4,9,112,16,111,9,146,162,
    30,141,40,78,106,12,84,94,128,248,207,128,62,70,200,236,
    67,10,167,67,19,68,48,13,170,66,21,12,185,243,184,224,
    239,25,145,141,26,109,127,131,113,161,118,252,56,124,19,176,
    247,137,230,28,106,160,103,94,244,190,216,106,73,87,197,43,
    200,248,58,220,171,186,78,16,132,170,234,120,94,213,81,42,
    242,183,246,148,140,171,42,172,222,136,107,20,80,107,70,67,
    43,211,215,235,106,40,81,216,17,74,201,131,231,187,10,31,
    102,249,129,163,16,75,133,176,216,9,189,24,249,164,162,41,
    149,69,155,84,228,228,144,55,194,168,177,73,148,150,71,185,
    51,248,188,174,119,194,208,172,149,52,144,98,217,222,86,21,
    198,164,19,199,54,239,132,248,12,63,82,252,218,105,239,73,
    214,142,24,82,184,33,34,147,61,140,4,128,139,100,140,182,
    157,13,10,194,192,235,225,254,124,247,22,45,189,200,48,156,
    100,32,206,33,8,199,112,44,225,191,37,49,111,184,133,20,
    122,37,13,63,42,132,10,56,248,34,141,63,66,241,16,139,
    78,205,224,170,193,54,113,86,94,37,138,94,182,150,105,184,
    68,195,101,26,86,180,217,239,218,246,169,227,182,63,164,245,
    12,54,152,77,163,0,153,218,52,239,72,102,93,232,103,22,
    22,199,6,101,136,65,121,212,207,144,2,21,210,232,9,141,
    40,202,185,103,66,252,37,149,109,202,36,86,70,73,131,240,
    39,170,159,20,236,40,107,154,28,48,174,241,108,17,72,243,
    72,109,230,144,106,81,140,24,166,214,5,93,19,109,146,72,
    0,106,45,145,170,226,0,79,87,105,184,50,42,119,247,161,
    214,60,1,181,143,105,233,233,20,106,83,12,177,10,254,78,
    27,174,153,198,32,59,43,103,143,65,140,240,85,24,128,175,
    155,68,153,39,173,30,49,180,82,91,127,145,131,22,109,207,
    200,155,180,129,68,111,129,44,201,131,106,1,15,254,151,193,
    2,158,229,6,159,229,119,249,44,231,126,128,187,192,164,80,
    155,92,171,19,162,72,46,217,54,97,62,61,163,227,50,142,
    104,208,219,94,53,220,174,42,182,153,234,234,227,27,241,234,
    141,248,99,172,152,213,39,92,171,146,154,153,84,197,72,118,
    169,170,209,171,159,190,117,37,159,140,252,100,219,73,17,179,
    185,160,217,233,137,139,248,154,35,135,26,218,211,92,206,99,
    21,81,21,31,137,175,43,153,175,105,235,159,209,98,21,118,
    180,41,22,16,75,21,193,59,178,147,234,205,189,23,207,226,
    239,83,114,62,89,45,129,110,6,86,35,217,47,155,66,70,
    89,239,31,193,203,59,54,196,170,163,230,175,52,78,74,125,
    156,208,175,169,161,255,71,224,174,84,192,31,128,144,128,1,
    79,161,159,101,10,133,126,150,196,127,5,156,35,3,122,1,
    174,55,13,58,255,89,2,203,80,252,144,69,147,214,224,51,
    248,33,151,96,250,0,55,211,190,51,127,128,23,178,90,197,
    16,250,81,135,116,225,104,81,163,224,236,56,49,137,37,149,
    170,159,179,253,227,32,107,23,177,82,191,107,60,141,39,203,
    216,180,163,111,251,104,162,35,112,73,204,26,57,140,220,163,
    97,45,131,135,208,188,119,184,185,21,24,126,94,219,201,137,
    240,13,237,160,192,123,62,59,198,135,65,78,71,134,255,162,
    198,255,90,134,127,201,103,215,247,124,21,161,209,160,128,31,
    26,2,239,192,216,186,209,149,179,0,178,8,155,37,202,20,
    110,180,69,154,72,66,87,50,170,123,71,14,70,118,202,70,
    226,174,44,230,73,56,105,120,59,146,10,65,17,125,220,118,
    58,91,158,243,228,183,180,20,173,231,234,212,50,244,230,167,
    243,155,167,180,16,195,246,207,143,15,180,17,175,71,82,29,
    62,68,205,217,230,57,23,188,208,229,146,240,229,142,172,118,
    100,103,11,111,156,59,126,183,186,221,118,154,28,25,51,53,
    238,11,109,156,226,208,30,111,53,226,219,52,134,85,55,12,
    176,108,239,185,42,140,170,158,196,107,152,244,170,119,170,92,
    243,171,126,92,117,182,112,214,113,85,2,243,163,153,202,61,
    173,19,53,99,110,95,119,223,16,57,178,200,218,120,199,246,
    177,145,255,29,100,71,108,114,249,203,74,56,183,232,73,214,
    224,178,120,193,82,189,164,102,81,219,97,173,210,240,30,140,
    178,210,223,71,205,191,161,37,200,93,37,177,100,148,13,53,
    115,52,81,95,208,123,241,201,116,173,136,31,145,174,201,199,
    170,52,105,75,36,41,199,232,142,79,99,153,202,253,102,69,
    51,39,120,156,100,230,148,102,158,225,241,44,51,167,53,115,
    134,199,115,204,156,213,204,243,60,206,49,115,94,51,23,120,
    92,100,230,5,205,188,200,227,18,51,151,53,243,18,143,151,
    153,185,162,153,85,30,175,48,243,170,254,68,119,141,153,215,
    97,243,6,125,223,33,206,77,42,67,99,255,109,25,226,52,
    30,89,2,255,240,63,173,62,214,195,255,203,222,173,71,144,
    54,35,195,42,143,200,27,54,149,84,158,150,208,247,160,188,
    85,252,201,101,113,16,244,109,55,146,142,146,73,144,150,71,
    100,40,215,175,100,225,63,245,203,201,201,142,125,61,179,233,
    144,219,176,222,121,142,93,114,5,228,216,137,151,193,69,108,
    221,11,220,186,63,166,214,125,159,29,96,27,73,247,222,7,
    101,49,243,3,21,172,64,190,177,79,248,34,233,207,105,111,
    78,183,43,3,207,186,13,249,150,155,167,71,130,3,42,152,
    127,129,92,79,100,138,243,216,99,159,204,64,58,16,114,54,
    114,16,139,89,206,141,42,156,140,219,191,106,220,214,232,131,
    70,255,84,176,30,211,192,231,64,118,4,88,63,203,130,113,
    113,48,40,59,97,76,119,186,211,166,177,51,227,3,145,30,
    134,200,121,29,231,222,41,106,120,90,171,161,7,117,105,160,
    156,31,218,78,23,27,63,210,116,186,4,41,163,90,153,62,
    171,203,3,165,119,101,111,43,116,34,143,21,254,27,17,210,
    72,199,175,102,12,49,5,23,59,205,82,158,214,150,210,195,
    112,185,181,211,213,172,229,213,172,169,11,67,228,20,107,25,
    62,75,74,76,86,50,204,3,221,182,163,182,195,168,115,138,
    147,50,17,237,36,205,24,18,165,184,43,157,93,25,157,18,
    71,45,161,227,152,62,115,66,114,253,244,100,91,42,121,178,
    116,40,2,120,250,209,200,147,216,194,133,61,188,193,243,141,
    24,159,219,182,61,186,174,231,167,192,215,87,136,201,105,216,
    245,136,18,246,61,115,226,216,95,163,92,42,11,110,45,143,
    253,183,73,178,71,186,174,36,55,193,94,108,241,105,116,54,
    75,89,246,180,110,240,40,187,249,187,197,134,211,73,62,203,
    242,39,71,235,26,164,31,135,172,91,89,234,19,28,248,250,
    157,124,232,192,26,204,109,47,119,185,214,7,196,167,15,155,
    157,7,171,218,192,213,196,192,103,148,225,6,79,169,43,3,
    37,208,63,207,3,133,142,136,92,249,2,231,150,6,74,53,
    252,78,242,149,155,187,192,252,188,23,57,72,207,29,227,198,
    50,242,157,182,255,157,84,213,129,250,158,58,177,239,190,240,
    195,79,228,107,223,149,67,22,237,207,47,14,222,84,47,86,
    178,115,226,101,25,236,117,236,207,101,39,140,122,159,135,158,
    84,203,199,230,215,61,47,178,156,160,41,237,215,146,238,18,
    39,28,179,158,94,36,18,29,90,106,176,33,71,101,135,24,
    130,147,169,247,86,6,71,169,29,186,187,210,75,101,46,13,
    151,249,36,236,144,183,23,6,74,60,127,180,246,193,67,46,
    63,3,231,238,126,116,127,125,125,248,171,15,62,82,215,7,
    57,146,240,65,179,207,156,216,117,60,201,30,29,166,228,238,
    253,181,83,22,184,63,100,111,47,242,165,103,192,171,129,138,
    158,133,56,132,237,97,56,113,27,73,177,225,252,210,147,138,
    210,238,56,122,41,123,178,39,190,229,28,239,237,184,16,69,
    178,233,35,182,34,214,145,201,167,173,14,85,137,33,101,53,
    255,238,200,202,86,242,245,32,249,216,249,132,190,166,196,59,
    56,208,127,135,148,207,150,177,132,81,15,100,138,10,118,65,
    5,115,114,186,92,152,156,40,23,202,99,38,127,193,158,18,
    179,70,165,80,158,152,20,255,217,223,21,44,129,21,99,101,
    177,44,254,5,82,27,192,21,
};

EmbeddedPython embedded_m5_internal_param_SouthBridge(
    "m5/internal/param_SouthBridge.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_SouthBridge.py",
    "m5.internal.param_SouthBridge",
    data_m5_internal_param_SouthBridge,
    2648,
    8403);

} // anonymous namespace