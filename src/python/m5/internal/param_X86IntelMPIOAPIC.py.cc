#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_X86IntelMPIOAPIC[] = {
    120,156,197,89,109,115,219,198,17,222,3,64,74,164,40,139,
    122,183,45,57,98,210,113,195,186,137,152,166,81,156,153,168,
    110,28,143,59,227,76,43,185,96,58,118,152,76,81,136,56,
    82,160,72,128,3,156,44,51,35,245,67,229,190,252,129,254,
    128,126,232,135,254,155,252,163,118,119,15,0,161,55,203,51,
    77,24,137,60,29,22,119,123,251,242,236,222,222,169,13,201,
    79,1,191,159,213,0,226,72,0,120,248,17,208,7,24,8,
    104,9,16,82,128,183,4,7,5,136,62,2,175,0,175,0,
    90,6,72,3,78,177,99,194,215,6,4,21,158,83,132,190,
    201,20,1,163,50,72,11,90,5,120,22,204,131,37,139,112,
    80,134,232,79,32,132,8,4,60,247,166,192,155,134,87,200,
    29,59,37,102,56,13,68,44,51,177,4,222,12,19,203,224,
    85,184,51,3,163,42,200,10,180,102,105,88,235,6,178,189,
    135,108,231,152,237,119,196,214,195,55,203,224,221,160,225,40,
    215,87,52,210,162,145,188,222,28,115,169,166,82,206,67,107,
    33,237,47,230,250,75,185,254,50,247,113,213,5,232,173,64,
    111,21,122,55,161,131,134,152,207,86,184,5,210,132,222,109,
    104,221,6,137,159,91,112,138,182,242,22,114,51,214,120,198,
    98,54,99,157,103,220,129,214,29,144,248,89,215,51,138,208,
    172,175,160,253,253,255,226,79,29,237,15,170,130,205,11,25,
    197,126,24,56,126,208,9,125,131,222,23,169,33,111,181,169,
    153,74,220,246,136,220,246,31,96,159,121,70,226,182,19,64,
    198,130,116,233,27,112,194,157,19,3,70,117,56,22,208,179,
    192,51,225,24,151,41,144,0,93,1,167,6,124,99,210,128,
    19,108,45,52,238,91,96,41,237,179,30,27,87,115,154,130,
    147,2,28,23,160,249,252,216,32,194,65,9,162,127,195,183,
    235,204,116,154,153,26,112,140,173,5,167,22,156,20,225,25,
    14,66,82,175,68,234,139,231,199,168,41,82,154,117,11,165,
    221,201,169,75,170,120,126,20,184,3,169,86,177,239,12,221,
    200,29,56,207,63,249,248,73,160,100,255,119,79,159,236,62,
    124,250,228,81,189,156,14,14,227,205,161,171,246,109,158,109,
    146,89,6,67,197,92,195,64,170,25,236,116,252,192,115,6,
    161,119,216,151,106,154,88,58,29,191,47,29,135,95,62,25,
    12,195,72,61,142,162,48,178,201,178,76,236,135,110,54,131,
    236,218,238,135,177,172,211,106,188,140,77,236,21,141,238,12,
    153,35,9,192,18,211,100,79,198,237,200,31,42,116,152,230,
    72,163,137,91,157,92,197,77,220,194,166,177,31,14,100,35,
    236,251,232,221,151,163,81,99,24,133,93,84,181,209,149,131,
    173,247,99,229,238,245,101,99,239,208,239,123,13,84,190,49,
    28,169,253,48,104,12,182,26,62,218,1,237,211,111,92,110,
    153,77,28,185,64,107,28,249,93,199,103,237,156,125,217,31,
    202,104,150,168,183,105,125,81,21,21,81,20,166,168,139,89,
    236,21,240,107,138,117,99,70,236,248,164,95,155,116,38,132,
    89,121,76,145,163,5,28,24,16,173,19,98,122,248,17,228,
    98,196,77,147,222,25,252,238,247,100,24,77,237,153,132,3,
    77,60,102,148,33,220,112,228,54,57,62,0,134,74,1,122,
    69,208,16,66,228,105,76,69,35,106,113,56,177,49,144,185,
    5,241,63,1,13,141,224,57,134,4,88,167,38,136,160,10,
    170,76,177,141,212,21,92,240,47,140,205,102,157,196,223,97,
    112,168,125,63,14,143,2,118,1,245,57,154,154,104,153,167,
    163,221,189,158,108,171,120,3,9,95,133,135,181,182,27,4,
    161,170,185,158,87,115,149,138,252,189,67,37,227,154,10,107,
    119,227,58,121,213,158,79,241,149,241,27,13,83,60,145,239,
    17,79,250,193,243,219,10,31,22,249,129,189,16,75,133,216,
    216,15,189,24,233,196,162,43,149,77,66,42,50,114,200,130,
    48,116,28,26,74,203,227,184,27,248,252,48,149,132,241,89,
    47,166,104,138,101,191,163,202,12,76,55,142,29,150,132,232,
    140,65,98,252,194,237,31,74,230,142,64,82,40,16,117,181,
    12,147,67,225,77,210,40,53,0,107,21,132,129,55,66,33,
    253,246,187,180,254,77,198,98,133,209,184,140,72,156,194,182,
    136,127,139,98,197,104,91,9,254,138,41,6,41,47,42,96,
    4,136,4,4,136,199,83,204,65,117,131,147,8,43,198,241,
    249,14,245,104,178,189,78,205,29,106,222,162,102,35,213,125,
    34,6,152,61,111,128,251,180,168,193,90,179,126,228,42,51,
    213,207,59,19,99,183,198,49,134,9,179,73,177,98,80,68,
    141,99,197,162,228,26,61,160,22,135,114,20,154,16,127,73,
    169,156,98,138,153,81,248,96,32,80,111,28,30,108,45,187,
    74,86,152,78,145,109,19,92,243,152,237,230,48,107,147,163,
    24,176,246,173,52,69,58,52,66,67,213,94,35,86,133,75,
    204,93,163,230,237,137,218,124,12,186,238,5,208,125,74,235,
    87,19,208,205,50,216,202,248,173,26,109,51,113,68,182,137,
    46,158,3,27,33,205,186,4,105,63,165,158,121,81,245,31,
    3,100,137,194,191,201,129,140,100,52,242,122,237,96,103,180,
    74,234,228,225,181,138,101,193,179,96,21,119,122,131,119,250,
    15,120,167,231,106,129,107,38,157,188,77,206,223,186,83,32,
    187,116,76,88,73,118,240,184,132,45,106,245,114,84,11,59,
    53,197,138,83,174,221,190,27,111,222,141,63,197,44,90,123,
    192,249,75,231,81,157,41,35,57,164,76,71,83,31,191,108,
    75,222,50,249,201,113,116,98,115,56,201,57,201,86,140,72,
    91,38,171,26,169,185,57,197,199,42,162,204,62,57,131,151,
    51,131,147,252,95,208,138,101,182,182,41,86,17,85,101,193,
    98,57,58,173,115,121,198,111,241,251,57,121,128,84,151,64,
    197,180,221,212,66,179,62,164,153,253,222,25,228,76,66,27,
    187,129,236,255,144,34,166,56,70,12,125,205,52,18,254,14,
    92,189,10,248,27,16,38,208,245,73,36,100,129,67,32,88,
    164,225,127,4,14,153,75,42,5,206,65,77,170,14,120,4,
    166,166,248,62,15,213,133,195,23,240,143,92,188,165,219,187,
    153,212,167,249,237,221,202,242,23,131,233,141,182,112,235,108,
    162,35,15,237,187,49,13,211,217,107,28,194,227,125,34,171,
    40,49,123,79,4,89,211,122,45,135,196,250,102,140,43,218,
    32,215,196,162,145,67,203,47,168,249,48,3,138,72,105,63,
    180,132,27,112,245,150,238,232,253,226,107,18,195,98,193,231,
    166,212,79,240,47,49,106,62,116,30,237,254,118,119,167,233,
    36,60,179,71,230,157,133,73,33,13,147,15,179,48,145,188,
    227,189,226,67,13,181,6,65,226,212,16,120,186,196,210,143,
    14,115,22,200,2,180,138,20,80,92,173,139,36,222,68,154,
    245,40,71,158,217,78,217,98,59,218,150,25,42,180,195,169,
    121,57,185,108,66,62,223,238,187,131,61,207,125,112,64,235,
    209,162,237,52,2,141,84,131,106,94,3,138,30,113,149,18,
    252,184,149,106,242,98,114,153,228,99,100,159,105,192,113,227,
    133,109,78,31,95,238,203,218,64,14,246,240,20,187,239,15,
    107,157,190,219,101,31,153,137,134,187,169,134,138,157,124,190,
    84,137,239,81,27,214,218,97,128,201,254,176,173,194,168,230,
    73,60,213,73,175,246,126,141,119,138,154,31,215,220,61,124,
    235,182,149,142,134,179,81,205,213,177,27,117,99,46,132,15,
    142,168,59,89,31,59,120,120,247,241,92,48,128,108,119,214,
    7,202,44,241,115,197,175,131,11,215,198,243,154,26,233,36,
    71,101,139,189,73,205,207,96,226,251,195,71,192,168,131,152,
    12,87,20,107,70,201,80,43,73,80,231,199,62,37,14,241,
    197,56,254,215,155,196,177,190,31,74,162,185,72,35,229,20,
    93,35,80,91,162,157,162,85,78,137,51,220,86,152,56,155,
    18,111,112,59,199,196,106,74,156,231,118,129,137,139,233,101,
    213,18,19,151,161,181,66,55,59,68,89,165,180,49,245,255,
    166,13,142,184,201,198,154,250,94,179,133,125,255,199,83,192,
    254,4,146,66,227,170,76,33,242,218,205,234,185,61,145,158,
    123,242,170,241,141,203,157,43,1,234,180,35,233,42,169,125,
    182,62,73,149,57,243,232,213,143,198,57,224,98,133,254,48,
    211,238,148,139,173,209,18,187,82,31,254,216,149,226,89,112,
    27,75,117,139,75,245,109,42,213,143,217,20,142,161,171,245,
    49,80,11,153,69,232,218,39,144,71,23,4,211,86,209,69,
    57,9,232,14,135,50,240,236,123,144,175,179,249,245,228,176,
    65,249,238,207,144,43,127,76,177,132,133,245,197,248,164,164,
    158,211,150,125,90,200,34,114,162,222,101,64,255,53,5,116,
    157,15,204,89,102,183,183,169,225,92,158,165,113,251,215,153,
    111,222,185,26,173,88,203,70,18,15,66,120,180,123,147,97,
    88,148,113,105,155,60,171,183,175,158,34,3,210,150,25,95,
    63,138,248,18,58,244,227,235,194,203,247,152,229,235,71,16,
    59,190,197,246,94,167,84,122,233,125,141,238,233,176,84,247,
    228,153,81,196,114,120,18,69,150,87,32,95,145,87,146,43,
    15,79,98,1,17,142,240,212,201,7,56,124,238,59,206,132,
    183,219,95,37,16,138,151,128,183,91,81,196,13,119,89,240,
    175,81,42,150,4,215,52,231,254,7,160,197,163,131,139,62,
    174,140,98,155,211,234,92,6,49,190,164,78,139,10,66,35,
    31,179,119,220,129,190,89,228,11,51,155,106,117,190,208,176,
    223,205,160,74,247,61,124,70,212,231,114,76,33,92,111,113,
    121,101,255,146,232,63,199,102,176,181,153,234,182,121,94,183,
    207,221,88,62,10,131,142,223,125,28,168,104,196,142,31,108,
    169,181,75,167,53,253,129,190,151,85,243,231,222,123,145,139,
    253,229,115,212,88,70,190,219,247,191,213,23,177,41,153,143,
    41,215,201,241,30,92,115,50,57,55,129,43,33,245,193,117,
    179,180,43,25,80,145,236,250,49,74,196,226,92,41,72,146,
    129,201,241,217,105,233,18,152,231,25,78,22,147,250,136,162,
    175,96,30,208,213,95,252,25,54,116,103,91,154,43,33,62,
    41,63,155,162,140,25,218,50,43,213,146,85,153,41,89,165,
    41,147,47,215,102,241,216,90,182,74,51,21,161,127,55,16,
    193,101,99,99,177,36,254,7,146,27,115,75,
};

EmbeddedPython embedded_m5_internal_param_X86IntelMPIOAPIC(
    "m5/internal/param_X86IntelMPIOAPIC.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_X86IntelMPIOAPIC.py",
    "m5.internal.param_X86IntelMPIOAPIC",
    data_m5_internal_param_X86IntelMPIOAPIC,
    2332,
    7279);

} // anonymous namespace
