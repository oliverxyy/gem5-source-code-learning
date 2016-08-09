#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BaseTLB[] = {
    120,156,189,88,255,111,219,198,21,127,71,82,180,37,203,177,
    18,127,75,98,175,102,55,100,213,130,213,218,186,186,41,80,
    207,88,179,117,192,138,193,109,169,12,73,213,98,28,45,158,
    36,202,20,41,144,231,36,42,228,95,230,96,219,63,176,63,
    97,63,236,191,217,127,180,189,247,142,164,36,39,1,10,116,
    178,35,94,142,199,187,119,239,203,231,125,185,235,66,254,87,
    193,231,55,14,64,214,19,0,1,254,4,68,0,35,1,29,
    1,66,10,8,182,224,188,2,233,135,16,84,224,21,64,199,
    0,105,192,21,118,76,248,198,128,184,206,107,108,136,76,30,
    17,48,169,129,180,160,83,129,167,241,109,176,164,13,231,53,
    72,255,2,66,136,88,192,179,96,5,130,85,120,133,212,177,
    83,101,130,171,64,131,53,30,172,66,176,198,131,53,8,234,
    220,89,131,73,3,100,29,58,235,52,173,115,11,201,62,68,
    178,27,76,246,63,68,54,192,47,219,16,220,162,233,200,215,
    215,52,211,162,153,188,223,6,83,105,20,92,222,134,206,157,
    162,191,57,215,223,226,62,238,116,7,134,219,48,220,129,225,
    46,160,66,130,219,37,213,187,32,77,24,222,131,206,61,144,
    248,187,11,87,168,159,224,206,220,138,251,188,98,179,92,177,
    199,43,246,161,179,15,18,127,123,122,133,13,237,230,14,234,
    60,252,47,254,53,81,231,160,234,216,60,151,105,22,38,177,
    23,198,189,36,52,232,187,77,13,89,168,75,205,74,110,170,
    223,146,169,254,13,108,167,192,200,77,117,9,72,88,144,44,
    145,1,151,220,185,52,96,210,132,169,128,161,5,129,9,83,
    220,166,66,12,244,5,92,25,240,173,73,19,46,177,181,80,
    161,239,128,165,180,157,134,172,80,77,105,5,46,43,48,173,
    64,251,217,212,160,129,243,42,164,255,130,239,246,153,232,42,
    19,53,96,138,173,5,87,22,92,218,240,20,39,225,208,176,
    74,226,139,103,83,148,20,71,218,77,11,185,61,157,19,151,
    68,9,194,52,246,71,82,221,194,190,55,246,83,127,228,61,
    246,51,249,228,143,143,155,181,98,78,146,29,142,125,53,112,
    121,145,73,218,24,141,21,19,75,98,169,214,176,211,11,227,
    192,27,37,193,69,36,213,42,81,242,122,97,36,61,143,63,
    254,97,52,78,82,245,89,154,38,169,75,10,229,193,40,241,
    203,21,164,206,110,148,100,178,73,187,241,54,46,145,87,52,
    187,55,102,138,196,0,51,74,139,3,153,117,211,112,172,208,
    78,154,34,205,38,106,77,178,16,55,217,87,216,180,6,201,
    72,182,146,40,68,163,190,156,76,90,227,52,233,163,132,173,
    190,28,29,189,159,41,255,44,146,173,179,139,48,10,90,207,
    62,254,168,53,158,168,65,18,183,70,71,173,48,86,18,213,
    18,181,22,20,114,136,19,238,16,233,23,97,223,11,89,40,
    111,32,163,177,76,215,105,244,62,109,43,26,162,46,108,97,
    138,166,88,199,94,5,31,83,236,27,107,226,52,36,177,186,
    36,42,225,201,154,71,16,153,85,192,185,1,233,62,225,99,
    136,63,65,6,69,148,180,233,155,193,223,190,34,125,232,209,
    161,73,86,215,131,83,198,20,130,11,103,30,147,153,99,96,
    96,84,96,104,131,6,12,226,76,35,40,157,80,139,211,137,
    140,129,196,45,200,254,9,168,95,132,202,20,114,24,93,153,
    32,226,6,168,26,121,47,142,238,224,134,127,101,36,182,155,
    196,254,41,99,66,13,194,44,121,17,179,230,169,207,190,211,
    70,205,124,57,249,226,108,40,187,42,59,192,129,175,147,11,
    167,235,199,113,162,28,63,8,28,95,169,52,60,187,80,50,
    115,84,226,60,200,154,100,76,247,118,1,171,146,222,100,92,
    192,136,76,142,48,210,47,65,216,85,248,178,201,47,108,133,
    76,42,132,196,32,9,50,28,39,18,125,169,92,98,82,145,
    146,19,102,132,17,227,209,84,218,30,231,17,212,63,45,56,
    97,88,54,237,2,68,153,140,122,170,198,120,244,179,204,99,
    78,104,156,161,71,132,159,251,209,133,100,234,136,31,133,12,
    81,87,243,176,116,240,221,37,65,10,185,89,152,56,137,131,
    9,242,22,118,223,163,109,239,50,4,235,12,194,109,4,224,
    10,182,54,254,111,139,29,163,107,229,176,179,11,232,81,240,
    83,192,134,23,185,237,17,134,87,24,104,154,6,71,10,150,
    135,189,241,199,212,163,197,238,62,53,63,162,230,29,106,14,
    10,145,151,41,247,250,117,185,31,209,94,6,11,203,98,145,
    97,204,66,172,96,193,163,238,205,60,10,131,97,155,60,195,
    32,255,153,121,134,69,129,51,61,161,22,167,178,207,153,144,
    61,161,48,77,30,196,196,200,89,16,246,212,155,57,3,43,
    201,109,144,240,171,5,142,93,2,231,60,66,251,115,8,117,
    201,62,12,79,247,94,17,7,61,154,161,129,233,238,17,169,
    202,27,180,236,80,243,238,77,168,122,6,177,254,107,16,251,
    132,182,109,228,16,91,103,104,213,240,105,24,93,51,215,127,
    153,23,55,175,65,139,112,101,189,1,87,63,165,158,249,186,
    196,55,8,169,92,206,223,207,65,138,88,51,230,197,57,197,
    206,100,151,164,152,7,211,46,38,248,167,241,46,230,108,131,
    115,246,47,56,103,115,222,231,138,71,7,102,147,99,179,238,
    84,72,29,61,19,118,242,92,156,85,177,69,97,94,78,156,
    164,231,40,150,151,226,232,241,131,236,240,65,246,9,70,72,
    231,132,99,147,142,145,58,10,166,114,76,81,140,150,126,246,
    178,43,57,11,242,155,231,233,160,229,113,0,243,242,236,138,
    184,218,38,101,26,133,150,57,124,103,42,165,168,189,116,61,
    215,74,61,19,219,159,211,70,53,86,178,41,118,17,67,53,
    193,220,120,58,82,115,125,197,95,241,121,76,138,39,137,37,
    80,5,236,182,53,175,44,6,9,228,254,124,1,39,75,20,
    194,109,33,213,63,21,248,176,103,248,160,199,44,224,254,119,
    224,170,83,192,223,128,16,128,134,206,225,94,122,7,153,124,
    147,166,255,25,216,47,222,144,243,57,190,180,41,207,243,12,
    12,59,217,35,158,170,75,128,207,225,31,115,78,85,36,106,
    51,175,43,231,19,181,85,198,38,134,206,247,74,198,214,98,
    16,35,195,12,252,140,166,233,200,52,243,211,89,232,47,75,
    66,140,204,203,196,209,170,222,194,35,110,190,157,161,136,82,
    221,158,216,52,230,176,241,75,106,62,40,97,33,138,177,37,
    49,118,0,111,207,201,158,142,252,223,208,238,22,243,187,177,
    194,106,205,215,151,120,175,20,120,255,160,196,187,228,252,244,
    138,143,23,212,26,100,228,43,67,224,217,14,203,50,58,74,
    89,32,43,208,177,201,51,184,128,22,185,227,136,34,106,81,
    140,91,72,126,172,140,83,173,166,210,206,218,132,212,188,92,
    122,52,32,43,30,71,254,232,44,240,79,134,180,13,237,213,
    45,92,201,40,24,111,204,51,78,110,32,222,198,59,191,30,
    21,2,60,95,122,36,248,8,152,45,205,56,227,62,72,186,
    236,254,79,6,210,25,201,209,25,158,30,7,225,216,233,69,
    126,159,45,98,230,130,125,81,8,166,216,164,215,203,136,236,
    33,181,137,211,77,98,12,205,23,93,149,164,78,32,241,88,
    37,3,231,125,135,227,186,19,102,142,127,134,95,253,174,210,
    176,94,244,74,174,83,253,180,159,113,73,122,254,130,186,55,
    98,81,15,207,202,33,22,230,17,148,41,84,31,228,202,48,
    173,79,151,236,37,184,37,30,152,212,68,199,38,42,41,220,
    67,106,126,6,55,21,205,63,4,125,156,207,72,77,182,216,
    51,170,6,151,3,249,148,47,105,126,246,186,107,254,238,251,
    184,166,190,112,201,29,212,6,185,66,7,116,186,76,169,228,
    151,41,232,172,246,15,117,86,6,252,141,64,61,251,191,250,
    168,251,232,198,249,118,63,134,60,61,191,205,63,23,234,188,
    79,181,127,106,11,96,18,159,108,177,140,250,192,192,50,138,
    167,241,125,44,248,44,46,248,142,169,224,155,114,81,232,25,
    186,230,155,25,142,235,120,190,156,32,85,196,242,133,183,0,
    48,93,209,17,22,252,241,88,198,129,251,16,230,139,52,254,
    188,116,93,145,251,61,135,185,108,106,138,45,172,202,94,71,
    39,133,150,57,217,24,133,149,18,143,251,55,101,215,203,194,
    174,250,226,160,140,47,238,49,53,141,133,96,162,99,11,27,
    96,139,184,151,145,84,114,209,6,138,86,228,231,180,64,98,
    100,77,38,88,60,115,65,138,239,145,231,221,76,64,250,53,
    82,77,11,43,96,64,18,182,81,181,171,130,35,251,181,27,
    72,205,11,29,147,116,209,53,201,92,134,249,70,41,43,223,
    149,21,49,150,212,194,71,131,83,127,164,111,58,248,36,239,
    254,4,242,179,151,251,94,169,51,58,145,114,165,171,207,18,
    8,88,206,58,156,100,220,95,209,56,245,70,71,135,133,32,
    135,90,144,118,56,210,23,62,124,91,55,58,98,184,207,79,
    11,82,31,251,219,215,70,51,153,134,126,20,126,167,47,122,
    138,97,69,130,92,167,75,252,148,111,28,190,217,162,185,6,
    217,124,169,236,135,25,146,224,245,229,220,220,211,72,191,234,
    62,92,139,240,11,235,110,196,208,186,252,209,167,178,19,2,
    112,118,130,13,221,213,84,55,170,104,116,114,63,19,143,68,
    235,194,50,235,141,170,85,95,171,90,213,21,147,143,217,235,
    88,228,214,172,234,90,93,208,191,3,132,71,205,56,168,87,
    197,255,0,85,185,66,230,
};

EmbeddedPython embedded_m5_internal_param_BaseTLB(
    "m5/internal/param_BaseTLB.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_BaseTLB.py",
    "m5.internal.param_BaseTLB",
    data_m5_internal_param_BaseTLB,
    2119,
    6206);

} // anonymous namespace
