#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_LinuxX86System[] = {
    120,156,197,88,221,115,19,201,17,239,217,149,100,75,182,177,
    193,152,79,115,22,119,1,116,28,216,28,96,160,234,8,9,
    95,87,197,85,240,57,43,82,248,124,87,217,172,181,99,121,
    101,105,87,181,59,6,235,202,188,196,212,37,15,73,85,30,
    146,167,84,229,45,15,249,111,242,31,37,221,61,187,171,177,
    44,3,149,228,20,35,13,173,153,158,158,238,233,95,247,244,
    76,3,210,191,34,126,127,94,5,72,254,102,1,248,248,17,
    208,6,232,8,88,23,32,164,0,255,36,108,23,33,190,13,
    126,17,222,2,172,91,32,45,216,71,194,134,111,45,8,39,
    121,78,9,218,54,247,8,232,85,64,22,96,189,8,47,195,
    227,80,144,37,216,174,64,252,27,16,66,132,2,214,252,49,
    240,199,225,45,74,71,162,204,2,199,129,58,43,220,89,6,
    127,130,59,43,224,79,50,49,1,189,25,144,147,176,62,69,
    108,235,199,80,236,85,20,59,205,98,255,73,98,125,28,153,
    3,255,24,177,163,94,223,16,103,129,56,121,189,105,150,50,
    147,105,121,28,214,79,100,244,172,65,159,52,232,57,131,62,
    101,208,167,13,250,140,65,159,53,232,115,6,125,222,160,231,
    13,250,130,65,127,100,208,11,6,93,53,232,139,6,253,177,
    65,127,98,208,63,49,232,75,6,125,217,160,175,24,116,205,
    160,63,53,232,171,6,253,153,65,95,51,232,235,6,189,200,
    52,122,232,4,180,150,160,117,3,90,159,195,38,130,230,120,
    238,141,155,32,109,104,221,130,245,91,32,241,115,19,246,17,
    87,254,9,99,198,109,158,49,155,207,88,230,25,119,96,253,
    14,72,252,44,235,25,37,168,215,78,33,86,131,127,225,95,
    77,32,165,38,177,121,37,227,36,136,66,55,8,55,163,192,
    162,241,18,53,132,236,6,53,99,41,196,31,19,196,255,1,
    140,111,223,74,33,254,6,80,176,32,91,218,22,188,97,226,
    141,5,189,26,236,9,104,21,192,183,97,15,151,41,146,2,
    77,1,251,22,124,103,19,195,27,108,11,8,196,143,160,160,
    52,190,91,12,68,45,105,12,222,20,97,175,8,245,181,61,
    139,58,182,203,16,255,29,190,159,103,161,227,44,212,130,61,
    108,11,176,95,128,55,37,120,137,76,216,213,42,147,249,98,
    109,15,45,197,158,122,173,128,218,174,24,230,146,41,126,16,
    135,94,71,170,57,164,221,174,23,123,29,247,23,65,184,179,
    187,118,239,78,189,151,40,217,169,85,50,214,40,89,236,122,
    106,203,225,185,54,109,74,167,171,88,102,20,74,53,129,196,
    102,16,250,110,39,242,119,218,82,141,147,64,119,51,104,75,
    215,229,193,103,157,110,20,171,167,113,28,197,14,237,43,119,
    182,35,47,159,65,187,218,104,71,137,172,209,106,188,140,67,
    226,21,113,111,118,89,34,41,192,250,210,100,95,38,141,56,
    232,42,116,151,150,72,220,36,173,70,142,226,38,89,195,102,
    105,43,234,200,165,168,29,160,111,119,123,189,165,110,28,53,
    209,208,165,166,236,44,95,79,148,183,209,150,75,27,59,65,
    219,95,66,171,151,186,61,181,21,133,75,157,229,165,32,84,
    18,119,167,189,52,108,95,22,145,239,4,173,240,58,104,186,
    1,219,230,110,201,118,87,198,83,212,123,142,86,23,51,98,
    82,148,132,45,106,98,10,169,34,126,109,49,111,77,136,149,
    128,172,107,144,197,132,174,130,137,39,114,178,128,109,11,226,
    121,66,75,11,63,130,220,139,152,169,211,152,197,99,191,164,
    109,209,189,45,155,48,160,59,247,24,97,8,53,228,188,79,
    78,15,129,97,82,132,86,9,52,124,16,117,26,79,113,143,
    90,100,39,49,22,10,47,64,242,23,192,109,70,224,236,65,
    10,170,125,27,68,56,3,170,66,57,16,123,79,225,130,191,
    101,92,214,107,164,254,10,67,67,109,5,73,244,58,100,7,
    16,205,145,84,199,157,89,237,125,189,209,146,13,149,44,96,
    199,55,209,78,181,225,133,97,164,170,158,239,87,61,165,226,
    96,99,71,201,164,170,162,234,165,164,70,62,117,142,103,232,
    202,229,245,186,25,154,200,243,136,38,253,195,15,26,10,127,
    204,242,15,246,66,34,21,34,99,43,242,19,236,39,17,77,
    169,28,82,82,209,38,71,172,8,3,199,37,86,90,30,249,
    142,225,239,135,153,38,140,206,90,41,195,82,34,219,155,170,
    194,176,244,146,196,101,77,168,159,17,72,130,95,121,237,29,
    201,210,17,70,10,21,34,82,235,48,42,12,158,33,123,50,
    243,217,166,48,10,253,30,170,24,52,174,208,234,103,24,137,
    147,140,197,57,196,225,24,182,37,252,191,36,78,89,141,66,
    138,190,82,134,64,202,136,10,216,255,34,133,0,162,113,31,
    179,79,205,226,244,193,102,113,108,126,76,20,77,118,230,169,
    185,64,205,71,212,44,100,150,143,192,252,169,65,243,239,210,
    146,22,219,204,214,145,155,236,204,58,255,64,124,157,237,199,
    23,38,202,58,197,137,69,209,212,143,147,2,37,213,248,1,
    181,200,202,17,104,67,242,130,82,56,197,19,11,163,208,193,
    32,32,170,31,26,188,87,206,12,237,193,120,134,106,135,160,
    106,226,181,105,224,213,33,55,49,88,157,179,89,114,116,137,
    67,195,212,57,79,162,138,67,54,187,74,205,197,17,238,120,
    31,112,205,67,128,251,130,86,159,73,1,55,197,64,171,224,
    119,198,106,216,169,27,242,163,115,118,0,104,132,178,194,16,
    148,93,38,202,62,108,248,232,1,150,154,251,165,1,48,210,
    208,50,173,90,65,162,119,154,140,49,161,117,26,75,129,151,
    225,105,60,221,45,62,221,111,240,233,206,21,2,215,148,58,
    105,219,156,183,53,81,164,93,217,180,225,84,122,106,39,101,
    108,209,166,221,94,53,218,172,42,54,155,114,236,253,75,201,
    226,165,228,11,204,158,213,7,156,183,116,254,212,25,50,150,
    93,202,112,52,245,233,110,67,242,65,201,191,92,87,39,52,
    151,147,155,155,30,192,136,50,170,2,216,3,188,217,156,218,
    19,21,83,70,31,213,118,87,242,237,38,237,191,162,245,42,
    188,215,182,56,141,136,170,8,86,202,213,201,156,11,50,30,
    197,239,35,218,127,50,92,2,93,53,156,186,86,153,173,33,
    187,156,107,7,80,243,227,219,226,44,161,240,95,101,104,41,
    245,209,66,95,59,139,129,223,1,87,171,2,126,0,194,3,
    186,61,141,129,60,100,8,0,179,196,254,107,224,96,25,82,
    29,112,238,169,83,69,192,28,152,146,146,187,204,170,139,133,
    175,224,247,70,164,101,71,186,157,214,163,230,145,94,200,243,
    22,3,233,131,142,237,194,193,4,71,254,217,242,18,98,211,
    89,171,31,188,253,211,33,175,33,49,107,143,0,85,227,122,
    37,151,148,250,174,143,41,58,20,207,139,89,203,64,202,231,
    212,220,204,65,34,178,190,31,87,191,5,56,250,16,119,245,
    25,241,45,41,81,96,181,167,199,184,106,57,40,38,143,133,
    98,22,11,55,243,88,144,124,160,189,229,187,10,181,22,121,
    126,223,18,120,193,198,170,142,238,179,5,144,69,88,47,81,
    212,112,25,46,210,160,18,89,98,163,52,120,224,180,228,173,
    89,209,155,150,59,95,251,149,154,221,81,37,12,114,237,253,
    182,215,217,240,189,7,111,105,53,90,178,145,133,153,149,233,
    63,99,234,79,33,34,142,50,129,127,46,103,118,188,26,85,
    178,184,3,236,33,173,63,135,134,31,53,56,67,188,216,146,
    213,142,236,108,224,197,116,43,232,86,55,219,94,147,253,99,
    167,246,125,157,217,167,216,193,131,85,72,114,149,218,168,218,
    136,66,204,229,59,13,21,197,85,95,226,85,77,250,213,235,
    85,62,8,170,65,82,245,54,112,212,107,40,13,249,131,129,
    203,69,175,23,55,19,174,111,183,95,19,57,74,255,186,120,
    27,15,176,216,255,1,242,163,87,223,17,243,188,206,1,161,
    35,8,87,198,75,152,234,233,44,70,21,137,179,72,205,167,
    48,226,244,127,27,248,253,1,18,218,180,146,56,111,149,45,
    117,242,80,220,174,210,236,228,112,244,110,124,72,244,234,135,
    177,52,134,75,196,41,199,232,77,128,218,50,29,3,244,238,
    85,76,223,189,168,83,191,123,149,184,231,24,69,251,216,127,
    27,237,28,42,163,12,146,63,254,79,131,220,185,251,255,82,
    223,185,7,105,9,112,84,128,11,211,182,41,29,224,45,145,
    221,68,76,195,248,245,227,252,17,216,114,27,177,244,148,212,
    222,154,31,157,185,156,44,244,218,127,234,7,238,225,154,249,
    97,110,217,62,151,64,189,147,236,68,125,21,99,39,138,151,
    225,57,44,158,11,92,60,223,167,226,121,143,183,193,181,116,
    253,220,7,104,49,223,13,186,166,132,242,181,59,108,71,116,
    145,76,234,121,221,174,12,125,231,42,152,117,47,15,143,10,
    19,148,160,254,12,70,73,98,139,147,88,232,30,142,73,202,
    193,134,165,236,205,98,30,133,35,244,43,195,248,175,25,140,
    107,211,96,38,98,231,62,53,156,122,243,172,235,252,44,247,
    202,39,71,97,84,222,187,121,195,101,157,232,158,245,65,124,
    88,42,241,117,161,223,197,155,201,113,224,203,182,84,114,168,
    243,21,41,151,222,192,125,137,135,94,212,195,139,16,223,42,
    240,119,219,117,71,122,72,252,20,133,255,129,86,161,3,12,
    15,9,81,194,99,98,206,42,151,202,130,207,223,129,39,104,
    173,22,229,44,93,61,247,18,135,115,201,116,190,195,252,74,
    154,29,129,228,12,190,241,173,120,29,253,184,197,175,54,14,
    109,47,223,172,157,43,185,167,232,217,129,175,44,250,138,136,
    177,195,181,1,151,2,206,45,234,167,242,189,179,188,152,217,
    180,168,109,66,115,158,162,7,94,208,30,240,83,109,103,153,
    235,89,147,51,229,121,26,170,184,231,190,146,84,131,188,91,
    26,115,178,39,15,179,212,131,142,126,84,84,199,7,198,253,
    216,67,122,110,160,55,145,113,224,181,131,239,229,17,242,114,
    127,168,11,71,141,63,124,188,250,204,169,63,89,125,47,199,
    11,117,249,176,233,52,134,43,60,145,73,131,119,41,219,129,
    75,239,18,102,78,120,247,178,107,180,236,103,71,113,60,195,
    158,246,243,213,47,219,145,167,130,176,185,26,49,139,186,242,
    30,254,199,81,184,25,52,245,234,215,14,155,148,114,61,242,
    18,169,57,15,184,246,125,202,12,76,59,196,223,231,124,186,
    171,134,200,191,250,30,249,7,103,29,114,73,223,237,207,31,
    5,81,162,91,109,234,16,77,76,166,58,23,208,59,177,124,
    175,38,67,103,241,161,52,4,208,26,125,131,232,148,225,78,
    199,125,46,59,81,220,123,30,249,82,205,15,140,63,244,253,
    216,241,194,102,174,204,197,65,134,180,164,215,50,50,174,234,
    80,29,14,242,30,17,41,56,152,70,222,96,128,235,241,199,
    237,168,177,45,253,148,103,56,104,153,231,73,212,161,72,165,
    92,149,13,43,74,97,131,1,73,153,40,255,197,5,245,144,
    42,135,83,121,44,155,65,66,192,158,54,167,164,199,61,37,
    89,222,157,161,71,138,57,125,148,185,95,95,93,245,219,219,
    3,122,239,77,98,108,232,153,190,60,93,198,115,128,74,1,
    91,84,176,24,40,216,147,51,229,194,228,68,185,80,30,179,
    249,77,117,74,204,90,149,66,121,98,82,252,167,255,22,240,
    132,169,88,11,51,101,241,111,230,227,118,160,
};

EmbeddedPython embedded_m5_internal_param_LinuxX86System(
    "m5/internal/param_LinuxX86System.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_LinuxX86System.py",
    "m5.internal.param_LinuxX86System",
    data_m5_internal_param_LinuxX86System,
    2540,
    8122);

} // anonymous namespace
