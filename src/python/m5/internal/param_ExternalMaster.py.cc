#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_ExternalMaster[] = {
    120,156,197,88,109,111,227,198,17,158,37,41,217,146,229,179,
    252,126,119,246,157,149,20,215,168,135,198,106,147,56,23,32,
    215,107,211,228,10,52,64,156,148,74,113,142,18,148,165,201,
    181,76,153,34,5,114,125,103,5,54,10,212,135,180,40,250,
    181,63,161,31,250,111,250,143,218,153,89,146,162,223,154,67,
    218,168,150,180,94,14,119,103,231,229,153,217,217,245,32,251,
    171,224,239,23,45,128,244,47,2,192,199,175,128,16,96,40,
    160,39,64,72,1,254,10,28,85,32,121,7,252,10,188,4,
    232,25,32,13,56,199,142,9,95,26,16,53,120,78,21,66,
    147,41,2,198,117,144,22,244,42,240,44,90,4,75,86,225,
    168,14,201,239,65,8,17,9,216,243,103,192,159,133,151,200,
    29,59,53,102,56,11,68,172,51,177,6,254,28,19,235,224,
    55,184,51,7,227,38,200,6,244,230,105,88,239,22,178,125,
    136,108,23,152,237,63,137,173,143,111,86,193,191,69,195,81,
    174,47,104,164,69,35,121,189,5,230,210,204,165,92,132,222,
    82,222,95,46,245,87,74,253,213,82,127,173,212,95,231,62,
    74,179,4,131,219,48,184,3,131,187,112,128,6,90,44,86,
    222,0,105,194,96,19,122,155,32,241,187,1,231,104,67,127,
    169,52,227,30,207,88,46,102,220,231,25,91,208,219,2,137,
    223,251,122,70,21,186,237,53,244,75,240,47,252,107,163,95,
    64,53,176,121,46,147,52,136,35,39,136,14,226,192,160,247,
    85,106,200,139,30,53,51,153,59,63,36,119,254,3,216,151,
    190,145,185,243,12,144,177,32,93,66,3,206,184,115,102,192,
    184,13,167,2,6,22,248,38,156,226,50,21,18,160,47,224,
    220,128,175,76,26,112,134,173,133,70,191,15,150,210,190,28,
    176,209,53,167,25,56,171,192,105,5,186,123,167,6,17,142,
    106,144,252,29,190,222,100,166,179,204,212,128,83,108,45,56,
    183,224,172,10,207,112,16,146,6,53,82,95,236,157,162,166,
    72,233,182,45,148,118,183,164,46,169,226,7,73,228,14,165,
    90,197,190,51,114,19,119,232,60,61,81,18,137,225,39,110,
    138,157,118,61,31,26,167,219,35,87,29,218,60,215,36,163,
    12,71,138,121,198,145,84,115,216,57,8,34,223,25,198,254,
    113,40,213,44,49,116,14,130,80,58,14,191,252,245,112,20,
    39,234,105,146,196,137,77,118,101,98,24,187,197,12,178,170,
    23,198,169,108,211,106,188,140,77,236,21,141,62,24,49,71,
    18,128,229,165,201,190,76,189,36,24,41,116,151,230,72,163,
    137,91,155,28,197,77,186,135,77,231,48,30,202,78,28,6,
    232,219,147,241,184,51,74,226,62,42,218,233,203,225,206,155,
    169,114,247,67,217,217,63,14,66,191,179,247,222,187,157,209,
    88,29,198,81,103,184,211,9,34,109,136,206,117,118,217,198,
    113,75,180,194,139,160,239,4,172,155,115,40,195,145,76,230,
    137,122,151,86,23,77,209,16,85,97,138,182,152,199,94,5,
    127,166,216,52,230,196,110,64,218,121,164,49,161,203,42,227,
    137,156,44,224,200,128,100,147,208,50,192,175,32,247,34,102,
    186,244,206,224,119,191,33,179,104,234,192,36,12,104,226,41,
    35,12,161,134,35,31,147,211,35,96,152,84,96,80,5,13,
    31,68,157,198,83,50,166,22,135,19,27,3,153,91,144,254,
    13,208,204,8,156,83,200,64,117,110,130,136,154,160,234,20,
    239,72,93,195,5,255,200,184,236,182,73,252,93,134,134,58,
    12,210,248,69,196,14,160,62,71,82,23,45,243,217,248,211,
    253,129,244,84,186,133,132,47,226,227,150,231,70,81,172,90,
    174,239,183,92,165,146,96,255,88,201,180,165,226,214,131,180,
    77,62,181,23,115,116,21,252,198,163,28,77,228,121,68,147,
    126,240,3,79,225,195,50,63,176,23,82,169,16,25,135,177,
    159,34,157,88,244,165,178,73,72,69,70,142,89,16,6,142,
    67,67,105,121,28,119,11,159,63,200,37,97,116,182,171,57,
    150,82,25,30,168,58,195,210,77,83,135,37,33,58,35,144,
    24,63,119,195,99,201,220,17,70,10,5,162,174,150,97,90,
    24,188,77,250,228,234,179,78,81,28,249,99,20,49,240,222,
    160,213,111,51,18,27,140,197,85,196,225,12,182,85,252,95,
    21,107,134,103,101,232,171,230,8,164,140,168,128,253,47,50,
    8,32,26,207,49,251,180,13,78,31,172,22,199,230,235,212,
    163,201,246,38,53,247,168,185,79,205,86,174,249,20,212,159,
    191,172,254,35,90,210,96,157,89,59,114,147,153,107,231,95,
    136,175,59,147,248,194,68,217,165,56,49,40,154,38,113,98,
    81,82,77,158,80,139,67,57,2,77,72,63,167,20,78,241,
    196,204,40,116,48,8,168,55,9,13,182,149,221,36,27,204,
    230,168,182,9,170,101,188,246,75,120,181,201,77,12,86,251,
    78,158,28,29,26,161,97,106,111,16,171,202,53,198,110,81,
    243,218,20,45,62,1,92,255,10,224,222,167,213,155,25,224,
    230,25,104,117,252,53,13,207,204,220,80,108,157,203,151,128,
    70,40,179,174,65,217,15,169,103,94,85,124,250,0,203,212,
    253,85,9,96,36,161,81,214,106,23,59,227,117,82,166,12,
    173,117,44,5,158,69,235,184,187,27,188,187,255,132,119,119,
    174,16,184,126,210,73,219,228,188,173,59,21,178,202,129,9,
    107,217,174,157,214,176,69,157,78,198,173,248,160,165,88,109,
    202,177,143,31,164,219,15,210,247,49,123,182,158,112,222,210,
    249,83,103,200,68,142,40,195,209,212,167,39,158,228,141,146,
    159,28,71,39,52,135,147,155,147,109,192,136,50,170,2,216,
    3,108,108,78,237,169,74,40,163,79,203,220,245,194,220,36,
    253,199,180,94,157,109,109,138,117,68,84,93,176,80,142,78,
    230,92,144,241,91,252,253,146,236,79,138,75,160,178,218,238,
    106,145,89,27,210,203,254,241,5,212,124,255,186,216,29,100,
    254,219,28,45,213,9,90,232,103,230,49,240,39,224,106,85,
    192,55,64,120,64,183,103,49,80,132,12,1,96,153,134,255,
    14,56,88,174,169,14,56,247,116,169,34,224,17,152,146,210,
    71,60,84,23,11,31,195,159,75,145,150,111,233,102,86,143,
    150,183,116,171,200,91,12,164,87,218,182,173,139,9,142,252,
    115,232,166,52,76,103,173,73,240,78,118,135,162,134,196,172,
    61,5,84,205,234,149,28,18,234,171,9,166,104,83,220,16,
    203,70,9,41,63,165,230,173,2,36,34,167,125,191,242,109,
    193,205,155,184,163,247,136,47,73,8,139,197,94,152,225,170,
    229,34,155,34,22,42,121,44,188,85,196,130,228,13,237,37,
    159,85,168,53,200,243,231,134,192,195,36,86,117,116,118,179,
    64,86,160,87,165,168,225,50,92,100,65,37,242,196,70,105,
    240,194,110,201,166,217,213,70,43,156,175,253,74,205,201,180,
    18,6,185,246,113,232,14,247,125,247,201,144,86,163,37,189,
    60,204,140,92,254,102,89,126,10,17,113,147,10,252,184,147,
    235,241,124,90,201,226,93,100,94,200,207,161,225,199,30,103,
    136,207,15,101,107,40,135,251,120,48,61,12,70,173,131,208,
    237,179,127,204,76,191,79,115,253,20,59,248,114,21,146,62,
    164,54,110,121,113,132,185,252,216,83,113,210,242,37,30,213,
    164,223,122,179,197,27,65,43,72,91,238,62,190,117,61,165,
    33,127,49,112,185,232,117,147,126,202,245,237,209,11,234,78,
    211,191,14,158,198,3,44,246,99,40,182,94,125,70,44,242,
    58,7,132,142,32,92,25,15,97,106,172,179,24,85,36,246,
    54,53,63,130,41,167,255,119,144,121,72,171,144,209,170,98,
    195,168,25,106,229,74,220,126,70,179,211,171,209,251,215,87,
    137,94,125,9,148,197,112,149,70,202,25,186,19,160,182,70,
    219,64,175,158,19,231,184,109,48,113,62,39,222,226,118,129,
    137,205,252,242,105,145,137,75,208,91,166,27,25,162,172,80,
    94,152,249,111,243,2,7,213,52,195,233,249,255,52,29,216,
    143,254,95,226,219,239,65,86,44,220,148,10,68,89,183,121,
    157,10,6,34,63,179,148,21,227,123,146,141,27,80,232,120,
    137,116,149,212,222,218,156,158,186,156,86,244,218,227,73,136,
    95,173,174,63,40,52,59,231,98,105,188,194,78,212,135,54,
    118,162,120,22,221,197,50,219,226,50,251,49,149,217,167,108,
    6,199,208,149,246,4,160,149,194,26,116,160,137,228,11,231,
    58,139,232,114,154,196,115,71,35,25,249,246,67,40,87,200,
    252,122,90,152,160,84,246,7,40,21,47,166,88,193,146,248,
    106,76,82,182,46,105,202,222,172,20,81,56,69,191,50,140,
    191,201,97,220,230,108,93,164,108,251,49,53,156,164,139,252,
    108,255,188,240,202,235,55,97,148,111,218,124,87,185,116,32,
    123,149,97,88,82,241,169,167,160,124,203,36,186,230,121,5,
    222,60,236,2,111,190,31,122,251,63,78,226,6,119,226,8,
    79,47,116,203,236,197,199,145,226,197,190,203,60,90,253,110,
    190,250,181,67,24,52,28,239,190,12,165,146,215,130,92,145,
    19,178,59,9,95,98,25,16,143,241,104,200,231,44,124,14,
    29,103,170,219,230,207,144,249,49,173,66,183,171,184,109,138,
    42,110,156,171,2,63,70,173,90,19,92,149,92,186,152,215,
    162,181,32,63,83,140,83,155,243,230,66,129,38,190,59,206,
    11,3,2,30,123,109,215,29,234,43,63,190,203,178,127,0,
    217,125,131,253,70,129,74,186,140,225,131,156,62,56,99,158,
    224,138,137,11,36,251,237,220,184,195,157,237,92,175,109,173,
    215,39,114,168,111,62,249,246,122,184,195,37,254,213,97,31,
    134,177,119,36,253,108,232,189,155,199,124,20,15,93,164,95,
    191,88,55,200,23,91,188,244,222,79,104,214,234,37,106,42,
    147,192,13,131,175,245,189,106,78,86,100,174,203,210,147,214,
    197,19,151,52,215,236,30,12,157,68,246,3,122,96,54,197,
    148,44,141,146,83,213,107,55,65,188,60,125,154,88,211,135,
    7,125,251,241,132,110,220,210,143,176,161,139,210,218,66,13,
    113,71,41,214,20,117,76,178,150,217,104,214,172,198,92,205,
    170,205,152,124,171,53,143,231,198,186,85,155,107,136,201,103,
    11,241,89,55,182,150,106,226,223,66,98,126,225,
};

EmbeddedPython embedded_m5_internal_param_ExternalMaster(
    "m5/internal/param_ExternalMaster.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_ExternalMaster.py",
    "m5.internal.param_ExternalMaster",
    data_m5_internal_param_ExternalMaster,
    2301,
    7162);

} // anonymous namespace