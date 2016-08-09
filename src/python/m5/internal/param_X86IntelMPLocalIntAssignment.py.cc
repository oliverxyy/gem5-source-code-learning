#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_X86IntelMPLocalIntAssignment[] = {
    120,156,205,89,239,114,219,198,17,223,3,64,74,164,36,75,
    178,44,201,182,100,139,77,198,9,227,38,98,155,90,113,58,
    81,221,58,30,119,198,153,68,118,65,119,236,48,153,162,16,
    113,164,64,129,0,11,156,108,211,35,125,169,60,109,103,250,
    185,125,131,126,200,219,244,17,250,38,237,238,30,0,66,146,
    255,200,118,198,172,69,158,15,139,187,189,253,243,219,189,189,
    99,27,210,127,37,252,254,166,6,144,252,83,0,120,248,17,
    16,0,244,5,180,4,8,41,192,59,7,187,37,136,175,129,
    87,130,103,0,45,3,164,1,135,216,49,225,59,3,194,105,
    158,83,134,192,100,138,128,97,21,164,5,173,18,60,8,231,
    193,146,101,216,173,66,252,71,16,66,132,2,30,122,19,224,
    77,194,51,228,142,157,10,51,156,4,34,86,153,88,1,111,
    138,137,85,240,166,185,51,5,195,57,144,211,208,154,161,97,
    173,51,200,246,42,178,157,101,182,255,38,182,30,190,89,4,
    239,12,13,71,185,190,165,145,22,141,228,245,102,153,203,92,
    38,229,60,180,206,102,253,133,66,255,92,161,191,88,232,47,
    21,250,203,133,254,121,238,163,100,103,161,119,1,122,23,161,
    183,2,29,52,214,124,46,197,42,72,19,122,151,160,117,9,
    36,126,86,225,16,237,233,157,45,204,184,204,51,22,242,25,
    107,60,163,6,173,26,72,252,172,233,25,101,104,214,151,208,
    71,254,127,241,95,29,125,4,106,26,155,71,50,78,252,40,
    116,252,176,19,249,6,189,47,83,67,30,109,83,51,145,186,
    246,22,185,246,7,96,191,122,70,234,218,3,64,198,130,116,
    9,12,56,224,206,129,1,195,58,236,11,232,89,224,153,176,
    143,203,148,72,128,174,128,67,3,190,55,105,192,1,182,22,
    58,224,50,88,74,251,181,199,14,208,156,38,224,160,4,251,
    37,104,62,220,55,136,176,91,129,248,95,240,116,149,153,78,
    50,83,3,246,177,181,224,208,130,131,50,60,192,65,72,234,
    85,72,125,241,112,31,53,69,74,179,110,161,180,91,5,117,
    73,21,207,143,67,183,47,213,251,216,119,6,110,236,246,157,
    135,159,127,118,39,84,50,248,230,222,215,81,219,13,176,127,
    51,73,252,110,216,151,161,170,87,179,137,81,178,62,112,213,
    142,205,156,76,50,81,127,160,120,133,40,148,106,10,59,29,
    63,244,156,126,228,237,5,82,77,18,123,167,227,7,210,113,
    248,229,157,254,32,138,213,237,56,142,98,155,172,204,196,32,
    114,243,25,100,227,118,16,37,178,78,171,241,50,54,177,87,
    52,186,51,96,142,36,0,75,79,147,61,153,180,99,127,160,
    208,121,154,35,141,38,110,117,114,27,55,73,7,155,198,78,
    212,151,141,40,240,209,211,79,134,195,198,32,142,186,168,118,
    163,43,251,27,159,36,202,221,14,100,99,123,207,15,188,6,
    26,162,49,24,170,157,40,108,244,55,26,62,218,4,109,21,
    52,94,109,165,117,156,117,150,214,123,236,119,29,159,53,117,
    118,100,48,144,241,12,81,47,146,44,98,78,76,139,178,48,
    69,93,204,96,175,132,95,83,172,26,83,98,203,39,93,219,
    164,63,33,207,42,98,141,0,32,96,215,128,120,149,144,212,
    195,143,32,215,35,158,154,244,206,224,119,191,35,35,105,106,
    207,36,124,104,226,62,163,15,97,136,35,55,9,16,33,48,
    132,74,208,43,131,134,22,34,82,99,45,30,82,139,195,137,
    141,129,204,45,72,254,1,104,116,4,213,62,164,128,59,52,
    65,132,115,160,170,148,23,144,186,132,11,254,153,49,219,172,
    147,248,91,12,20,181,227,39,209,227,144,221,65,125,142,178,
    38,90,230,222,240,238,118,79,182,85,178,134,132,111,163,189,
    90,219,13,195,72,213,92,207,171,185,74,197,254,246,158,146,
    73,77,69,181,43,73,157,60,108,207,103,88,203,249,13,7,
    25,182,8,7,136,45,253,224,249,109,133,15,11,252,192,94,
    72,164,66,156,236,68,94,130,116,98,209,149,202,38,33,21,
    25,57,98,65,24,70,14,13,165,229,113,220,25,124,190,153,
    73,194,88,173,151,51,100,37,50,232,168,42,131,212,77,18,
    135,37,33,58,227,145,24,63,114,131,61,201,220,17,84,10,
    5,162,174,150,97,60,136,60,79,218,101,198,96,13,195,40,
    244,134,40,176,223,254,144,100,57,207,184,156,102,100,46,34,
    42,39,176,45,227,255,101,177,100,180,173,20,139,229,12,143,
    148,59,21,48,26,68,10,8,196,230,33,230,169,186,193,137,
    134,149,228,184,125,143,122,52,217,94,165,230,18,53,151,169,
    89,203,236,240,206,141,49,115,220,24,215,73,0,131,45,192,
    186,146,11,205,76,87,239,72,236,93,24,197,30,38,216,38,
    197,144,65,145,54,138,33,139,146,113,124,131,90,28,202,209,
    105,66,114,159,82,63,197,26,51,163,176,194,0,161,222,40,
    108,216,114,246,28,89,100,50,67,188,77,48,46,98,185,91,
    192,178,77,78,99,32,219,23,178,52,234,208,8,13,97,123,
    133,88,149,158,99,250,26,53,63,25,155,253,71,96,236,158,
    0,227,23,36,203,92,10,198,25,6,97,21,191,115,70,219,
    76,157,146,111,192,11,199,64,72,8,180,158,131,192,15,168,
    103,158,52,195,184,193,151,42,255,219,2,248,72,94,163,168,
    227,22,118,134,203,164,90,17,118,203,88,94,60,8,151,177,
    98,48,184,98,248,25,87,12,92,117,112,125,166,147,189,201,
    249,94,119,74,100,163,142,9,75,105,37,144,84,176,69,13,
    159,12,107,81,167,166,216,8,148,155,55,175,36,235,87,146,
    47,48,235,214,110,112,190,211,121,87,103,214,88,14,40,51,
    210,212,219,79,218,146,183,91,126,114,28,157,8,29,78,138,
    78,186,141,35,2,23,201,194,70,102,122,222,18,18,21,211,
    78,48,30,227,87,115,227,147,46,95,209,234,85,182,188,41,
    150,17,109,85,193,34,58,122,75,224,146,143,223,226,247,75,
    242,6,153,65,2,21,241,118,83,43,192,186,145,150,246,199,
    71,16,245,174,53,179,27,184,212,239,51,36,149,71,72,162,
    175,153,69,203,95,129,171,99,1,127,1,194,10,66,34,141,
    150,60,184,8,28,11,52,252,15,192,97,245,156,138,131,115,
    86,147,170,12,30,129,169,44,185,206,67,117,1,242,21,252,
    173,16,147,89,153,96,166,245,111,177,76,176,242,124,199,32,
    59,85,41,96,29,77,140,228,173,29,55,161,97,58,219,141,
    194,124,180,199,228,85,42,102,251,119,142,184,73,189,174,67,
    34,126,63,194,27,109,180,43,98,193,40,160,232,231,212,124,
    154,3,72,100,180,119,41,237,26,188,184,76,112,244,190,243,
    29,137,100,177,18,179,19,138,96,71,76,155,55,157,91,119,
    191,190,187,213,116,82,254,217,227,201,117,242,176,42,101,97,
    245,105,30,86,146,119,209,103,124,176,162,214,32,216,28,26,
    2,79,193,88,102,210,161,211,2,89,130,86,153,2,144,79,
    9,34,141,79,145,101,76,202,175,71,182,104,182,228,150,182,
    113,142,28,13,10,106,158,140,39,19,17,46,54,3,183,191,
    237,185,55,66,90,155,4,104,103,17,107,100,218,204,21,181,
    161,104,19,47,82,136,31,55,50,173,30,141,39,11,125,134,
    75,229,218,112,204,121,81,155,83,207,253,29,89,235,203,254,
    54,158,176,119,252,65,173,19,184,93,246,157,153,106,123,55,
    211,86,177,243,143,151,69,201,85,106,163,90,59,10,113,3,
    217,107,171,40,174,121,18,79,153,210,171,125,82,227,221,167,
    230,39,53,119,27,223,186,109,165,163,231,104,70,224,10,221,
    141,187,9,23,227,187,143,169,59,62,223,59,142,31,250,120,
    78,25,64,190,251,235,195,110,190,153,240,9,68,7,35,202,
    129,231,71,53,212,201,146,74,36,123,157,154,143,96,172,123,
    206,53,92,170,79,107,146,65,203,98,197,168,24,234,189,52,
    33,188,104,222,61,226,156,156,204,1,255,57,77,14,208,119,
    96,105,38,40,211,72,57,65,215,32,212,86,104,39,106,85,
    51,226,20,183,211,76,156,201,136,103,184,157,101,226,92,70,
    156,231,246,44,19,23,50,226,57,110,23,153,184,148,17,151,
    185,61,207,196,11,25,241,34,183,43,76,92,205,238,243,46,
    49,241,50,180,214,232,98,139,40,53,202,88,19,111,155,177,
    56,192,199,23,218,143,127,212,68,101,95,255,255,80,198,254,
    28,210,250,232,69,73,74,20,53,157,209,73,170,39,178,227,
    93,81,77,190,124,250,232,84,49,224,180,99,233,42,169,253,
    186,58,46,83,112,50,212,146,60,29,165,162,147,7,145,155,
    185,214,135,92,59,14,207,177,187,245,217,151,221,45,30,132,
    23,241,68,98,241,137,100,147,78,36,251,108,34,199,208,135,
    146,17,176,75,185,165,232,104,22,202,199,47,21,82,91,75,
    159,67,72,88,119,48,144,161,103,95,133,226,209,130,95,143,
    7,75,148,142,159,65,161,178,51,197,57,60,75,156,140,115,
    218,127,10,86,96,191,151,242,200,30,27,2,56,24,254,158,
    5,67,157,174,5,70,155,144,189,73,13,111,59,249,142,99,
    255,58,247,223,47,79,135,116,79,38,202,9,232,173,227,14,
    176,146,244,61,58,252,190,197,108,172,78,249,146,245,228,43,
    181,249,134,76,67,229,135,44,213,91,49,32,193,22,159,39,
    24,189,85,27,167,99,205,254,139,247,6,138,111,22,89,168,
    55,155,73,210,80,21,113,148,156,23,237,175,96,54,136,2,
    55,246,213,144,5,120,221,57,180,52,95,211,167,4,117,237,
    116,12,146,104,47,110,75,103,123,47,201,80,242,70,19,105,
    121,190,111,47,82,79,107,197,226,164,248,79,175,99,255,99,
    51,51,251,31,37,171,245,211,49,195,163,111,183,43,99,94,
    255,53,167,208,194,124,255,174,159,57,71,241,182,228,201,64,
    42,121,138,124,171,40,230,211,59,71,2,115,28,13,29,71,
    223,142,224,115,224,56,99,172,59,127,133,75,61,130,244,199,
    20,172,59,69,25,43,207,69,81,248,51,42,229,138,224,210,
    255,216,207,120,245,60,199,233,27,129,97,98,115,9,48,155,
    167,52,6,109,86,111,83,46,224,27,174,45,183,175,127,4,
    224,251,108,155,126,49,227,123,69,251,195,60,53,210,21,44,
    95,195,232,43,49,220,214,248,88,194,167,16,251,23,68,167,
    197,251,27,235,153,182,235,50,220,43,42,123,39,139,210,251,
    20,164,6,15,230,223,230,94,50,231,94,22,94,31,188,124,
    220,125,13,132,111,34,79,170,159,30,27,122,220,232,95,186,
    137,188,21,133,29,191,123,59,84,241,80,173,60,119,124,211,
    239,235,159,112,212,252,177,247,94,236,98,127,241,24,53,145,
    177,239,6,254,83,253,155,77,70,230,27,135,87,9,240,49,
    188,226,146,225,216,4,62,152,228,121,252,53,174,38,24,238,
    177,236,250,9,74,199,162,189,80,168,180,42,33,40,158,54,
    55,22,153,143,47,122,244,205,131,190,161,189,65,191,24,36,
    116,1,77,63,251,84,102,43,24,73,84,191,152,162,138,21,
    140,101,78,207,85,172,233,169,138,85,153,48,249,30,126,70,
    44,24,85,171,50,53,45,138,127,107,24,107,85,99,109,169,
    34,254,7,238,19,49,227,
};

EmbeddedPython embedded_m5_internal_param_X86IntelMPLocalIntAssignment(
    "m5/internal/param_X86IntelMPLocalIntAssignment.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_X86IntelMPLocalIntAssignment.py",
    "m5.internal.param_X86IntelMPLocalIntAssignment",
    data_m5_internal_param_X86IntelMPLocalIntAssignment,
    2503,
    8448);

} // anonymous namespace