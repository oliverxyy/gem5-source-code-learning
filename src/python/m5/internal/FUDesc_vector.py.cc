#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_FUDesc_vector[] = {
    120,156,205,92,107,136,36,87,21,62,85,253,152,237,158,153,
    157,153,157,215,238,206,100,183,243,216,164,19,205,142,143,108,
    18,77,216,24,55,15,18,113,215,212,36,236,102,18,173,212,
    116,221,153,169,153,238,170,78,85,205,238,118,156,85,226,172,
    198,39,42,62,80,17,17,193,31,42,136,32,8,130,32,8,
    130,32,136,130,32,8,130,16,16,4,65,16,252,39,196,243,
    157,170,234,174,158,238,149,108,122,186,198,204,244,201,157,170,
    219,247,220,239,158,239,158,123,207,169,91,91,163,248,191,2,
    127,222,87,33,10,174,107,68,54,255,106,84,39,106,104,180,
    162,145,166,52,178,103,104,171,64,254,125,100,23,232,58,209,
    138,78,74,167,93,46,228,232,5,157,220,49,249,78,145,234,
    57,185,162,81,171,76,42,79,43,5,186,232,78,81,94,21,
    105,171,76,254,75,164,105,154,171,209,37,123,132,236,67,196,
    170,86,184,80,146,6,15,17,46,150,229,98,137,236,81,185,
    88,38,123,76,10,163,212,154,36,53,70,43,227,168,182,114,
    152,155,189,135,155,157,144,102,127,143,102,109,190,51,75,246,
    97,84,231,126,61,143,154,121,212,20,125,19,210,202,36,217,
    210,202,26,227,153,106,87,156,34,149,163,205,35,180,114,132,
    20,255,78,209,46,67,142,225,76,211,202,76,2,109,54,85,
    158,75,149,231,83,229,163,169,242,177,84,249,184,148,143,36,
    202,167,219,202,23,68,249,34,173,44,146,226,223,133,72,121,
    145,150,171,115,108,17,231,13,254,175,202,22,161,112,140,197,
    101,229,7,142,231,154,142,187,230,57,58,238,23,33,96,191,
    26,196,72,108,200,115,48,228,15,72,172,104,235,177,33,175,
    17,55,172,161,67,117,157,174,73,225,154,78,173,42,237,104,
    180,153,39,59,71,59,172,166,128,14,172,107,180,171,211,139,
    57,84,184,198,50,207,195,125,130,242,97,100,197,77,25,238,
    168,165,17,186,86,160,157,2,45,95,218,209,113,97,171,68,
    254,247,233,149,69,105,244,144,52,170,211,14,203,60,237,230,
    233,90,145,46,114,37,190,180,89,2,124,237,210,14,35,229,
    43,203,213,60,247,246,124,10,46,160,216,142,239,90,13,21,
    30,230,178,249,196,115,143,169,160,102,94,86,181,208,243,171,
    229,164,142,23,156,110,90,225,134,33,95,202,97,52,26,205,
    80,26,243,92,21,142,114,97,205,113,109,179,225,217,219,117,
    21,30,66,75,230,154,83,87,166,41,55,159,106,52,61,63,
    124,220,247,61,223,192,128,202,197,186,103,181,191,129,225,172,
    213,189,64,85,161,77,212,24,104,62,68,237,181,166,180,136,
    14,72,71,241,101,155,123,233,59,205,144,237,20,181,136,218,
    104,173,10,11,137,8,158,97,177,180,225,53,212,146,87,119,
    216,168,87,91,173,165,166,239,173,251,86,99,105,93,53,206,
    220,27,132,214,106,93,45,173,110,59,117,123,233,210,131,247,
    47,53,91,225,134,231,46,53,206,44,57,110,168,120,88,234,
    75,93,3,114,154,43,28,65,211,87,156,117,211,17,80,230,
    134,170,55,149,63,142,171,199,161,86,155,212,198,180,162,150,
    211,170,218,56,151,10,252,201,105,139,250,168,118,222,1,172,
    26,160,130,79,249,52,131,96,86,141,182,116,242,23,193,143,
    77,254,213,96,80,102,201,50,238,233,114,239,25,140,71,116,
    117,51,7,171,71,23,119,132,83,76,46,174,249,48,204,236,
    146,16,163,64,155,69,138,8,195,60,139,24,228,183,32,185,
    58,154,209,185,241,60,5,95,35,30,95,166,202,14,197,52,
    218,205,145,230,78,82,88,198,220,230,171,115,172,240,19,194,
    196,229,42,186,127,94,56,17,110,56,129,119,197,149,145,71,
    89,230,206,50,143,204,135,90,23,86,55,121,188,130,147,124,
    225,121,111,187,82,179,92,215,11,43,150,109,87,172,48,244,
    157,213,237,80,5,149,208,171,156,10,170,48,166,49,149,208,
    170,221,94,171,153,208,8,38,103,26,69,127,216,78,45,228,
    63,166,229,15,177,66,160,66,166,196,134,103,7,124,29,77,
    172,171,208,64,39,67,12,178,39,29,17,198,152,168,10,245,
    92,15,84,127,52,233,137,208,178,90,76,72,20,168,250,90,
    88,22,62,90,65,96,74,79,112,93,168,135,134,47,91,245,
    109,37,173,51,127,66,238,16,138,81,31,134,78,190,163,0,
    146,224,22,48,174,231,218,45,238,155,83,187,11,106,143,10,
    5,199,132,132,179,76,192,17,150,69,254,127,81,155,211,107,
    249,152,118,197,132,122,112,126,33,137,225,181,216,246,76,195,
    93,118,52,85,93,60,133,224,145,217,120,27,74,248,178,177,
    8,113,11,196,9,136,147,9,228,97,226,30,223,139,251,1,
    232,210,5,172,192,130,97,114,9,44,187,107,70,29,235,204,
    40,118,134,203,152,25,58,230,79,103,102,228,225,56,253,179,
    144,92,85,230,92,142,130,103,225,166,49,131,164,49,76,22,
    166,61,74,157,201,32,131,100,76,2,252,161,132,199,6,200,
    153,102,232,122,138,161,6,236,35,244,52,142,37,126,208,68,
    141,136,152,198,2,154,42,244,25,229,10,196,173,89,12,117,
    135,98,235,61,20,123,8,106,39,99,138,141,11,181,202,252,
    153,212,107,185,120,252,219,235,226,244,30,106,129,87,249,62,
    188,186,19,165,92,47,226,12,41,21,227,124,34,69,41,116,
    77,79,195,57,207,133,214,60,80,164,201,52,207,11,252,69,
    119,158,215,108,93,214,236,119,200,154,45,235,190,236,135,34,
    199,156,19,223,28,21,10,24,142,181,28,205,197,107,113,80,
    98,201,96,174,182,42,222,90,37,20,188,240,163,15,159,10,
    78,159,10,30,98,15,89,57,43,190,41,242,145,145,23,244,
    85,19,94,12,95,125,252,106,77,201,42,40,127,153,102,228,
    180,76,113,96,102,188,186,50,175,102,49,152,122,50,202,226,
    190,131,208,135,215,30,250,56,151,219,227,140,110,63,13,69,
    101,25,228,156,54,207,28,42,107,210,27,51,242,212,178,191,
    146,187,252,121,63,6,30,136,21,97,127,108,44,71,125,21,
    24,0,100,188,189,139,39,67,4,97,44,113,171,207,37,252,
    40,118,248,129,79,46,161,251,107,36,187,78,141,62,69,96,
    0,27,58,166,123,123,118,192,228,211,168,254,17,146,121,209,
    103,205,23,255,178,140,117,94,106,176,219,9,30,144,170,209,
    22,224,105,250,116,106,82,37,11,117,46,222,87,166,23,234,
    124,219,55,9,117,222,212,98,156,239,118,98,48,204,134,21,
    160,90,228,153,58,243,180,227,250,219,91,66,246,204,195,228,
    209,161,72,133,137,222,188,216,97,17,150,186,5,109,90,79,
    113,227,157,16,239,106,211,66,75,174,13,169,99,39,233,198,
    107,178,25,121,254,23,160,61,47,253,157,24,145,221,71,180,
    79,122,138,219,180,184,153,54,237,11,9,237,95,111,211,94,
    201,50,117,93,162,12,72,29,182,222,213,53,14,0,121,119,
    134,120,43,79,170,64,43,69,82,35,8,6,16,214,21,146,
    176,174,24,135,117,157,72,112,76,202,37,41,143,75,36,72,
    8,223,226,72,112,34,137,4,57,134,27,151,194,84,28,236,
    113,216,22,135,119,211,8,239,80,152,137,195,187,149,89,196,
    92,40,204,197,49,215,202,60,34,89,20,142,34,92,68,225,
    24,217,115,82,56,78,246,188,20,22,48,163,177,218,200,108,
    74,62,226,109,225,155,187,22,109,49,226,249,200,188,109,126,
    70,212,131,184,58,116,47,6,246,61,92,183,26,171,182,117,
    118,29,106,160,171,150,184,0,61,233,248,100,186,227,152,190,
    218,141,250,46,127,158,73,0,92,30,186,7,187,159,91,109,
    119,92,230,171,237,213,196,109,61,187,161,42,13,213,88,229,
    168,119,195,105,86,214,234,214,186,88,36,23,3,187,144,0,
    11,133,131,123,183,63,193,61,144,94,165,230,185,188,164,108,
    67,95,197,86,28,14,42,187,114,111,69,214,163,138,19,84,
    172,85,190,107,213,194,104,58,118,123,19,217,95,91,254,122,
    32,91,233,173,43,40,102,98,81,147,99,124,135,3,10,135,
    186,151,254,46,50,162,115,118,135,131,210,255,66,219,177,44,
    82,6,203,15,140,215,104,179,46,233,93,155,117,227,145,113,
    54,181,100,251,154,166,156,68,199,51,61,46,199,148,48,38,
    67,12,112,147,94,103,156,163,96,184,189,134,166,39,80,27,
    74,180,254,245,162,153,238,69,227,184,53,63,181,193,193,55,
    220,161,83,8,204,128,98,63,69,159,125,67,100,171,52,34,
    227,108,6,54,18,64,208,187,61,48,160,249,62,128,28,238,
    147,91,83,41,80,103,178,0,133,137,158,232,190,58,48,176,
    62,51,73,189,188,109,213,179,70,5,15,36,138,63,218,199,
    119,221,132,103,232,195,188,154,215,108,101,231,24,132,116,80,
    249,177,125,7,226,170,171,97,198,64,160,242,213,193,128,244,
    153,58,166,64,49,205,12,193,196,201,55,81,123,125,223,1,
    53,125,117,217,241,182,131,140,1,37,106,95,27,216,13,204,
    245,98,178,236,203,123,220,91,38,62,27,65,90,172,250,179,
    3,195,154,237,199,61,245,50,51,47,99,247,86,20,250,65,
    243,23,134,3,202,85,7,5,10,154,191,52,48,168,190,94,
    194,225,168,190,11,86,38,12,140,119,210,162,251,43,67,2,
    22,108,175,30,24,48,209,253,245,97,184,12,211,60,16,131,
    73,194,45,82,253,77,162,158,164,39,96,61,217,15,214,231,
    251,121,247,190,176,246,154,235,61,217,193,18,213,223,166,222,
    21,171,43,160,219,233,4,116,210,173,172,23,86,135,111,155,
    230,119,58,221,172,202,56,181,211,154,209,211,88,201,42,177,
    202,166,242,195,86,148,203,67,10,222,56,13,113,119,151,131,
    179,85,93,133,202,236,182,68,56,73,237,199,16,182,226,0,
    220,107,153,102,60,78,252,5,211,148,168,204,120,4,226,81,
    136,115,16,143,67,60,9,241,20,196,7,32,62,8,113,1,
    2,163,98,44,67,32,33,106,92,132,120,30,2,73,46,227,
    197,174,33,28,102,60,121,31,183,186,134,230,49,92,69,109,
    65,47,233,69,173,164,149,244,82,110,140,127,74,55,250,209,
    37,243,31,181,19,63,238,238,77,191,217,218,155,72,191,69,
    39,47,226,36,92,49,201,186,141,36,89,55,57,106,129,66,
    73,114,111,81,66,174,148,36,228,162,196,219,88,146,120,27,
    79,18,111,135,147,196,219,68,146,120,155,76,18,111,83,73,
    226,237,72,146,120,155,78,18,111,51,73,226,109,54,73,188,
    205,37,137,183,249,36,241,118,52,73,188,29,35,251,104,146,
    138,59,22,167,226,236,227,82,88,36,123,65,10,183,144,189,
    40,133,19,100,223,34,133,147,100,159,144,66,133,236,147,82,
    184,149,236,138,20,110,35,251,86,41,220,78,246,109,82,184,
    131,236,219,165,112,138,236,59,164,112,39,169,187,104,179,74,
    43,119,147,125,74,174,220,131,252,31,30,255,12,148,255,203,
    100,229,150,4,204,143,104,63,211,126,198,3,153,247,219,120,
    144,226,39,21,55,74,249,221,228,198,126,110,239,68,18,183,
    6,199,147,177,63,77,212,254,152,254,135,219,47,183,141,180,
    187,39,155,87,163,44,87,0,241,145,63,233,211,211,155,24,
    249,99,61,35,111,226,169,195,43,202,247,50,141,18,163,231,
    215,109,205,63,29,12,85,47,159,76,115,213,243,234,7,16,
    248,70,106,127,54,24,158,217,62,120,234,202,205,20,78,180,
    216,139,214,159,167,208,164,159,72,10,154,105,234,222,235,69,
    79,23,123,49,29,239,131,105,93,133,65,221,169,33,162,234,
    228,244,53,153,151,144,155,67,7,57,70,241,49,138,184,27,
    191,216,99,183,155,223,213,246,131,25,116,96,102,188,179,141,
    240,117,244,255,114,88,150,228,77,97,143,37,141,45,136,122,
    118,56,59,157,248,213,192,118,236,231,40,185,125,94,48,26,
    217,155,113,52,129,23,169,255,245,80,208,241,44,56,72,116,
    109,245,191,25,10,186,224,96,209,181,213,255,150,6,90,25,
    166,122,160,53,189,102,134,171,2,12,193,26,127,151,66,241,
    214,146,26,51,61,64,172,102,83,185,246,65,164,214,34,205,
    127,24,204,50,211,61,128,84,163,25,102,249,60,68,158,236,
    64,231,31,7,67,114,164,7,73,224,188,146,225,19,223,232,
    52,45,171,252,211,192,44,235,3,229,138,213,76,113,44,139,
    96,42,194,195,122,255,188,223,12,91,85,235,142,155,49,195,
    68,231,95,246,219,139,97,230,103,235,197,88,227,95,7,67,
    209,235,194,252,172,13,2,239,21,41,125,125,191,231,188,159,
    173,73,48,71,160,242,111,251,61,71,106,117,101,101,153,86,
    136,222,72,97,157,127,31,12,201,66,15,146,117,28,98,172,
    215,189,90,198,137,18,244,181,75,247,63,6,67,214,27,173,
    243,142,194,92,181,106,91,89,63,213,141,213,254,115,15,158,
    155,223,113,246,89,252,125,43,80,89,239,53,101,7,0,197,
    255,218,131,40,57,97,47,136,30,235,32,98,56,209,107,102,
    51,146,225,74,94,111,192,75,110,23,221,227,148,231,105,137,
    227,233,143,224,120,250,142,28,45,54,245,232,132,122,39,19,
    86,160,180,91,119,213,21,179,107,36,162,28,38,14,155,24,
    56,240,151,10,15,49,36,114,119,232,137,51,60,251,248,55,
    81,114,244,119,66,203,105,51,218,248,32,15,26,247,240,119,
    59,216,136,8,156,241,238,85,94,2,72,148,255,103,191,93,
    231,154,239,185,89,158,131,1,147,68,231,27,131,33,233,93,
    204,50,118,46,88,204,160,82,215,246,33,221,210,39,82,10,
    2,103,221,77,205,164,179,153,17,78,194,37,81,95,208,6,
    117,154,125,246,79,74,34,141,140,189,166,108,162,68,243,161,
    33,96,114,220,64,249,225,65,96,138,52,143,166,48,189,53,
    127,215,155,141,230,209,82,254,229,3,57,177,20,171,62,172,
    237,243,22,164,102,53,173,154,147,105,188,142,45,72,162,118,
    170,15,158,174,115,7,255,7,7,201,103,180,248,121,96,245,
    109,148,62,114,96,124,24,66,14,25,116,206,23,224,209,152,
    60,176,50,20,4,222,32,48,112,90,222,192,113,116,3,231,
    185,141,151,33,208,160,129,99,195,198,21,136,22,37,155,133,
    107,16,31,135,120,21,98,23,226,147,16,56,131,103,124,6,
    226,115,16,56,230,101,124,17,226,203,16,114,204,225,171,16,
    56,120,99,124,3,226,91,16,56,217,97,224,220,132,241,93,
    136,239,117,77,217,248,16,68,247,198,197,68,149,151,186,70,
    119,152,67,108,113,171,63,68,243,120,213,173,168,45,104,69,
    29,103,18,110,234,103,100,239,153,133,146,38,203,196,158,127,
    141,32,194,130,87,38,163,23,176,90,129,129,43,198,68,123,
    80,162,67,143,241,249,17,216,87,182,26,231,173,70,244,214,
    179,188,213,107,220,14,129,7,241,198,93,109,227,35,247,42,
    111,189,69,239,21,242,118,80,222,228,144,23,55,140,119,67,
    224,252,133,68,55,123,14,255,224,176,137,207,161,108,192,23,
    228,101,210,198,153,211,201,104,157,110,90,60,184,177,97,228,
    221,254,198,25,73,243,166,235,92,104,166,70,244,6,45,68,
    117,122,110,42,119,27,247,206,161,171,210,183,222,111,46,59,
    141,232,237,116,217,230,166,239,219,190,197,229,217,61,87,217,
    77,57,86,157,23,21,49,108,159,135,38,105,188,153,48,44,
    122,23,41,122,181,243,44,210,210,1,222,92,197,11,223,165,
    137,18,179,13,255,222,64,78,43,243,198,56,159,27,155,44,
    229,199,70,75,249,210,72,78,222,213,29,215,166,245,114,190,
    52,58,247,222,146,86,230,154,233,159,185,173,146,246,95,119,
    159,143,32,
};

EmbeddedPython embedded_m5_internal_FUDesc_vector(
    "m5/internal/FUDesc_vector.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/FUDesc_vector.py",
    "m5.internal.FUDesc_vector",
    data_m5_internal_FUDesc_vector,
    3507,
    17577);

} // anonymous namespace
