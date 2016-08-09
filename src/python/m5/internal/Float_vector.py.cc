#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_Float_vector[] = {
    120,156,197,92,107,136,36,87,21,62,85,253,152,237,222,153,
    157,231,206,236,43,187,157,199,38,147,104,118,162,102,147,104,
    194,106,140,73,72,196,77,82,19,217,205,36,90,233,233,186,
    51,83,51,221,85,157,170,154,221,237,56,171,196,89,141,79,
    84,124,160,34,34,62,144,128,32,8,130,32,8,130,32,8,
    66,64,16,4,65,80,4,65,240,159,32,8,234,249,206,173,
    234,174,126,172,100,211,211,53,51,211,103,111,87,223,190,231,
    126,231,126,247,220,123,78,221,218,26,197,63,5,126,189,167,
    66,20,222,99,16,57,252,103,80,157,168,97,208,138,65,134,
    50,200,153,163,173,2,5,247,146,83,160,107,68,43,38,41,
    147,118,185,144,163,231,77,242,198,229,59,69,170,231,228,138,
    65,173,50,169,60,173,20,232,130,55,77,121,85,164,173,50,
    5,47,146,97,24,158,65,23,157,49,114,14,208,53,110,157,
    11,37,105,240,0,225,98,89,46,150,200,57,40,23,203,228,
    140,75,225,32,181,166,72,141,211,202,4,170,173,28,226,102,
    239,226,102,39,165,217,215,209,172,195,159,28,38,231,16,170,
    115,191,158,67,205,60,106,138,190,73,105,101,138,28,105,101,
    141,241,76,183,43,78,147,202,209,230,12,173,204,144,226,191,
    105,218,101,200,206,76,82,113,182,93,113,86,42,206,209,202,
    28,41,254,155,213,21,139,180,188,56,207,214,115,255,203,63,
    139,108,61,138,198,89,92,82,65,232,250,158,237,122,107,190,
    107,226,243,34,4,108,93,131,24,139,141,254,8,140,254,26,
    137,197,29,51,54,250,85,226,134,13,216,180,110,210,85,41,
    92,53,169,181,72,59,6,109,230,201,201,209,14,171,41,160,
    3,235,6,237,154,244,66,14,21,174,178,204,179,105,78,82,
    62,210,22,223,20,211,232,150,198,232,106,129,118,10,180,124,
    113,199,196,133,173,18,5,63,160,151,79,72,163,7,164,81,
    147,118,88,230,105,55,79,87,139,116,129,43,241,165,205,18,
    224,27,23,119,24,41,95,89,94,204,115,111,207,167,224,2,
    138,227,6,94,181,161,162,9,46,219,143,213,253,106,100,95,
    82,181,200,15,22,203,73,21,63,60,211,172,70,27,150,124,
    39,7,99,52,154,145,180,229,123,42,58,200,133,53,215,115,
    236,134,239,108,215,85,116,0,13,217,107,110,93,217,182,124,
    248,68,163,233,7,209,163,65,224,7,22,236,41,23,89,81,
    251,27,176,102,173,238,135,106,17,218,68,141,133,230,35,212,
    94,107,74,139,232,128,244,19,95,118,84,88,11,220,102,196,
    195,164,91,68,109,180,182,136,1,18,17,62,205,98,105,195,
    111,168,37,191,238,242,152,94,105,181,150,154,129,191,30,84,
    27,75,235,170,113,246,238,48,170,174,214,213,210,234,182,91,
    119,150,46,62,112,223,82,179,21,109,248,222,82,227,236,146,
    235,69,138,173,82,95,74,219,227,12,127,62,131,150,47,187,
    235,182,43,152,236,13,85,111,170,0,166,11,143,65,171,49,
    101,140,27,69,35,103,44,26,19,92,42,240,43,103,156,48,
    15,26,231,93,160,170,1,41,216,148,79,243,7,131,106,208,
    150,73,193,9,176,99,147,255,12,12,39,115,100,25,159,153,
    242,217,51,48,135,190,186,153,195,152,235,139,59,194,40,166,
    22,215,124,8,131,236,145,208,162,64,155,69,210,116,97,150,
    105,254,4,45,72,174,142,102,76,110,60,79,225,87,137,205,
    203,68,217,161,152,68,187,57,50,188,41,138,202,152,133,124,
    117,158,21,126,92,120,184,188,136,238,159,23,74,68,27,110,
    232,95,246,196,240,40,203,204,89,102,203,60,221,122,106,117,
    147,237,21,158,226,11,207,249,219,149,90,213,243,252,168,82,
    117,156,74,53,138,2,119,117,59,82,97,37,242,43,167,195,
    69,140,165,53,157,176,170,221,94,171,153,176,8,35,206,44,
    210,111,28,183,22,241,155,89,121,35,163,16,170,136,25,177,
    225,59,33,95,71,19,235,42,178,208,201,8,70,246,165,35,
    66,24,27,85,161,158,235,29,226,247,15,39,61,17,86,46,
    22,19,14,133,170,190,22,149,133,142,213,48,180,165,39,184,
    46,204,67,195,151,170,245,109,37,173,51,125,34,238,16,138,
    186,15,163,230,222,17,224,72,96,11,22,207,247,156,22,119,
    205,173,221,1,173,71,132,129,227,194,193,195,204,191,49,150,
    69,254,183,104,204,155,181,124,204,186,98,194,60,120,190,136,
    100,220,141,120,232,153,133,187,236,101,22,77,113,19,2,71,
    230,226,45,40,225,203,214,9,136,155,32,78,66,156,74,16,
    143,16,246,68,47,236,251,161,202,20,172,130,10,195,146,75,
    80,57,93,243,233,104,103,62,177,35,92,198,188,48,49,123,
    58,243,34,15,167,25,156,131,228,170,50,227,114,20,62,11,
    23,141,249,35,141,97,170,48,233,81,234,76,5,177,145,53,
    5,236,7,18,22,91,160,102,154,159,235,41,126,90,24,30,
    33,167,117,52,113,130,54,106,104,90,90,199,209,84,97,128,
    145,43,16,55,103,96,233,14,193,214,251,8,246,32,180,78,
    197,4,155,16,98,149,249,53,101,214,114,177,249,219,75,226,
    108,15,177,192,170,252,0,86,221,142,82,174,31,112,118,132,
    138,97,62,150,34,20,122,102,166,209,156,231,66,107,1,32,
    210,84,90,224,165,253,130,183,192,171,181,41,171,245,61,178,
    90,203,138,47,187,22,237,148,115,226,151,117,161,0,107,172,
    229,104,62,94,133,195,18,75,198,114,165,85,241,215,42,145,
    192,133,15,125,232,116,120,230,116,248,32,123,199,202,57,241,
    75,218,63,106,15,24,168,38,60,24,190,250,232,149,154,146,
    5,80,222,217,182,118,88,182,56,47,59,94,88,153,85,135,
    97,75,51,49,178,184,238,48,10,224,177,71,109,230,114,219,
    204,232,245,147,208,83,22,27,231,140,5,102,80,217,144,206,
    216,218,73,203,198,74,62,229,215,123,97,119,0,86,132,77,
    172,181,172,187,42,40,128,199,122,107,23,75,70,135,193,90,
    226,70,63,152,176,163,216,97,7,94,185,132,235,175,146,236,
    54,13,250,36,97,252,121,152,99,174,183,167,6,6,124,22,
    213,63,76,50,41,6,172,246,226,91,150,177,194,75,13,118,
    57,225,253,82,85,47,254,79,210,167,82,51,42,89,162,115,
    241,126,50,189,68,231,219,126,73,136,243,134,150,225,124,183,
    3,195,184,108,84,67,84,211,94,169,51,73,59,94,191,189,
    23,100,175,60,66,22,29,208,26,108,116,230,133,14,135,176,
    200,29,55,102,205,20,51,222,6,241,246,54,41,140,228,218,
    104,250,117,138,174,191,24,219,218,231,63,15,229,121,233,238,
    228,152,236,58,244,254,232,9,110,178,202,205,180,57,95,72,
    56,255,151,54,231,149,44,80,215,36,182,128,52,49,210,187,
    166,193,33,26,239,202,16,17,229,73,21,104,165,72,106,12,
    33,0,2,175,66,18,120,21,227,192,171,19,171,141,75,185,
    36,229,9,137,213,8,1,86,28,171,77,38,177,26,71,89,
    19,82,152,142,195,49,14,172,226,0,108,22,1,24,10,115,
    113,0,182,194,33,219,140,20,230,227,72,107,101,1,177,38,
    10,71,16,208,161,112,148,156,121,41,28,35,103,65,10,199,
    49,157,177,208,200,92,74,94,226,105,225,151,187,150,107,25,
    195,243,122,116,219,236,212,196,131,184,50,106,15,6,238,61,
    84,175,54,86,157,234,185,117,104,129,170,90,50,255,205,164,
    223,83,233,126,99,238,26,215,235,186,188,61,155,244,255,210,
    168,189,215,125,220,104,187,223,50,87,29,191,38,46,235,217,
    13,85,105,168,198,42,71,186,27,110,179,178,86,175,174,203,
    120,228,98,92,79,37,184,34,97,96,239,182,39,188,11,210,
    175,212,124,143,23,147,109,232,171,56,138,99,64,229,84,238,
    174,200,74,84,113,195,74,117,149,63,173,214,34,61,23,187,
    61,137,236,170,171,193,122,40,27,232,173,203,40,102,49,158,
    54,135,245,46,71,17,46,117,175,249,93,76,68,223,156,14,
    1,165,251,133,182,83,57,65,163,95,120,48,116,141,54,229,
    146,206,181,41,55,161,135,102,211,72,54,173,105,190,73,64,
    60,215,231,110,108,9,93,178,131,0,15,233,119,172,172,227,
    223,246,226,153,158,60,109,36,122,225,235,7,51,219,15,198,
    245,106,65,106,95,131,111,120,163,230,15,104,1,189,65,138,
    59,123,6,200,81,105,64,214,185,209,143,144,224,129,218,237,
    161,241,44,12,192,227,114,151,188,154,74,97,58,155,1,38,
    204,241,68,245,149,161,113,13,152,69,234,165,237,106,61,99,
    80,240,61,162,247,35,3,188,214,13,56,133,1,180,171,249,
    205,86,102,62,65,24,7,141,31,221,115,28,158,186,18,101,
    139,3,26,95,25,14,199,128,105,99,11,18,219,206,14,75,
    156,104,19,173,215,246,28,79,51,80,151,92,127,59,204,22,
    79,162,245,213,161,61,192,124,63,164,170,115,169,199,177,101,
    225,172,17,148,197,154,63,51,52,170,195,131,136,167,94,98,
    218,101,235,216,138,194,61,40,254,252,104,48,121,106,159,48,
    65,241,23,135,198,52,208,65,184,28,195,119,161,202,130,126,
    241,222,89,84,127,121,68,184,194,237,213,253,194,37,170,191,
    54,10,111,97,219,251,49,92,146,90,211,154,191,65,212,151,
    221,4,170,199,7,161,250,220,32,183,62,16,85,239,96,189,
    51,51,84,162,249,91,212,191,82,117,197,111,59,157,248,77,
    122,149,241,122,234,242,167,182,253,237,78,47,23,197,74,237,
    244,165,228,131,116,2,137,53,54,85,16,181,116,210,14,137,
    118,235,12,196,157,93,174,205,81,117,21,41,187,123,28,162,
    41,106,223,107,112,20,71,219,126,203,182,99,51,241,23,108,
    91,162,48,235,221,16,15,67,60,2,241,40,196,227,16,79,
    64,188,31,226,3,16,79,65,60,3,177,12,129,204,167,117,
    1,226,57,8,228,179,172,23,186,44,56,194,240,241,94,110,
    116,13,173,195,90,69,227,184,89,50,139,70,201,40,153,165,
    220,56,255,150,174,247,107,198,247,240,209,142,190,157,221,159,
    103,115,140,55,144,103,211,135,32,226,108,91,49,73,175,141,
    37,233,53,57,245,128,66,73,146,108,58,243,86,74,50,111,
    58,195,54,158,100,216,38,146,12,219,161,36,195,54,153,100,
    216,166,146,12,219,116,146,97,155,73,50,108,179,73,134,109,
    46,201,176,29,78,50,108,243,73,134,109,33,201,176,29,73,
    50,108,71,201,57,146,228,220,142,198,57,55,231,152,20,78,
    144,115,92,10,55,145,115,66,10,39,201,185,73,10,167,200,
    57,41,133,10,57,167,164,112,51,57,21,41,220,66,206,205,
    82,184,149,156,91,164,112,27,57,183,74,225,52,57,183,73,
    225,118,82,119,208,230,34,173,220,73,206,105,185,114,23,18,
    125,184,197,51,84,162,47,139,21,91,146,45,223,167,189,204,
    239,89,247,103,221,109,235,1,138,111,71,92,47,183,119,131,
    123,249,195,61,179,72,60,26,124,78,182,158,52,209,250,67,
    250,63,254,190,220,30,161,221,158,172,93,141,50,116,253,226,
    28,95,27,208,209,27,48,251,145,94,179,219,184,177,240,178,
    10,252,44,99,66,125,111,186,173,248,71,195,97,234,163,146,
    109,175,250,126,61,251,32,87,107,253,241,112,104,230,250,209,
    212,149,151,37,24,189,190,139,210,159,164,176,164,239,54,10,
    150,89,234,222,220,233,59,135,253,136,142,246,35,90,87,81,
    88,119,107,136,158,58,41,123,67,230,35,228,230,168,33,142,
    83,124,56,34,238,197,79,123,198,236,198,55,177,3,64,134,
    29,144,217,238,99,53,186,142,250,159,141,104,20,121,11,216,
    55,138,214,22,68,61,51,148,157,62,252,124,232,49,28,224,
    28,185,121,94,34,26,153,15,225,193,4,156,214,254,139,81,
    96,99,250,239,35,182,182,246,95,142,2,91,184,175,216,218,
    218,127,69,67,173,5,83,189,192,154,126,51,187,117,0,131,
    192,10,127,157,194,240,230,210,22,179,189,48,170,205,166,242,
    156,125,72,156,105,197,191,25,110,84,102,122,225,168,70,51,
    202,240,46,135,220,174,129,202,223,14,135,99,186,23,71,232,
    190,156,221,13,92,125,30,150,53,190,62,52,191,250,129,92,
    174,54,83,236,202,32,90,210,104,88,237,239,246,152,91,171,
    106,221,245,178,229,150,168,252,253,30,123,46,204,247,76,61,
    23,43,252,195,112,24,250,220,86,144,241,96,192,99,105,157,
    127,220,227,153,30,100,58,28,152,27,208,248,167,61,158,27,
    181,186,170,102,152,47,208,143,145,176,202,63,15,135,227,88,
    47,142,117,28,64,172,215,253,90,182,249,15,244,180,75,245,
    95,135,195,213,23,135,243,230,193,94,173,214,182,50,190,57,
    27,107,253,91,15,154,27,223,87,246,47,244,65,53,84,25,
    239,40,101,181,135,222,191,247,224,73,14,197,11,158,247,117,
    240,48,24,253,80,216,156,36,173,146,7,18,240,72,218,5,
    239,24,229,121,58,226,72,249,187,113,164,124,71,14,4,219,
    166,62,85,222,73,110,21,40,237,200,61,117,217,78,219,65,
    167,36,113,88,196,194,73,189,84,240,7,131,200,167,163,78,
    133,225,54,198,63,136,146,243,186,147,70,206,152,51,38,134,
    185,175,214,205,220,237,112,67,83,55,219,61,170,28,218,79,
    116,255,115,143,221,229,90,224,123,25,30,98,1,135,68,229,
    191,134,195,209,183,120,101,235,82,176,120,65,227,191,83,40,
    222,116,18,165,63,18,10,67,119,221,75,205,160,115,89,81,
    77,194,33,209,254,31,26,214,81,246,239,148,148,196,18,217,
    122,74,217,46,137,98,195,216,115,68,174,23,170,32,218,7,
    68,90,113,62,133,232,205,249,184,190,188,50,155,74,5,151,
    246,227,156,81,172,121,204,216,219,237,70,173,218,172,214,220,
    44,35,113,108,55,18,173,229,1,104,186,78,12,236,255,137,
    239,9,35,190,155,183,248,22,74,31,22,176,62,4,33,199,
    3,58,39,3,112,107,75,238,56,89,10,2,7,253,45,156,
    106,183,112,110,220,194,201,107,235,37,8,52,104,225,136,175,
    117,25,162,69,201,214,224,42,196,199,32,94,129,216,133,248,
    4,4,78,205,89,159,134,248,44,4,142,102,89,95,128,248,
    18,132,28,80,248,10,4,206,203,88,95,135,248,38,4,142,
    100,88,56,241,96,125,7,226,187,93,115,53,62,190,208,181,
    75,177,81,227,197,46,219,142,208,192,85,110,244,123,104,29,
    143,162,21,141,227,70,209,196,97,130,27,250,29,235,61,108,
    80,50,100,105,232,249,111,2,52,20,12,159,126,66,170,21,
    90,184,98,77,182,77,162,15,41,198,231,62,48,186,178,179,
    56,95,109,232,7,146,229,137,91,235,86,8,220,66,183,238,
    104,15,61,178,252,242,88,154,126,236,143,119,126,242,184,133,
    60,93,97,189,3,2,7,39,34,188,237,57,178,131,67,34,
    1,199,170,33,95,16,99,244,223,49,72,87,201,98,76,244,
    19,54,250,89,197,115,200,204,134,184,139,141,199,151,75,147,
    37,30,31,60,60,159,51,202,188,111,204,231,198,167,74,249,
    241,131,165,124,105,44,39,207,158,78,24,179,102,57,95,58,
    56,255,174,146,81,54,231,183,74,198,255,0,36,86,49,111,
};

EmbeddedPython embedded_m5_internal_Float_vector(
    "m5/internal/Float_vector.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/Float_vector.py",
    "m5.internal.Float_vector",
    data_m5_internal_Float_vector,
    3408,
    17170);

} // anonymous namespace
