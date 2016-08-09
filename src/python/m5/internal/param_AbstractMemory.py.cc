#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_AbstractMemory[] = {
    120,156,197,88,109,111,227,198,17,158,37,41,217,146,229,179,
    252,126,119,246,217,74,218,107,212,67,98,181,105,156,11,16,
    247,208,107,146,2,13,80,39,165,82,156,163,4,101,105,113,
    37,83,166,72,129,92,223,157,2,251,75,125,104,139,126,47,
    208,63,208,15,253,55,253,71,237,204,44,73,81,126,73,175,
    104,163,90,210,122,185,156,157,157,151,103,102,103,183,11,233,
    95,9,127,63,107,0,36,127,22,0,30,126,5,4,0,67,
    1,29,1,66,10,240,214,224,180,4,241,123,224,149,224,21,
    64,199,0,105,192,37,118,76,248,202,128,176,198,115,202,16,
    152,60,34,96,92,5,105,65,167,4,207,194,101,176,100,25,
    78,171,16,255,14,132,16,161,128,35,111,14,188,121,120,133,
    220,177,83,97,134,243,64,131,85,30,172,128,183,192,131,85,
    240,106,220,89,128,113,29,100,13,58,139,68,214,185,131,108,
    31,33,219,37,102,251,15,98,235,225,155,117,240,238,16,57,
    202,245,37,81,90,68,201,235,45,49,151,122,38,229,50,116,
    86,178,254,106,161,191,86,232,175,23,250,27,133,254,38,247,
    81,154,21,24,220,133,193,61,24,220,135,30,26,104,57,95,
    121,11,164,9,131,109,232,108,131,196,239,22,92,162,13,189,
    149,194,140,7,60,99,53,159,177,195,51,118,161,179,11,18,
    191,59,122,70,25,218,205,13,244,139,255,79,252,107,162,95,
    64,213,176,121,46,227,196,143,66,199,15,123,145,111,208,251,
    50,53,228,197,46,53,115,169,59,63,34,119,254,29,216,151,
    158,145,186,243,2,144,177,32,93,2,3,46,184,115,97,192,
    184,9,231,2,6,22,120,38,156,227,50,37,18,160,47,224,
    210,128,175,77,34,184,192,214,66,163,239,128,165,180,47,7,
    108,116,205,105,14,46,74,112,94,130,246,209,185,65,3,167,
    21,136,255,6,223,108,51,211,121,102,106,192,57,182,22,92,
    90,112,81,134,103,72,132,67,131,10,169,47,142,206,81,83,
    28,105,55,45,148,246,176,160,46,169,226,249,113,232,14,165,
    90,199,190,51,114,99,119,232,60,61,78,84,236,118,213,175,
    228,48,138,199,205,106,70,26,37,123,35,87,157,216,60,215,
    36,163,12,71,138,121,70,161,84,11,216,233,249,161,231,12,
    35,239,44,144,106,158,24,58,61,63,144,142,195,47,127,57,
    28,69,177,250,36,142,163,216,38,187,242,96,16,185,249,12,
    178,106,55,136,18,217,164,213,120,25,155,216,43,162,238,141,
    152,35,9,192,242,210,100,79,38,221,216,31,41,116,151,230,
    72,212,196,173,73,142,226,38,57,194,166,117,18,13,101,43,
    10,124,244,237,203,241,184,53,138,163,62,42,218,234,203,225,
    254,59,137,114,143,3,217,58,62,243,3,175,117,244,193,251,
    173,209,88,157,68,97,107,184,223,242,67,37,209,58,65,235,
    38,187,236,33,221,10,173,240,194,239,59,62,235,230,156,200,
    96,36,227,69,26,189,79,171,139,186,168,137,178,48,69,83,
    44,98,175,132,63,83,108,27,11,226,208,39,237,186,164,49,
    161,203,42,226,137,156,44,224,212,128,120,155,208,50,192,175,
    32,247,34,102,218,244,206,224,119,191,38,179,232,209,129,73,
    24,208,131,231,140,48,132,26,82,30,144,211,67,96,152,148,
    96,80,6,13,31,68,157,198,83,60,166,22,201,137,141,129,
    204,45,72,254,2,104,102,4,206,57,164,160,186,52,65,132,
    117,80,85,138,119,28,221,192,5,127,207,184,108,55,73,252,
    67,134,134,58,241,147,232,69,200,14,160,62,71,82,27,45,
    243,249,248,179,227,129,236,170,100,23,7,190,140,206,26,93,
    55,12,35,213,112,61,175,225,42,21,251,199,103,74,38,13,
    21,53,30,38,77,242,169,189,156,161,43,231,55,30,101,104,
    34,207,35,154,244,131,231,119,21,62,172,242,3,123,33,145,
    10,145,113,18,121,9,142,19,139,190,84,54,9,169,200,200,
    17,11,194,192,113,136,148,150,71,186,59,248,252,52,147,132,
    209,217,44,103,88,74,100,208,83,85,134,165,155,36,14,75,
    66,227,140,64,98,252,220,13,206,36,115,71,24,41,20,136,
    186,90,134,89,97,240,46,233,147,169,207,58,133,81,232,141,
    81,68,191,251,22,173,126,151,145,88,99,44,174,35,14,231,
    176,45,227,255,178,216,48,186,86,138,190,114,134,64,202,136,
    10,216,255,34,133,0,162,241,18,179,79,211,224,244,193,106,
    113,108,190,73,61,154,108,111,83,243,128,154,29,106,118,51,
    205,103,160,254,226,85,245,31,211,146,6,235,204,218,145,155,
    204,76,59,111,42,190,238,77,226,11,19,101,155,226,196,160,
    104,154,196,137,69,73,53,126,66,45,146,114,4,154,144,124,
    65,41,156,226,137,153,81,232,96,16,80,111,18,26,108,43,
    187,78,54,152,207,80,109,19,84,139,120,237,23,240,106,147,
    155,24,172,246,189,44,57,58,68,161,97,106,111,17,171,210,
    13,198,110,80,243,198,12,45,62,1,92,255,26,224,62,164,
    213,235,41,224,22,25,104,85,252,213,141,174,153,186,33,223,
    58,87,175,0,141,80,102,221,128,178,31,80,207,188,174,248,
    236,1,150,170,251,139,2,192,72,66,163,168,213,33,118,198,
    155,164,76,17,90,155,88,10,60,11,55,113,119,55,120,119,
    255,17,239,238,92,33,112,253,164,147,182,201,121,91,119,74,
    100,149,158,9,27,233,174,157,84,176,69,157,94,142,27,81,
    175,161,88,109,202,177,7,15,147,189,135,201,135,152,61,27,
    79,56,111,233,252,169,51,100,44,71,148,225,104,234,39,47,
    187,146,55,74,126,114,28,157,208,28,78,110,78,186,1,35,
    202,168,10,96,15,176,177,57,181,163,61,40,163,207,202,220,
    213,220,220,36,253,167,180,94,149,109,109,138,77,68,84,85,
    176,80,142,78,230,92,144,241,91,252,253,156,236,79,138,75,
    160,178,218,110,107,145,89,27,210,203,126,123,10,53,223,189,
    46,118,11,153,255,38,67,75,121,130,22,250,153,89,12,252,
    17,184,90,21,240,7,32,60,160,219,211,24,200,67,134,0,
    176,74,228,191,5,14,150,27,170,3,206,61,109,170,8,152,
    2,83,82,242,152,73,117,177,240,41,252,169,16,105,217,150,
    110,166,245,104,113,75,183,242,188,197,64,122,173,109,219,154,
    78,112,228,159,19,55,33,50,157,181,38,193,59,217,29,242,
    26,18,179,246,12,80,53,175,87,114,72,168,175,39,152,162,
    77,113,75,172,26,5,164,252,152,154,119,115,144,136,108,236,
    187,149,111,23,110,223,196,29,189,71,124,69,66,88,44,246,
    210,156,174,90,166,216,228,177,80,202,98,225,221,60,22,36,
    111,104,175,248,172,66,173,65,158,191,52,4,30,38,177,170,
    163,179,155,5,178,4,157,50,69,13,151,225,34,13,42,145,
    37,54,74,131,83,187,37,155,230,80,27,45,119,190,246,43,
    53,47,103,149,48,200,181,7,129,59,60,246,220,39,67,90,
    141,150,236,102,97,102,100,242,215,139,242,83,136,136,219,84,
    224,199,253,76,143,231,179,74,22,239,35,243,92,126,14,13,
    47,234,114,134,248,226,68,54,134,114,120,140,7,211,19,127,
    212,232,5,110,159,253,99,166,250,125,150,233,167,216,193,87,
    171,144,228,17,181,81,163,27,133,184,230,89,87,69,113,195,
    147,120,84,147,94,227,157,6,111,4,13,63,105,184,169,68,
    26,242,211,129,203,69,175,27,247,19,174,111,79,95,80,119,
    150,254,117,240,52,238,99,177,31,65,190,245,234,51,98,158,
    215,57,32,116,4,225,202,120,8,83,99,157,197,168,34,177,
    247,168,249,33,204,56,253,191,135,204,3,90,133,140,86,22,
    91,70,197,80,107,215,226,246,115,154,157,92,143,222,191,190,
    78,244,234,75,32,36,144,101,24,204,113,59,79,217,191,83,
    201,6,171,220,46,240,96,45,27,92,228,246,14,15,46,101,
    131,117,110,151,121,112,37,203,11,171,60,184,6,157,245,236,
    106,106,131,114,68,249,191,205,17,28,96,179,12,173,231,255,
    211,212,96,63,254,127,137,111,127,0,105,225,112,91,90,152,
    170,71,159,234,180,160,253,129,229,197,120,141,85,213,199,28,
    86,85,60,11,239,99,97,106,113,97,122,64,133,233,57,23,
    175,142,161,107,211,137,27,249,244,193,247,44,116,4,8,229,
    11,231,38,36,235,2,148,0,226,142,70,50,244,236,71,80,
    172,41,249,245,172,44,71,193,127,1,133,237,222,20,107,88,
    68,94,71,46,229,183,130,166,140,208,82,142,213,237,25,59,
    251,85,230,236,230,230,84,146,179,15,168,169,79,101,52,237,
    144,183,111,73,43,14,38,253,158,195,226,80,61,29,197,74,
    122,116,142,249,207,38,96,77,194,199,214,27,222,169,239,223,
    198,200,15,29,172,22,99,103,232,142,120,197,215,34,164,149,
    232,228,91,24,83,15,110,155,24,158,5,1,179,254,118,10,
    226,201,151,54,248,160,118,110,35,141,221,176,47,153,219,191,
    33,33,118,4,13,126,98,132,41,218,44,61,25,72,37,111,
    140,8,69,30,75,143,252,158,196,247,209,24,79,94,124,140,
    193,231,192,113,102,186,43,253,20,153,159,209,42,116,121,137,
    187,18,30,207,215,245,199,168,148,43,130,55,253,43,247,222,
    90,180,6,100,37,251,56,177,57,21,45,229,248,227,171,217,
    108,223,37,148,242,49,243,208,29,234,27,53,190,42,178,191,
    7,233,113,222,126,43,135,48,221,117,240,57,73,159,75,49,
    169,112,65,194,245,135,253,147,204,184,195,253,189,76,175,61,
    173,23,234,163,47,22,249,114,120,184,207,21,244,117,178,143,
    130,168,123,42,189,148,244,193,237,52,31,71,67,23,199,111,
    94,172,237,103,139,45,95,121,239,197,52,107,253,202,104,34,
    99,223,13,252,111,244,181,101,54,172,200,92,87,165,39,173,
    243,39,174,24,88,134,105,183,49,116,98,217,247,19,228,196,
    108,242,41,105,206,37,167,170,55,110,3,110,113,250,44,177,
    166,107,115,125,185,240,132,194,58,249,24,27,186,135,172,44,
    85,68,217,160,124,108,226,145,126,81,88,102,173,94,177,106,
    11,21,171,50,103,242,165,209,34,30,203,170,86,101,161,38,
    38,159,93,196,103,213,216,173,87,196,191,0,22,63,87,22,
};

EmbeddedPython embedded_m5_internal_param_AbstractMemory(
    "m5/internal/param_AbstractMemory.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_AbstractMemory.py",
    "m5.internal.param_AbstractMemory",
    data_m5_internal_param_AbstractMemory,
    2288,
    7001);

} // anonymous namespace
