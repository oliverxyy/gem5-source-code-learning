#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_StridePrefetcher[] = {
    120,156,197,89,239,110,27,199,17,159,189,59,82,34,37,89,
    255,37,219,146,45,218,142,28,214,104,196,54,141,226,0,113,
    221,58,105,10,52,64,20,231,152,194,14,19,244,122,226,45,
    201,163,200,59,226,110,101,155,129,4,20,149,209,22,232,183,
    2,125,132,126,232,219,228,9,250,22,253,220,206,204,222,29,
    79,148,232,10,104,203,88,228,122,111,110,118,118,103,230,55,
    179,179,203,38,36,255,10,248,253,121,5,32,254,135,0,240,
    240,35,160,7,208,23,208,16,32,164,0,111,13,142,10,16,
    189,7,94,1,94,3,52,12,144,6,156,97,199,132,175,13,
    8,230,121,76,17,122,38,83,4,12,203,32,45,104,20,224,
    89,176,12,150,44,194,81,25,162,223,130,16,34,16,240,220,
    155,1,111,22,94,163,116,236,148,88,224,44,16,177,204,196,
    18,120,115,76,44,131,55,207,157,57,24,46,129,156,135,198,
    2,177,53,174,161,216,7,40,118,145,197,126,71,98,61,124,
    179,14,222,53,98,199,117,125,69,156,22,113,242,124,139,44,
    101,41,93,229,50,52,86,210,254,106,174,191,150,235,175,231,
    250,27,185,254,102,174,127,61,215,191,145,235,223,204,245,183,
    114,253,237,92,255,86,174,127,155,251,168,225,10,116,119,160,
    91,129,238,29,104,161,209,151,51,109,238,130,52,161,123,15,
    26,247,64,226,231,46,156,161,95,188,149,220,136,183,120,196,
    106,54,98,151,71,220,135,198,125,144,248,217,213,35,138,80,
    175,110,160,175,253,127,225,191,42,250,26,212,60,54,47,100,
    20,251,97,224,248,65,43,244,13,122,95,164,134,144,209,164,
    102,38,129,200,199,4,145,191,3,227,195,51,18,136,156,2,
    10,22,164,75,207,128,83,238,156,26,48,172,194,137,128,174,
    5,158,9,39,56,77,129,22,208,22,112,102,192,55,38,49,
    156,98,107,161,35,111,131,165,52,62,186,236,72,45,105,6,
    78,11,112,82,128,250,243,19,131,8,71,37,136,254,6,223,
    110,179,208,89,22,106,192,9,182,22,156,89,112,90,132,103,
    200,132,164,110,137,212,23,207,79,80,83,164,212,171,22,174,
    246,32,167,46,169,226,249,81,224,246,165,218,196,190,51,112,
    35,183,239,212,85,228,123,242,105,36,91,82,53,59,50,170,
    150,83,230,48,222,27,184,170,99,243,104,147,204,210,31,40,
    150,26,6,82,205,97,167,229,7,158,211,15,189,227,158,84,
    179,36,210,105,249,61,233,56,252,242,87,253,65,24,169,79,
    162,40,140,108,178,44,19,123,161,155,141,32,187,54,123,97,
    44,171,52,27,79,99,147,120,69,220,173,1,75,164,5,240,
    138,105,176,39,227,102,228,15,20,58,76,75,36,110,146,86,
    37,87,113,19,55,176,169,117,194,190,172,133,61,31,189,251,
    106,56,172,13,162,176,141,170,214,218,178,191,255,78,172,220,
    195,158,172,29,30,251,61,175,246,252,131,247,107,131,161,234,
    132,65,173,191,95,243,3,37,209,62,189,218,229,150,217,67,
    206,21,154,227,165,223,118,124,214,206,233,200,222,64,70,11,
    68,189,73,243,139,37,49,47,138,194,20,85,177,128,189,2,
    126,77,177,109,204,137,3,159,244,107,146,206,132,48,43,143,
    41,114,180,128,35,3,162,109,66,76,23,63,130,92,140,184,
    169,211,59,131,223,125,65,134,209,212,174,73,56,208,196,19,
    70,25,194,13,57,31,145,227,3,96,168,20,160,91,4,13,
    33,68,158,198,84,52,164,22,217,73,140,129,194,45,136,255,
    10,104,104,4,207,9,36,192,58,51,65,4,75,160,202,148,
    71,144,186,129,19,254,158,177,89,175,210,242,15,24,28,170,
    227,199,225,203,128,93,64,125,142,166,58,90,230,233,240,243,
    195,174,108,170,120,7,9,95,133,199,149,166,27,4,161,170,
    184,158,87,113,21,154,244,240,88,201,184,162,194,202,110,92,
    37,175,218,203,41,190,50,121,195,65,138,39,242,61,226,73,
    63,120,126,83,225,195,42,63,176,23,98,169,16,27,157,208,
    139,145,78,34,218,82,217,180,72,69,70,14,121,33,12,29,
    135,88,105,122,228,187,134,207,79,210,149,48,62,171,197,20,
    77,177,236,181,84,153,129,233,198,177,195,43,33,58,99,144,
    4,191,112,123,199,146,165,35,144,20,46,136,186,122,13,211,
    67,225,117,210,40,53,0,107,21,132,129,55,196,69,250,205,
    183,105,254,235,140,197,121,70,227,58,34,113,6,219,34,254,
    95,20,27,70,211,74,240,87,76,49,72,121,81,1,35,64,
    36,32,64,60,158,97,14,170,26,156,68,88,49,142,207,187,
    212,163,193,246,54,53,183,168,185,77,205,78,170,251,84,12,
    176,48,110,128,135,52,169,193,90,179,126,228,42,51,213,207,
    59,23,99,55,70,49,134,9,179,78,177,98,80,68,141,98,
    197,162,228,26,61,166,22,89,57,10,77,136,191,164,84,78,
    49,197,194,40,124,48,16,168,55,10,15,182,150,189,68,86,
    152,77,145,109,19,92,243,152,109,231,48,107,147,163,24,176,
    246,141,52,69,58,196,161,161,106,111,145,168,194,37,230,174,
    80,115,103,170,54,31,129,174,125,1,116,31,210,252,75,9,
    232,22,24,108,101,252,46,25,77,51,113,68,182,137,174,142,
    129,141,144,102,93,130,180,251,212,51,47,170,254,125,128,44,
    81,248,151,57,144,209,26,141,188,94,7,216,25,110,146,58,
    121,120,109,98,89,240,44,216,196,157,222,224,157,254,71,188,
    211,115,181,192,245,153,78,222,38,231,111,221,41,144,93,90,
    38,108,36,59,120,92,194,22,181,122,53,172,132,173,138,98,
    197,41,215,62,218,141,247,118,227,15,49,139,86,30,115,254,
    210,121,84,103,202,72,14,40,211,209,208,79,94,53,37,111,
    153,252,228,56,58,177,57,156,228,156,100,43,70,164,173,147,
    85,141,212,220,156,226,99,21,81,102,159,158,193,203,153,193,
    105,253,159,210,140,101,182,182,41,54,17,85,101,193,203,114,
    116,90,231,242,140,223,226,247,35,242,0,169,46,129,10,119,
    187,174,23,205,250,144,102,246,15,207,33,103,26,218,216,53,
    20,255,235,20,49,197,17,98,232,107,166,145,240,71,224,234,
    85,192,31,128,48,129,174,79,34,33,11,28,2,193,42,177,
    255,6,56,100,46,169,20,56,7,213,169,58,96,14,76,77,
    241,67,102,213,133,195,167,240,167,92,188,165,219,187,153,212,
    167,249,237,221,202,242,23,131,233,74,91,184,117,62,209,145,
    135,58,110,76,108,58,123,141,66,120,180,79,100,21,37,102,
    239,169,32,107,86,207,229,208,178,190,25,225,138,54,200,45,
    177,106,228,208,242,99,106,222,205,128,34,82,218,255,123,133,
    59,48,121,75,119,244,126,241,53,45,195,226,133,47,206,40,
    242,210,184,160,44,38,10,105,76,188,155,197,132,228,237,237,
    53,159,96,168,53,200,255,103,134,192,99,43,214,121,116,74,
    180,64,22,160,81,164,232,225,210,92,36,193,37,210,20,71,
    9,241,220,222,201,230,57,208,134,203,32,160,189,75,205,171,
    233,165,14,114,240,163,158,219,63,244,220,199,148,35,99,154,
    180,153,134,155,145,106,176,148,215,128,66,69,76,82,130,31,
    247,83,77,94,76,47,109,188,15,188,63,106,13,56,72,188,
    176,201,185,226,203,142,172,244,101,255,16,143,172,29,127,80,
    105,245,220,54,251,200,76,52,252,60,213,80,177,147,199,235,
    146,248,1,181,97,165,25,6,152,217,143,155,42,140,42,158,
    196,35,156,244,42,239,84,120,91,168,248,113,197,61,196,183,
    110,83,105,232,159,15,97,46,133,221,168,29,115,213,123,244,
    146,186,211,245,177,131,39,117,31,15,1,47,32,219,138,245,
    233,49,203,242,92,222,235,72,194,185,241,112,166,134,58,163,
    81,141,98,239,81,243,3,152,250,102,240,94,226,209,152,12,
    87,20,91,70,201,80,27,151,68,240,83,146,16,95,140,227,
    127,94,37,142,245,197,83,18,205,69,226,148,51,116,103,64,
    109,137,182,133,70,57,37,206,113,59,207,196,133,148,120,141,
    219,69,38,46,165,196,101,110,87,152,184,154,18,215,184,93,
    103,226,70,74,220,228,246,58,19,111,164,196,155,220,110,49,
    113,59,37,222,226,246,54,19,119,210,155,181,10,19,239,64,
    227,46,93,13,17,229,30,165,162,153,255,54,21,113,20,79,
    55,126,79,255,167,25,200,126,248,253,41,96,127,0,73,165,
    50,41,251,136,188,118,11,58,251,116,69,122,112,202,171,198,
    87,54,183,38,130,222,105,70,210,85,82,251,108,123,154,42,
    115,54,211,179,255,110,148,87,46,150,248,79,50,237,206,184,
    90,27,174,177,43,245,233,145,93,41,158,5,55,177,214,183,
    184,214,127,68,181,254,9,155,194,49,116,185,63,2,106,33,
    179,8,221,27,5,242,229,133,133,105,171,232,170,158,22,232,
    14,6,50,240,236,7,144,47,212,249,245,244,176,65,57,244,
    207,144,171,159,76,177,134,149,249,197,248,164,141,34,167,45,
    251,180,144,69,228,84,189,203,128,254,75,10,232,234,91,144,
    223,45,236,71,212,240,254,144,109,13,246,207,50,223,220,153,
    140,86,79,182,35,41,233,104,120,5,46,172,233,216,135,250,
    81,221,155,60,160,239,190,114,112,135,110,177,224,171,240,145,
    104,218,25,83,194,27,7,249,193,213,132,167,124,153,240,132,
    160,118,39,15,66,39,69,106,36,254,106,156,52,1,97,105,
    68,82,247,39,15,100,24,56,88,168,132,77,158,227,138,172,
    52,9,221,172,228,104,111,90,158,102,195,65,241,127,82,36,
    199,153,42,50,34,189,113,117,157,72,198,157,145,177,174,200,
    154,41,50,162,241,249,105,194,208,227,88,34,78,98,12,16,
    199,247,120,158,43,51,211,76,148,204,207,81,57,252,57,131,
    123,178,39,149,156,144,178,248,104,146,92,118,121,18,171,201,
    112,232,56,250,232,142,207,61,199,153,114,237,245,83,189,229,
    66,76,105,7,107,47,81,196,234,107,93,156,251,51,74,197,
    146,224,66,119,236,87,32,189,76,186,84,213,7,214,97,108,
    243,190,184,152,229,8,254,153,34,173,52,41,157,240,69,203,
    129,219,215,119,203,124,101,106,83,176,241,149,150,253,118,150,
    107,232,198,143,111,9,244,205,12,238,1,92,132,115,205,109,
    255,132,232,52,180,191,191,151,234,184,167,117,252,226,88,30,
    75,111,164,35,255,94,210,223,87,149,75,185,63,114,227,156,
    61,248,22,239,34,83,125,136,46,238,171,173,177,151,50,56,
    238,59,159,201,126,24,13,63,11,61,169,182,199,222,63,241,
    188,200,118,131,182,116,94,72,58,79,112,38,60,199,144,28,
    38,180,140,148,235,242,133,158,231,189,176,22,205,132,47,245,
    207,11,124,96,190,248,254,227,94,216,60,146,94,194,115,107,
    50,207,47,194,190,139,244,203,103,169,251,233,44,203,99,239,
    189,136,70,173,143,81,99,25,249,110,207,255,86,255,106,145,
    146,185,192,159,224,50,10,145,113,34,31,18,46,45,144,56,
    150,34,217,246,41,16,89,236,248,216,164,88,32,168,191,41,
    177,231,229,76,55,10,245,9,93,95,55,62,166,28,198,113,
    68,191,79,148,22,75,24,145,84,74,152,162,140,197,132,101,
    206,47,149,172,249,185,146,85,154,49,249,34,121,65,172,26,
    101,171,52,55,47,38,253,237,96,244,150,141,157,205,146,248,
    55,127,92,179,27,
};

EmbeddedPython embedded_m5_internal_param_StridePrefetcher(
    "m5/internal/param_StridePrefetcher.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_StridePrefetcher.py",
    "m5.internal.param_StridePrefetcher",
    data_m5_internal_param_StridePrefetcher,
    2533,
    8153);

} // anonymous namespace
