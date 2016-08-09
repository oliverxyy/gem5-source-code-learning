#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_NativeTrace[] = {
    120,156,197,88,255,111,219,198,21,127,71,82,178,37,203,177,
    18,199,118,18,59,182,214,33,171,22,172,214,214,213,77,129,
    122,193,218,46,3,90,12,78,71,165,72,170,22,227,104,241,
    36,81,166,72,129,60,39,86,33,111,192,28,108,251,7,246,
    39,236,135,253,55,251,143,182,247,222,145,20,101,59,64,129,
    46,154,35,94,142,199,187,119,239,203,231,125,185,235,66,250,
    87,194,231,215,13,128,228,79,2,192,195,159,128,0,96,36,
    160,35,64,72,1,222,109,56,41,65,252,1,120,37,120,13,
    208,49,64,26,112,129,29,19,190,49,32,172,241,154,50,4,
    38,143,8,152,84,65,90,208,41,193,243,240,38,88,178,12,
    39,85,136,255,8,66,136,80,192,11,111,9,188,101,120,141,
    212,177,83,97,130,203,64,131,85,30,172,128,183,194,131,85,
    240,106,220,89,129,73,29,100,13,58,171,52,173,115,3,201,
    62,68,178,107,76,246,223,68,214,195,47,27,224,221,160,233,
    200,215,215,52,211,162,153,188,223,26,83,169,103,92,222,132,
    206,173,172,191,94,232,223,46,244,55,10,253,77,238,35,7,
    183,96,184,5,195,59,48,188,11,61,84,202,205,124,183,123,
    32,77,24,110,67,103,27,36,254,238,193,5,234,205,187,85,
    88,177,195,43,214,243,21,247,121,197,46,116,118,65,226,239,
    190,94,81,134,118,115,19,109,225,255,7,255,154,104,11,80,
    53,108,94,202,56,241,163,208,241,195,94,228,27,244,189,76,
    13,89,174,75,205,82,106,194,207,200,132,255,2,182,159,103,
    164,38,60,7,36,44,72,150,192,128,115,238,156,27,48,105,
    194,84,192,208,2,207,132,41,110,83,34,6,250,2,46,12,
    248,214,164,9,231,216,90,168,232,93,176,148,182,223,144,21,
    173,41,45,193,121,9,166,37,104,191,152,26,52,112,82,129,
    248,159,240,221,14,19,93,102,162,6,76,177,181,224,194,130,
    243,50,60,199,73,56,52,172,144,248,226,197,20,37,197,145,
    118,211,66,110,143,10,226,146,40,158,31,135,238,72,170,91,
    216,119,198,110,236,142,156,35,87,249,47,229,179,216,237,202,
    102,53,155,23,37,251,99,87,13,108,94,104,146,70,70,99,
    197,4,163,80,170,21,236,244,252,208,115,70,145,119,26,72,
    181,76,212,156,158,31,72,199,225,143,159,143,198,81,172,158,
    196,113,20,219,164,84,30,12,34,55,95,65,42,237,6,81,
    34,155,180,27,111,99,19,121,69,179,123,99,166,72,12,48,
    179,180,216,147,73,55,246,199,10,109,165,41,210,108,162,214,
    36,43,113,147,124,133,77,107,16,141,100,43,10,80,164,248,
    108,50,105,141,227,168,143,82,182,250,114,116,240,94,162,220,
    227,64,182,142,79,253,192,107,189,248,232,195,214,120,162,6,
    81,216,26,29,180,252,80,73,84,77,208,186,162,148,125,156,
    68,234,74,94,249,125,199,103,193,156,129,12,198,50,94,165,
    209,123,180,181,168,139,154,40,11,83,52,197,42,246,74,248,
    152,98,199,88,17,71,62,137,214,37,113,9,87,86,17,73,
    100,94,1,39,6,196,59,132,147,33,254,4,25,22,209,210,
    166,111,6,127,251,61,233,68,143,14,77,178,190,30,156,50,
    182,16,100,56,243,144,204,29,2,3,164,4,195,50,104,224,
    32,222,52,146,226,9,181,56,157,200,24,72,220,130,228,31,
    128,58,70,200,76,33,133,211,133,9,34,172,131,170,146,119,
    227,232,38,110,248,23,70,100,187,73,236,31,49,46,212,192,
    79,162,87,33,107,159,250,236,67,109,212,204,151,147,167,199,
    67,217,85,201,30,14,124,29,157,54,186,110,24,70,170,225,
    122,94,195,85,42,246,143,79,149,76,26,42,106,60,72,154,
    100,80,251,102,6,173,156,222,100,156,65,137,204,142,80,210,
    47,158,223,85,248,178,206,47,108,133,68,42,132,197,32,242,
    18,28,39,18,125,169,108,98,82,145,146,35,102,132,81,227,
    208,84,218,30,231,221,192,247,79,50,78,24,154,205,114,6,
    164,68,6,61,85,101,76,186,73,226,48,39,52,206,240,35,
    194,47,221,224,84,50,117,196,144,66,134,168,171,121,88,8,
    0,239,144,48,153,236,44,80,24,133,222,4,249,243,187,239,
    210,214,119,24,134,53,6,226,6,130,112,9,219,50,254,95,
    22,155,70,215,74,161,87,206,224,71,129,80,1,27,95,164,
    246,71,40,94,96,208,105,26,28,53,88,38,246,202,119,168,
    71,139,237,29,106,238,83,179,75,205,94,38,246,219,150,125,
    245,178,236,143,104,63,131,5,102,209,200,64,102,38,154,55,
    231,89,119,103,158,133,193,177,77,30,98,144,31,205,60,196,
    162,64,26,63,166,22,167,178,239,153,144,60,163,176,77,158,
    196,196,200,105,16,254,212,155,57,5,43,202,174,147,2,150,
    51,60,219,4,210,34,82,251,5,164,218,100,35,134,169,125,
    55,139,137,14,205,208,0,181,183,137,84,233,26,77,55,168,
    249,209,162,212,61,131,90,255,10,212,62,166,173,235,41,212,
    86,25,98,85,124,234,70,215,76,109,144,231,202,245,75,16,
    35,124,89,215,224,235,39,212,51,175,74,189,96,104,165,178,
    254,182,0,45,98,207,40,138,116,132,157,201,22,73,82,4,
    213,22,38,254,231,225,22,230,114,131,115,249,207,57,151,115,
    61,192,21,146,14,212,38,199,106,221,41,145,74,122,38,108,
    166,57,58,169,96,139,2,157,77,26,81,175,161,88,102,138,
    171,135,15,146,253,7,201,199,24,49,27,143,57,86,233,152,
    169,163,98,44,199,20,213,104,233,147,179,174,228,204,200,111,
    142,163,131,152,195,1,205,73,51,46,226,107,131,20,106,100,
    154,230,112,158,168,152,162,248,66,116,93,205,117,77,172,127,
    65,155,85,89,209,166,216,66,44,85,5,115,228,232,232,205,
    181,23,127,197,231,83,82,62,73,45,129,170,102,187,173,249,
    101,81,72,40,251,103,115,120,121,203,130,216,45,164,252,85,
    134,147,242,12,39,244,152,25,244,255,6,92,149,10,248,43,
    16,18,208,224,41,244,115,79,33,211,175,211,244,63,0,251,
    200,53,181,0,199,155,54,229,127,158,129,97,40,121,196,83,
    117,105,240,5,252,189,224,96,89,2,55,211,186,179,152,192,
    173,60,86,49,132,190,87,146,182,230,131,26,25,103,224,38,
    52,77,71,170,153,207,206,210,65,94,46,98,164,126,219,120,
    90,214,219,56,196,209,183,51,52,81,10,220,22,235,70,1,
    35,191,160,230,253,28,30,34,27,123,139,204,237,193,155,243,
    181,163,51,194,55,196,129,197,60,175,45,113,212,229,213,206,
    103,79,127,247,244,168,93,36,152,59,67,41,115,134,247,115,
    103,144,156,200,94,243,185,132,90,131,172,127,97,8,60,44,
    98,29,71,103,51,11,100,9,58,101,114,27,174,186,69,234,
    85,34,11,107,20,4,231,178,36,107,232,72,235,46,7,128,
    182,45,53,103,11,9,23,100,222,195,192,29,29,123,238,227,
    128,182,162,253,186,153,159,25,25,243,245,34,243,228,35,226,
    77,252,243,235,65,38,196,203,133,132,138,15,145,114,206,60,
    59,134,23,117,57,62,60,27,200,198,72,142,142,241,248,57,
    240,199,141,94,224,246,217,50,102,42,220,211,76,56,197,166,
    189,92,119,36,15,169,141,26,221,40,196,24,126,218,85,81,
    220,240,36,158,201,164,215,120,175,193,9,160,225,39,13,247,
    24,191,186,93,165,49,63,239,182,92,224,186,113,63,225,90,
    246,228,21,117,23,102,89,7,15,220,62,86,245,33,228,249,
    86,159,4,243,120,206,245,186,118,33,220,22,79,91,106,162,
    3,24,213,32,246,62,53,63,133,69,134,253,15,72,75,180,
    5,169,171,44,182,141,138,161,8,93,133,105,95,210,186,228,
    170,187,254,230,251,184,171,190,213,73,157,182,12,114,137,78,
    251,116,99,83,74,111,108,208,129,203,63,212,129,217,1,22,
    6,253,211,255,169,223,218,143,254,47,188,219,31,65,154,211,
    223,228,179,115,69,226,39,218,103,181,37,48,243,79,110,179,
    156,250,212,193,114,138,231,225,61,172,22,45,174,22,15,169,
    90,156,114,69,233,24,186,96,156,25,144,15,3,124,219,65,
    110,17,202,87,206,21,176,233,146,144,112,225,142,199,50,244,
    236,135,80,172,242,248,243,66,116,70,110,121,6,133,52,108,
    138,219,88,214,93,69,43,133,157,130,140,140,202,82,142,207,
    157,69,218,248,207,153,141,245,141,68,30,123,236,67,106,234,
    115,129,70,199,29,54,6,229,106,79,6,82,201,171,246,80,
    180,42,61,252,121,18,163,111,52,193,74,156,43,91,124,15,
    28,103,113,1,235,87,26,137,218,34,24,176,68,217,168,148,
    43,130,51,192,165,171,78,205,15,21,45,186,122,155,36,54,
    67,127,45,151,153,47,228,178,56,76,234,225,179,198,145,59,
    210,87,41,124,77,96,255,24,210,3,157,253,110,174,59,58,
    234,114,201,172,15,39,8,98,206,78,156,140,236,95,210,56,
    245,70,7,251,153,48,251,90,152,39,103,90,146,152,175,4,
    71,7,106,231,218,105,159,99,254,75,231,93,79,167,237,143,
    244,205,20,7,235,226,119,47,118,177,191,113,105,52,145,177,
    239,6,254,119,250,42,42,27,86,164,137,203,140,209,45,202,
    92,205,150,127,226,100,161,30,92,254,94,48,16,35,36,150,
    125,63,193,13,152,122,190,56,117,108,54,223,46,92,147,96,
    230,214,46,12,79,186,34,211,167,201,199,228,47,201,167,216,
    208,125,83,101,173,130,216,34,143,55,241,24,183,42,44,179,
    86,175,88,181,149,138,85,89,50,249,138,96,21,11,242,170,
    85,89,169,137,236,223,30,34,177,106,236,213,42,226,191,8,
    167,158,81,
};

EmbeddedPython embedded_m5_internal_param_NativeTrace(
    "m5/internal/param_NativeTrace.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_NativeTrace.py",
    "m5.internal.param_NativeTrace",
    data_m5_internal_param_NativeTrace,
    2163,
    6442);

} // anonymous namespace