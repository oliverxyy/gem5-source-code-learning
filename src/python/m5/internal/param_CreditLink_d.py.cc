#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_CreditLink_d[] = {
    120,156,197,88,109,115,219,198,17,222,3,64,74,164,36,75,
    178,44,201,182,100,11,105,199,13,227,105,196,54,141,226,204,
    88,113,155,58,233,76,50,137,146,130,105,236,48,153,162,16,
    112,162,64,129,0,7,56,89,102,134,250,82,121,218,78,191,
    247,39,244,67,255,77,255,81,186,187,7,128,208,219,76,102,
    90,179,50,113,62,28,238,246,246,229,217,151,59,31,242,191,
    26,62,191,177,1,178,191,11,128,0,127,2,34,128,129,128,
    174,0,33,5,4,183,224,168,6,233,187,16,212,224,21,64,
    215,0,105,192,25,118,76,248,214,128,120,158,215,212,33,50,
    121,68,192,168,9,210,130,110,13,158,197,203,96,201,58,28,
    53,33,253,19,8,33,98,1,207,131,25,8,102,225,21,82,
    199,78,131,9,206,2,13,54,121,176,1,193,28,15,54,33,
    152,231,206,28,140,150,64,206,67,119,129,166,117,111,32,217,
    135,72,118,145,201,254,155,200,6,248,101,21,130,27,52,29,
    249,250,134,102,90,52,147,247,91,100,42,75,5,151,203,208,
    189,89,244,87,42,253,91,149,254,106,165,191,86,233,175,115,
    31,185,185,9,253,219,208,191,3,253,187,112,128,10,90,46,
    119,222,0,105,66,127,19,186,155,32,241,183,1,103,168,195,
    224,102,101,197,61,94,177,82,174,184,207,43,182,160,187,5,
    18,127,247,245,138,58,116,90,107,104,151,240,7,252,107,161,
    93,64,205,99,243,66,166,89,152,196,110,24,31,36,161,65,
    223,235,212,144,21,125,106,102,114,115,62,37,115,254,11,216,
    150,129,145,155,243,20,144,176,32,89,34,3,78,185,115,106,
    192,168,5,99,1,125,11,2,19,198,184,77,141,24,232,9,
    56,51,224,59,147,38,156,98,107,161,210,239,131,165,180,45,
    251,172,116,77,105,6,78,107,48,174,65,231,249,216,160,129,
    163,6,164,255,132,239,55,153,232,44,19,53,96,140,173,5,
    103,22,156,214,225,25,78,194,161,126,131,196,23,207,199,40,
    41,142,116,90,22,114,187,87,17,151,68,9,194,52,246,6,
    82,173,96,223,29,122,169,55,112,159,166,50,8,213,103,97,
    124,228,6,173,102,49,49,201,182,135,158,58,116,120,165,73,
    42,25,12,21,83,76,98,169,230,176,115,16,198,129,59,72,
    130,227,72,170,89,34,231,30,132,145,116,93,254,248,201,96,
    152,164,234,227,52,77,82,135,180,202,131,81,226,149,43,72,
    167,126,148,100,178,69,187,241,54,14,145,87,52,251,96,200,
    20,137,1,230,150,22,7,50,243,211,112,168,208,88,154,34,
    205,38,106,45,50,19,55,217,215,216,180,15,147,129,108,39,
    81,136,150,125,57,26,181,135,105,210,67,49,219,61,57,216,
    121,59,83,222,126,36,219,251,199,97,20,180,159,191,255,94,
    123,56,82,135,73,220,30,236,180,195,88,73,212,77,212,190,
    172,149,109,156,117,147,232,159,132,61,55,100,201,220,67,25,
    13,101,186,64,163,119,105,111,177,36,230,69,93,152,162,37,
    22,176,87,195,199,20,155,198,156,216,11,73,54,159,228,37,
    100,89,85,44,145,129,5,28,25,144,110,18,82,250,248,19,
    100,90,196,75,135,190,25,252,237,247,164,20,61,218,55,201,
    254,122,112,204,232,66,152,225,204,93,50,120,12,12,145,26,
    244,235,160,161,131,136,211,88,74,71,212,226,116,34,99,32,
    113,11,178,127,0,42,25,65,51,134,28,80,103,38,136,120,
    9,84,147,124,29,71,215,112,195,63,51,38,59,45,98,127,
    143,129,161,14,195,44,57,137,89,253,212,103,47,234,160,102,
    190,28,125,177,223,151,190,202,182,112,224,155,228,216,246,189,
    56,78,148,237,5,129,237,41,149,134,251,199,74,102,182,74,
    236,7,89,139,44,234,44,23,216,42,233,141,134,5,150,200,
    238,136,37,253,18,132,190,194,23,6,173,203,86,200,164,66,
    92,28,38,65,134,227,68,162,39,149,67,76,42,82,114,194,
    140,48,108,92,154,74,219,227,188,27,248,254,97,193,9,99,
    179,85,47,144,148,201,232,64,53,25,148,94,150,185,204,9,
    141,51,254,136,240,11,47,58,150,76,29,65,164,144,33,234,
    106,30,166,131,192,219,36,77,33,60,75,20,39,113,48,66,
    6,67,255,77,218,251,54,227,112,158,145,184,138,40,156,193,
    182,142,255,215,197,154,225,91,57,246,234,5,254,40,22,42,
    96,235,139,28,0,136,197,51,140,59,45,131,3,7,11,197,
    126,249,19,234,209,98,103,147,154,123,212,220,167,102,171,144,
    251,181,11,191,112,81,248,71,180,161,193,18,179,108,100,34,
    179,144,45,56,231,91,119,38,190,133,1,178,67,62,98,144,
    39,77,124,196,162,96,154,62,161,22,167,178,247,153,144,125,
    69,161,155,124,137,137,145,219,160,3,80,111,226,22,172,41,
    103,137,52,48,91,32,218,33,152,86,177,218,171,96,213,33,
    35,49,80,157,59,69,88,116,105,134,134,168,179,65,164,106,
    87,168,218,166,230,141,169,233,123,2,182,222,37,176,61,166,
    189,151,114,176,45,48,200,154,248,44,25,190,153,27,161,76,
    152,43,23,64,70,8,179,174,64,216,207,168,103,94,22,123,
    218,224,202,133,253,93,5,92,196,159,81,149,105,15,59,163,
    117,18,165,10,171,117,76,255,207,226,117,204,232,6,103,244,
    95,112,70,231,170,128,107,38,29,172,77,142,215,186,83,35,
    157,28,152,176,150,103,234,172,129,45,74,244,114,100,39,7,
    182,98,161,41,182,238,62,200,182,31,100,143,49,106,218,79,
    56,94,233,184,169,35,99,42,135,20,217,104,233,199,47,125,
    201,233,145,223,92,87,7,50,151,131,154,155,167,93,68,216,
    42,105,212,40,84,205,33,61,83,41,69,242,233,40,187,89,
    42,155,120,255,148,118,107,178,166,77,177,142,104,106,10,102,
    201,213,33,156,75,48,254,138,207,111,73,251,36,182,4,42,
    164,157,142,102,152,101,33,169,156,159,159,67,204,235,150,196,
    105,35,233,63,20,72,169,79,144,66,143,89,160,255,175,192,
    213,169,128,191,0,97,1,77,158,163,191,116,22,50,254,10,
    77,255,35,176,155,92,81,17,112,204,233,80,21,192,51,48,
    20,101,143,120,170,46,16,62,133,191,85,124,172,72,227,102,
    94,127,86,211,184,85,198,43,6,209,143,74,213,214,249,192,
    70,214,57,244,50,154,166,163,213,196,109,39,57,161,172,26,
    49,90,191,118,68,205,234,125,92,98,233,187,9,158,40,17,
    110,136,21,163,130,146,95,82,243,78,9,16,81,140,189,78,
    238,182,224,250,180,237,234,188,240,45,177,96,49,211,139,51,
    92,79,85,137,148,62,80,43,124,224,157,210,7,36,167,176,
    87,124,42,161,214,32,155,159,25,2,143,141,88,195,209,41,
    205,2,89,131,110,157,188,133,75,110,145,59,147,40,194,25,
    5,191,115,249,145,213,178,167,21,86,154,93,91,148,154,151,
    211,9,19,100,212,221,200,27,236,7,222,147,1,237,69,27,
    250,133,123,25,5,247,75,85,238,201,53,196,117,2,240,235,
    78,33,197,139,233,132,136,247,144,116,201,61,59,68,144,248,
    28,23,190,58,148,246,64,14,246,241,248,121,24,14,237,131,
    200,235,177,109,204,92,186,47,10,233,20,27,247,98,205,145,
    61,164,54,177,253,36,198,232,125,236,171,36,181,3,137,71,
    50,25,216,111,219,28,250,237,48,179,189,125,252,234,249,74,
    67,253,188,187,114,121,235,165,189,140,43,217,163,19,234,78,
    207,182,46,158,184,67,44,234,19,40,83,173,62,9,150,145,
    156,203,117,237,57,184,47,30,182,212,72,71,46,170,63,156,
    109,106,222,130,169,6,252,119,145,116,68,123,144,194,234,98,
    195,104,24,124,46,172,206,251,146,86,102,151,125,246,243,31,
    227,179,250,146,39,247,220,122,113,59,52,3,146,15,106,116,
    129,83,207,47,112,208,157,103,254,91,119,102,111,152,158,31,
    188,248,159,122,177,243,232,255,195,188,243,62,228,153,253,58,
    15,22,85,201,22,180,7,247,69,113,176,168,138,197,215,24,
    119,174,4,144,235,167,210,83,82,219,105,115,90,162,114,36,
    208,59,143,38,126,121,185,4,254,176,148,234,140,171,154,209,
    45,54,159,62,85,177,249,196,179,248,46,214,194,22,215,194,
    187,84,11,143,89,5,174,161,203,225,9,48,107,165,38,232,
    68,26,203,19,247,178,54,116,197,75,204,121,195,161,140,3,
    231,33,84,139,88,254,60,29,44,60,214,72,157,212,24,166,
    184,133,85,235,101,63,164,224,90,145,146,237,88,43,61,111,
    106,22,101,240,158,21,224,109,81,109,49,137,176,206,46,53,
    28,83,203,112,234,252,26,138,16,91,194,51,144,145,84,242,
    10,187,40,90,155,159,114,3,137,169,38,25,225,129,131,235,
    119,124,143,92,119,138,193,249,3,36,125,12,249,153,9,131,
    179,168,27,13,179,81,111,8,206,120,23,174,118,53,75,54,
    20,85,234,40,115,216,185,23,75,185,249,254,177,72,59,164,
    39,62,85,237,121,3,125,113,196,119,34,206,79,33,63,187,
    58,111,150,74,36,133,241,209,64,31,195,16,208,156,141,57,
    249,58,191,162,113,170,8,7,59,219,133,60,219,90,158,61,
    169,78,146,244,72,11,196,183,160,131,157,107,166,62,141,18,
    255,72,6,250,186,77,221,187,126,206,71,201,192,195,241,141,
    43,103,116,194,65,78,97,249,194,247,32,165,85,171,23,70,
    51,153,134,94,20,126,175,111,232,138,97,190,232,184,74,2,
    138,124,231,70,56,125,94,10,119,12,158,84,246,194,12,233,
    49,177,115,139,114,255,255,160,208,219,21,145,178,74,96,122,
    120,211,5,170,62,85,63,161,91,156,236,35,108,232,234,173,
    177,216,64,236,81,100,48,241,52,187,32,44,115,126,169,97,
    205,207,53,172,198,140,201,119,37,11,120,42,105,90,141,185,
    121,49,249,183,133,72,109,26,91,184,246,63,108,171,229,199,
};

EmbeddedPython embedded_m5_internal_param_CreditLink_d(
    "m5/internal/param_CreditLink_d.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_CreditLink_d.py",
    "m5.internal.param_CreditLink_d",
    data_m5_internal_param_CreditLink_d,
    2208,
    6726);

} // anonymous namespace