#include "sim/init.hh"

namespace {

const uint8_t data_m5_util_smartdict[] = {
    120,156,213,88,109,111,27,69,16,222,59,191,36,118,210,54,
    73,211,180,208,183,165,188,153,210,198,80,90,104,171,10,161,
    182,42,170,128,0,151,210,210,168,112,186,220,174,237,75,206,
    119,238,237,94,18,163,246,11,32,33,33,129,196,7,36,126,
    13,127,128,31,5,51,115,47,182,83,183,200,106,79,86,157,
    120,52,217,187,204,62,51,243,204,220,237,184,44,253,148,224,
    251,9,103,76,121,160,8,248,53,152,207,216,29,212,76,230,
    27,172,107,176,13,131,25,162,196,164,201,90,6,19,101,246,
    51,99,63,49,118,127,163,196,68,133,201,50,173,86,243,213,
    10,19,51,76,26,180,58,155,175,86,113,181,13,182,102,152,
    168,177,245,70,29,118,243,254,133,79,195,0,77,163,56,155,
    168,179,32,28,173,35,225,185,154,254,184,235,68,158,179,233,
    75,55,131,140,183,93,71,200,255,128,34,25,226,3,184,27,
    38,98,135,253,16,151,73,74,153,1,108,84,42,41,108,132,
    81,33,101,38,133,188,49,139,192,80,169,165,112,55,234,136,
    16,149,57,38,234,164,204,51,49,71,202,1,38,230,73,57,
    200,196,1,82,14,49,113,144,148,5,38,14,145,178,200,196,
    2,41,75,76,44,146,114,152,137,37,82,150,153,56,76,202,
    17,38,150,73,89,97,226,8,41,71,153,181,222,88,1,127,
    212,111,32,110,7,90,250,190,215,150,129,230,189,40,220,235,
    115,215,119,148,226,173,48,226,235,93,39,210,55,33,56,171,
    156,103,161,225,187,158,239,243,88,73,174,59,178,206,225,179,
    3,87,194,88,113,55,12,118,100,164,121,43,14,92,237,133,
    129,226,58,228,16,94,217,237,105,84,179,235,59,142,31,75,
    186,8,86,208,36,89,209,253,158,84,174,145,198,28,169,114,
    3,227,190,128,121,162,172,61,130,244,26,248,187,78,201,91,
    107,152,120,169,134,34,68,39,218,96,27,255,77,233,40,201,
    110,25,255,144,126,171,129,137,36,161,110,128,104,118,194,174,
    108,134,190,7,96,246,250,253,38,56,221,142,156,110,179,45,
    187,151,206,43,141,128,154,42,114,155,189,190,238,132,65,179,
    123,169,25,107,207,111,42,140,5,18,101,21,46,204,128,29,
    219,246,2,109,219,23,209,174,73,92,153,28,125,149,208,127,
    30,6,109,11,209,210,37,171,146,193,125,177,152,103,9,179,
    15,123,217,246,135,207,3,122,134,64,223,242,67,71,23,143,
    186,70,168,91,184,153,109,95,126,254,88,95,15,67,191,120,
    212,115,132,58,8,131,31,100,20,218,246,213,33,220,102,138,
    217,204,112,7,9,238,71,132,248,177,201,30,153,232,195,182,
    201,162,51,3,103,214,113,213,164,213,203,168,12,173,150,104,
    117,13,149,161,213,50,173,10,84,210,85,93,201,180,52,42,
    213,172,72,176,244,72,217,132,224,80,13,1,179,105,1,217,
    162,49,70,173,65,182,75,89,220,232,66,8,93,32,162,158,
    170,11,170,179,180,109,92,67,187,203,20,195,121,248,169,143,
    124,41,170,230,112,84,15,131,2,238,110,101,129,5,185,205,
    114,215,41,247,104,140,168,65,222,88,75,197,80,161,154,148,
    29,176,247,211,125,44,248,95,188,198,20,241,74,219,190,61,
    41,94,115,122,120,229,67,219,254,108,82,188,165,233,225,13,
    32,190,95,76,138,183,60,61,188,109,224,239,151,147,226,173,
    76,17,47,196,247,235,103,225,93,124,18,239,209,41,160,77,
    94,36,28,33,108,251,206,132,112,143,77,13,174,138,55,109,
    251,238,132,112,151,167,6,183,27,251,182,253,237,132,112,143,
    79,13,174,240,118,108,123,227,37,128,155,188,228,232,40,150,
    4,249,193,24,200,165,17,200,198,8,234,105,148,91,242,14,
    28,81,189,217,19,226,157,70,189,165,120,169,224,156,9,241,
    78,163,224,82,188,84,113,238,132,120,167,65,225,20,47,241,
    87,190,4,120,231,19,188,121,205,181,7,152,27,199,88,58,
    209,128,183,11,167,11,207,63,93,167,63,186,161,136,241,117,
    46,107,47,161,107,219,22,62,37,45,188,217,194,155,172,249,
    204,5,235,8,10,28,14,88,71,81,160,81,235,21,20,175,
    162,56,142,226,4,138,147,40,78,161,56,141,130,163,120,13,
    197,25,20,175,163,120,99,36,6,47,52,16,22,250,252,30,
    26,60,7,162,90,170,26,53,51,253,153,203,53,179,86,122,
    82,211,152,201,111,2,33,91,94,32,197,211,231,61,232,253,
    216,121,143,181,78,105,86,127,131,248,202,119,92,217,9,125,
    33,163,116,106,162,67,30,201,94,36,21,142,83,226,108,27,
    26,145,224,62,106,149,243,123,158,239,211,196,163,45,3,25,
    57,190,223,231,174,131,243,20,39,224,114,207,149,61,156,157,
    240,221,14,92,133,0,113,79,115,79,225,164,68,156,227,155,
    177,230,18,199,39,142,166,1,10,153,193,35,38,77,107,240,
    244,38,193,8,176,67,119,56,220,161,189,160,205,85,236,118,
    184,163,184,23,224,6,94,139,67,132,181,236,2,190,252,8,
    109,100,44,167,195,224,232,41,137,14,121,183,28,95,201,34,
    15,204,68,192,96,136,204,136,196,122,19,197,91,40,222,206,
    72,90,20,159,208,218,22,75,67,128,124,162,193,67,62,255,
    202,249,97,102,252,120,240,52,126,76,58,15,196,73,160,65,
    35,65,26,87,162,94,71,146,97,93,170,62,8,220,31,24,
    225,68,217,100,78,119,28,205,145,117,10,114,25,65,142,85,
    194,12,111,48,201,67,78,37,199,102,149,12,215,58,33,16,
    44,189,27,73,74,199,246,100,230,198,133,236,201,64,32,87,
    128,119,176,238,69,64,55,167,45,243,46,88,206,248,113,62,
    239,130,219,85,26,79,208,184,101,139,198,17,216,20,205,100,
    208,80,194,57,236,186,192,227,43,149,74,15,68,36,117,28,
    5,138,59,131,65,98,50,109,4,70,194,158,217,80,80,238,
    121,74,19,89,113,81,56,144,61,135,74,67,144,31,248,201,
    45,5,252,137,58,78,252,218,245,148,92,75,40,68,147,31,
    212,104,196,139,237,188,45,53,101,123,208,172,105,121,91,246,
    11,123,69,130,45,61,40,57,219,142,209,54,182,94,86,158,
    55,86,170,46,110,140,223,74,22,224,147,73,1,110,25,105,
    68,53,141,131,112,152,85,98,198,32,160,247,25,141,104,100,
    132,221,130,34,165,164,166,106,15,91,16,151,126,222,111,184,
    10,19,186,236,66,12,253,93,167,175,242,48,42,29,70,3,
    74,100,1,7,220,107,201,132,231,93,132,146,224,87,25,254,
    125,3,32,235,124,22,93,188,92,64,85,54,193,206,143,121,
    87,40,81,195,26,33,36,54,135,246,80,200,32,82,55,191,
    91,98,143,147,200,209,99,250,123,147,61,92,200,91,218,192,
    53,76,67,66,58,34,201,16,29,240,206,157,2,188,121,31,
    236,252,50,232,113,185,55,136,201,69,111,222,1,101,239,204,
    62,111,22,71,188,185,107,128,51,247,4,27,227,80,61,201,
    67,52,222,169,74,230,110,1,142,125,0,118,126,205,154,39,
    51,86,18,215,70,136,125,97,92,162,78,177,7,38,122,247,
    216,28,225,122,203,124,106,210,42,41,215,82,247,6,163,72,
    188,113,219,186,80,204,195,225,18,216,249,125,95,226,74,41,
    13,41,113,120,195,222,234,62,239,78,62,213,187,103,36,177,
    150,38,113,156,147,84,110,214,197,98,124,252,8,236,252,49,
    200,225,9,195,66,54,230,29,170,154,37,242,52,99,251,219,
    62,14,158,179,38,53,52,94,206,251,47,121,102,225,139,218,
    216,254,129,175,165,208,198,157,216,47,96,130,76,219,254,57,
    148,187,23,230,16,213,27,116,198,20,250,56,215,172,203,197,
    228,234,10,216,249,107,224,83,3,187,244,254,151,165,85,20,
    216,63,169,237,80,137,18,143,41,209,73,50,174,176,226,222,
    167,206,130,157,135,104,16,15,66,85,179,106,214,14,212,42,
    240,22,94,198,239,124,105,173,65,103,15,58,113,24,25,49,
    146,135,53,38,38,220,220,146,110,242,152,38,75,217,152,204,
    135,99,101,65,136,233,228,116,45,57,43,125,140,201,83,152,
    223,186,177,96,174,92,93,153,89,57,251,31,133,225,162,100,
};

EmbeddedPython embedded_m5_util_smartdict(
    "m5/util/smartdict.py",
    "/home/oliverxyy/program/gem5-stable/src/python/m5/util/smartdict.py",
    "m5.util.smartdict",
    data_m5_util_smartdict,
    1824,
    7877);

} // anonymous namespace