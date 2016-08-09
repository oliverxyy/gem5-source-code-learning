#include "sim/init.hh"

namespace {

const uint8_t data_m5_util_sorteddict[] = {
    120,156,197,89,221,83,28,199,17,239,217,189,59,56,56,56,
    62,4,2,33,225,147,28,98,176,45,161,200,150,162,40,138,
    34,43,216,56,149,20,169,90,148,128,73,84,151,211,237,128,
    22,238,139,221,69,230,98,84,169,50,122,75,149,243,144,135,
    84,30,242,154,215,60,250,207,115,250,215,179,95,7,178,229,
    170,120,109,142,27,250,102,150,153,254,245,175,187,167,103,174,
    73,209,207,24,191,31,214,136,130,99,69,228,242,175,162,22,
    81,91,209,142,162,182,69,59,22,41,215,34,109,211,174,34,
    215,166,151,68,167,68,159,236,20,72,23,201,45,208,129,69,
    126,168,220,34,6,118,74,164,11,228,150,200,29,34,119,152,
    220,50,185,35,228,142,146,91,33,119,140,220,113,114,171,228,
    78,160,231,148,74,59,67,164,249,249,33,58,85,188,194,36,
    173,179,184,63,140,201,215,63,206,12,76,225,35,143,200,92,
    232,73,199,208,205,243,79,147,123,129,78,173,228,243,12,185,
    179,217,207,23,95,241,111,163,228,89,162,83,133,238,184,115,
    228,206,211,157,115,15,49,142,95,100,20,169,66,114,47,209,
    253,129,135,198,140,118,21,163,247,241,45,12,237,143,9,74,
    134,187,64,122,156,85,81,107,79,102,232,79,108,203,42,237,
    76,144,174,50,214,9,126,250,112,91,109,117,216,224,151,105,
    115,249,10,179,224,125,197,63,203,22,75,225,40,55,79,189,
    64,55,195,122,75,239,134,97,37,253,236,123,123,207,194,112,
    132,59,54,187,126,168,221,53,175,25,54,99,50,11,252,126,
    4,50,127,202,100,106,2,135,76,41,168,177,64,44,4,27,
    28,105,97,148,141,180,83,20,202,44,225,85,129,65,195,49,
    179,195,228,66,96,30,13,185,101,88,4,194,8,248,133,48,
    10,150,33,84,192,53,132,49,176,4,97,28,44,67,168,194,
    198,16,38,96,111,8,147,48,13,132,41,248,3,132,105,216,
    11,194,5,98,71,128,192,28,78,137,48,11,126,33,92,4,
    203,16,152,173,43,248,213,151,64,245,75,155,118,22,64,56,
    134,46,131,108,8,252,192,156,8,139,242,240,60,189,100,104,
    111,128,61,116,214,64,12,132,171,164,175,201,3,151,241,0,
    224,191,73,206,230,242,34,27,177,201,246,35,21,89,244,87,
    176,232,4,120,33,58,145,8,9,249,113,155,105,3,89,27,
    225,16,183,245,64,216,48,244,161,99,79,135,141,48,244,195,
    18,203,209,24,230,11,49,97,160,91,187,203,224,75,154,96,
    141,155,213,103,221,182,94,237,182,188,231,218,63,238,247,87,
    123,126,119,207,111,180,87,247,116,251,246,245,32,108,60,109,
    233,213,192,111,174,246,250,225,179,110,103,181,125,123,245,40,
    244,90,171,102,102,151,125,224,6,143,192,109,234,188,112,164,
    12,128,4,80,136,84,19,127,226,183,224,185,200,194,137,2,
    158,186,160,218,87,240,97,197,238,178,41,138,110,8,20,199,
    134,206,101,204,235,234,86,253,64,247,3,211,15,88,33,6,
    159,55,90,185,65,9,18,40,53,76,94,16,40,101,37,228,
    64,139,98,12,230,67,22,250,163,2,131,245,223,234,188,71,
    5,166,232,96,132,252,135,164,148,66,191,5,117,247,11,120,
    230,84,200,126,161,50,248,21,109,110,103,160,99,98,9,186,
    104,121,1,30,142,115,199,7,204,169,247,244,40,212,31,250,
    126,215,119,138,49,165,208,59,28,70,20,135,218,31,52,147,
    51,26,155,231,59,182,17,22,23,205,150,48,45,212,37,203,
    86,163,106,76,205,193,74,80,192,142,222,98,165,187,68,145,
    137,78,4,50,199,255,11,43,226,254,196,162,121,238,58,176,
    201,127,159,66,75,134,21,125,174,136,51,20,143,197,134,17,
    119,0,76,201,75,236,107,78,53,238,249,141,238,139,77,228,
    153,212,63,88,63,81,212,235,184,250,56,15,43,164,218,232,
    195,27,152,121,88,220,100,74,77,171,106,142,70,144,152,144,
    100,204,139,139,21,156,73,128,75,208,59,83,104,166,115,226,
    62,179,188,62,124,255,245,176,111,38,233,107,95,101,96,243,
    238,93,67,203,57,109,110,51,139,88,130,65,210,155,167,18,
    88,88,225,7,133,218,10,239,101,161,150,212,112,94,80,213,
    15,1,213,153,225,137,30,190,30,225,207,191,14,97,24,123,
    242,169,241,226,119,196,109,57,14,7,193,26,55,46,12,224,
    148,96,109,233,206,247,74,109,38,147,132,31,101,113,207,168,
    66,190,184,83,87,158,251,190,73,70,37,241,219,115,96,227,
    130,35,217,211,174,177,208,31,1,174,63,19,109,117,38,147,
    45,237,10,182,180,142,162,237,179,123,53,246,25,7,219,128,
    116,10,146,60,244,31,142,236,23,233,111,171,17,222,113,206,
    23,76,87,73,74,37,174,45,57,215,154,170,50,41,51,214,
    204,22,60,187,41,100,108,72,113,149,150,179,203,159,45,5,
    47,86,164,114,121,183,150,108,246,96,39,192,196,243,36,51,
    61,169,98,11,119,37,137,207,254,65,29,218,88,205,4,114,
    0,232,75,254,189,218,146,191,97,44,32,101,25,154,27,55,
    101,187,230,93,186,157,131,109,2,228,170,251,123,186,163,143,
    123,254,131,79,40,170,191,74,202,40,128,149,247,187,94,71,
    82,26,10,5,168,17,228,202,150,9,177,186,175,123,126,189,
    190,77,73,21,197,123,68,28,90,3,140,133,38,182,76,109,
    104,161,202,141,170,167,51,197,161,196,10,206,142,166,82,67,
    169,6,48,245,186,184,199,217,80,90,200,201,21,223,224,137,
    254,152,130,154,78,235,220,36,103,44,158,1,133,147,225,183,
    66,196,245,110,22,145,53,128,40,15,48,53,146,90,52,6,
    51,145,198,148,138,193,44,80,92,188,68,229,250,215,34,49,
    69,105,131,207,32,72,40,205,150,110,248,6,71,174,169,1,
    57,171,153,66,24,75,75,245,228,220,49,75,113,14,207,214,
    224,167,42,209,62,57,37,133,253,158,206,28,54,164,63,143,
    8,193,90,205,110,175,191,151,42,94,121,133,237,135,19,219,
    47,102,85,149,77,36,223,16,134,78,40,243,15,226,124,66,
    169,122,137,93,211,243,169,113,141,140,69,147,220,211,242,2,
    115,117,128,220,195,7,183,35,157,115,242,193,188,102,157,206,
    255,161,187,243,22,154,43,185,219,185,104,44,211,14,14,191,
    73,217,177,1,101,95,97,101,24,55,127,175,112,198,141,30,
    3,138,198,153,175,89,139,210,249,241,165,200,103,215,204,118,
    105,114,224,60,111,151,35,180,149,73,25,169,39,15,6,220,
    65,30,154,99,186,79,233,108,162,195,194,133,88,119,48,126,
    92,75,116,159,73,51,5,212,223,181,232,219,32,112,222,205,
    201,244,112,197,207,6,0,20,34,229,147,234,173,69,162,42,
    43,120,80,38,127,49,83,179,74,214,51,7,202,104,244,49,
    157,112,205,250,136,76,50,199,174,171,80,215,30,62,150,30,
    59,237,233,136,135,161,179,24,237,207,92,242,92,188,122,170,
    6,15,48,165,216,19,55,186,29,45,229,166,28,44,28,92,
    249,56,239,36,118,42,36,76,67,107,198,238,135,114,8,208,
    29,215,20,40,157,102,235,40,96,67,229,85,150,112,78,243,
    27,157,61,253,87,204,60,46,166,172,168,41,171,194,167,158,
    41,107,202,106,198,229,36,148,234,194,168,111,195,43,222,140,
    143,239,98,128,47,34,215,182,197,55,108,113,237,233,243,142,
    241,147,65,192,144,26,254,94,32,9,234,224,83,136,121,57,
    139,228,91,201,129,130,245,111,169,219,204,171,4,97,33,70,
    136,179,235,241,202,121,132,51,130,48,1,105,2,224,181,56,
    157,91,104,222,203,49,20,226,74,182,45,224,254,126,14,92,
    92,95,254,46,78,72,103,234,203,47,212,107,171,49,0,57,
    234,113,41,163,207,150,150,9,186,60,98,252,14,79,244,143,
    20,207,116,90,47,39,49,142,219,166,126,37,78,74,155,91,
    157,5,62,163,145,156,209,86,228,218,209,138,198,238,159,57,
    173,9,85,175,56,114,154,98,125,183,225,181,186,79,247,243,
    242,69,174,212,93,189,219,56,106,133,255,164,204,33,174,130,
    106,205,202,64,92,7,196,117,64,92,252,134,2,122,171,115,
    141,65,219,2,250,81,4,58,184,71,159,83,156,226,96,153,
    237,204,21,11,37,30,42,236,98,181,94,183,103,14,149,175,
    58,133,223,202,137,223,159,241,68,255,78,19,143,205,89,124,
    156,77,80,226,215,249,251,229,223,195,12,11,81,88,2,146,
    217,74,163,34,124,171,115,133,109,96,137,13,238,193,6,140,
    202,181,226,171,195,139,180,205,86,227,148,131,164,29,91,144,
    3,120,83,140,98,39,70,145,147,10,155,66,142,166,43,247,
    106,80,212,235,118,26,126,191,230,5,53,221,238,133,125,201,
    236,230,214,2,246,18,58,127,141,123,85,115,25,61,25,91,
    85,192,229,127,98,145,239,57,34,141,255,67,241,21,52,155,
    114,76,76,57,97,53,99,95,74,142,152,80,224,68,190,66,
    228,124,118,60,7,107,172,61,153,160,23,5,248,10,242,91,
    129,238,31,78,208,214,137,157,4,139,232,93,140,125,165,217,
    10,228,111,160,15,101,219,146,204,42,165,138,155,27,76,216,
    122,215,239,182,81,131,255,55,137,25,171,44,56,55,150,127,
    148,68,110,189,211,104,235,122,93,120,169,215,219,93,247,168,
    197,31,29,152,201,25,137,159,98,21,122,218,15,251,242,221,
    129,33,243,2,26,28,140,204,22,45,155,53,110,57,204,222,
    45,161,1,231,144,227,174,28,19,229,160,229,224,218,223,249,
    49,154,21,52,216,29,165,42,148,2,75,138,20,231,58,69,
    219,251,71,141,86,160,101,123,112,112,91,236,220,70,131,28,
    231,220,141,29,198,145,235,181,81,49,114,35,8,218,154,77,
    226,58,15,7,140,250,29,7,33,92,20,122,6,15,8,183,
    36,101,171,108,87,173,106,185,92,26,120,217,229,66,244,178,
    163,23,203,83,21,211,87,41,149,249,199,86,17,3,237,134,
    215,169,215,207,87,237,79,140,227,173,127,108,98,88,190,174,
    13,21,37,137,140,131,213,140,217,3,99,133,129,177,226,192,
    88,41,25,203,164,246,161,132,147,183,18,58,86,18,78,222,
    142,137,49,91,192,47,115,140,75,215,11,122,173,70,255,203,
    52,197,21,213,168,154,140,222,18,49,199,30,238,62,69,212,
    94,49,22,247,189,66,44,62,245,172,88,252,139,119,41,214,
    88,252,36,40,154,216,175,237,203,112,67,42,79,17,247,188,
    161,88,172,123,195,177,248,76,218,190,115,83,172,132,76,37,
    27,187,249,82,219,220,198,138,73,36,125,73,230,250,0,205,
    250,192,162,198,77,239,38,174,187,20,123,178,20,116,143,253,
    35,45,117,142,44,245,60,47,167,21,71,187,111,130,251,1,
    46,100,164,104,155,181,102,255,85,81,229,242,117,53,98,85,
    249,93,228,228,48,98,77,240,174,130,191,11,252,30,82,35,
    120,89,37,107,212,90,82,255,3,24,170,149,164,
};

EmbeddedPython embedded_m5_util_sorteddict(
    "m5/util/sorteddict.py",
    "/home/oliverxyy/program/gem5-stable/src/python/m5/util/sorteddict.py",
    "m5.util.sorteddict",
    data_m5_util_sorteddict,
    2445,
    8576);

} // anonymous namespace
