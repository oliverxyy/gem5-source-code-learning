#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_DRAMCtrl[] = {
    120,156,205,90,73,115,35,71,118,78,144,32,72,112,95,209,
    100,111,42,109,221,108,73,13,2,5,2,36,165,150,212,36,
    192,150,122,68,182,56,5,170,217,131,177,93,81,68,21,137,
    82,99,161,81,197,238,166,195,7,135,229,203,68,120,157,136,
    137,177,195,14,251,96,135,127,133,127,135,207,62,250,224,139,
    79,62,218,239,125,149,89,85,0,23,75,49,148,61,13,49,
    145,249,229,82,185,124,223,123,175,18,170,11,249,111,144,254,
    30,107,66,120,90,66,8,155,254,75,136,166,16,251,50,151,
    160,220,128,112,6,196,81,66,216,131,226,79,132,248,78,136,
    159,213,6,133,157,84,232,80,136,38,133,157,82,232,112,136,
    14,9,123,68,56,9,160,233,16,77,9,123,84,56,41,160,
    99,33,58,44,236,113,225,12,3,157,8,209,17,97,79,42,
    116,42,68,211,194,158,86,35,204,132,232,168,176,103,21,58,
    23,162,99,194,158,87,232,66,136,142,11,59,163,208,27,33,
    58,33,236,69,133,46,133,232,164,176,111,138,234,242,45,218,
    45,247,191,233,223,50,237,150,240,57,249,192,31,161,116,215,
    105,85,235,13,199,174,171,125,29,160,191,45,222,215,121,202,
    56,66,212,18,114,119,143,7,68,109,64,24,213,101,110,225,
    39,41,57,170,31,121,126,138,51,93,206,46,243,145,96,80,
    211,108,91,45,199,52,253,81,20,90,29,251,180,201,69,238,
    244,202,106,122,203,252,160,40,241,214,40,89,105,116,90,206,
    74,167,233,190,114,186,111,206,206,86,78,186,157,227,174,213,
    90,57,118,90,197,135,158,111,29,54,157,21,175,91,95,105,
    57,173,149,138,177,185,91,246,187,205,236,201,153,193,75,209,
    121,16,158,86,74,248,195,244,181,105,219,221,93,235,36,92,
    211,160,90,211,141,190,53,17,71,142,7,229,178,48,123,158,
    176,209,49,172,45,171,220,40,119,226,197,78,185,33,139,101,
    89,143,30,6,175,201,24,226,36,213,179,168,223,124,101,188,
    146,143,99,43,227,167,239,89,199,206,174,213,166,52,92,92,
    82,45,238,230,249,197,17,245,143,147,114,125,73,117,108,157,
    19,167,237,79,200,140,105,217,214,137,79,83,243,121,13,245,
    102,199,115,252,73,149,11,43,127,236,165,142,81,223,207,120,
    144,36,150,154,78,128,70,170,197,249,115,252,171,100,108,169,
    180,60,90,45,105,219,73,242,154,191,163,252,16,11,225,219,
    97,94,63,9,254,59,98,238,136,66,134,88,236,140,164,129,
    140,178,228,73,232,140,140,41,36,205,34,103,100,92,245,26,
    99,129,51,50,1,100,146,101,78,226,102,100,10,200,52,75,
    156,132,205,200,12,144,89,150,55,137,154,145,185,216,56,115,
    64,32,235,111,23,88,223,60,97,232,249,219,27,172,114,210,
    54,55,88,140,33,55,128,44,169,65,22,209,229,166,42,46,
    161,120,75,13,120,19,197,219,170,246,22,138,119,84,241,54,
    138,119,85,241,142,176,239,98,240,183,20,242,22,26,104,170,
    168,9,251,109,52,120,27,200,59,194,121,87,216,239,0,121,
    79,205,240,93,116,121,95,21,223,67,241,158,42,190,143,226,
    125,85,188,135,226,178,42,222,71,241,129,42,46,163,248,129,
    42,62,64,241,67,85,252,0,197,143,84,241,67,97,127,132,
    153,60,84,200,67,52,200,170,98,22,197,21,85,92,65,49,
    167,138,57,20,243,170,152,71,81,87,69,29,197,66,236,89,
    5,60,107,85,33,171,104,80,84,27,85,68,177,20,107,95,
    66,251,181,24,178,6,100,61,134,172,3,217,136,33,27,64,
    62,6,242,137,176,63,22,246,39,64,30,197,144,71,64,62,
    141,33,159,2,249,44,134,124,6,228,243,24,242,57,144,199,
    49,228,49,144,205,24,178,9,100,43,134,108,1,41,199,144,
    50,144,74,12,169,0,217,142,33,219,64,158,196,144,39,64,
    190,136,33,95,0,249,50,134,124,9,228,105,12,121,10,228,
    39,49,228,39,64,190,138,33,95,1,217,137,33,59,64,118,
    99,200,46,144,103,49,228,25,144,175,99,200,215,64,246,98,
    200,30,144,159,2,49,132,253,83,97,27,64,170,49,164,10,
    100,159,45,236,62,155,198,105,54,99,156,176,141,179,201,248,
    153,117,54,114,141,134,199,246,187,218,180,94,57,218,73,167,
    235,187,143,185,225,29,74,158,157,182,14,157,174,214,57,210,
    94,119,93,223,209,126,255,212,57,117,52,178,209,93,215,241,
    92,141,155,221,238,105,214,117,44,187,175,213,55,220,106,137,
    146,253,70,215,241,26,157,166,173,249,29,237,168,211,173,59,
    193,168,158,171,95,216,132,140,114,215,87,77,48,247,44,71,
    5,110,219,109,157,182,228,132,14,79,187,158,239,105,135,14,
    141,231,104,222,107,215,175,55,220,246,49,119,231,169,120,6,
    219,101,111,49,136,38,58,221,51,205,227,136,226,180,201,109,
    78,200,25,212,207,140,52,183,200,72,215,236,120,158,214,178,
    78,78,98,245,19,170,158,29,28,85,178,135,107,209,226,100,
    189,247,46,15,110,189,209,172,122,157,122,59,158,118,66,91,
    209,237,188,86,147,98,119,69,195,121,51,188,199,238,31,56,
    188,77,236,62,52,154,233,9,188,94,62,215,246,224,255,171,
    190,229,187,117,237,168,219,105,251,78,219,214,154,22,125,209,
    35,50,81,221,161,85,127,25,175,202,81,149,109,249,22,237,
    132,167,189,118,109,191,161,185,109,237,208,165,77,161,167,107,
    142,85,111,4,79,179,157,87,110,221,89,225,135,194,29,111,
    241,206,105,77,167,125,220,240,181,229,173,157,7,232,231,88,
    190,231,221,83,139,93,198,50,78,143,142,156,238,3,205,227,
    185,243,218,226,35,105,61,167,31,212,120,178,10,251,96,181,
    95,122,183,122,41,194,16,42,235,13,171,221,118,154,46,124,
    244,221,158,70,135,212,72,59,238,118,78,227,227,44,158,107,
    18,85,186,28,108,121,179,61,45,228,248,158,55,39,61,182,
    214,176,60,173,178,179,163,209,206,180,59,190,55,78,120,185,
    217,169,191,228,97,220,142,13,113,24,155,85,166,79,153,190,
    108,167,105,157,121,28,1,112,73,109,249,44,66,173,215,218,
    73,215,161,39,116,105,159,124,183,229,160,239,102,121,159,251,
    238,25,219,178,47,63,249,0,76,165,198,29,10,61,206,130,
    198,76,6,131,181,66,173,195,113,112,152,193,185,216,167,93,
    58,238,78,91,91,230,83,172,84,140,128,233,56,47,58,226,
    21,77,215,234,103,245,166,227,61,240,153,225,76,32,62,138,
    42,197,182,177,189,195,34,122,214,130,185,59,71,172,178,96,
    128,96,58,139,113,184,211,34,146,219,68,7,223,233,114,72,
    124,63,92,131,84,213,71,154,199,207,225,109,143,75,142,7,
    186,31,91,23,20,122,69,91,166,133,193,56,15,123,65,125,
    108,67,249,43,88,192,69,171,60,215,136,119,253,5,9,146,
    226,195,96,23,95,187,109,187,243,218,123,91,74,181,29,82,
    68,182,33,213,186,97,43,142,49,247,58,175,157,238,67,26,
    187,130,1,239,158,131,168,49,157,3,115,199,177,153,82,48,
    95,85,167,121,244,176,43,247,209,121,227,250,33,103,238,92,
    85,203,253,131,99,108,109,226,40,54,121,86,78,140,96,245,
    211,110,151,76,14,134,185,172,82,123,94,169,232,232,190,23,
    86,97,206,180,166,182,230,53,105,101,119,174,168,188,186,251,
    145,229,249,151,118,231,202,160,251,82,79,11,50,89,109,251,
    240,44,156,253,221,171,106,131,1,22,162,5,246,205,253,230,
    101,53,87,116,196,172,47,236,24,77,57,19,85,247,207,247,
    214,165,85,65,87,54,31,198,246,102,37,236,48,211,7,4,
    205,216,129,28,24,79,247,183,195,118,179,253,72,208,112,42,
    174,66,217,116,254,60,22,52,158,239,103,84,89,246,56,71,
    196,114,172,155,207,47,108,185,231,152,193,174,69,156,127,222,
    105,250,108,234,73,136,199,129,89,210,73,251,61,232,242,55,
    61,111,88,112,89,254,217,137,131,183,190,250,155,55,102,131,
    4,239,116,253,180,10,34,246,40,134,64,43,14,38,240,242,
    182,103,81,180,129,23,167,111,218,158,123,220,118,108,159,31,
    5,11,97,6,14,198,100,255,226,179,230,217,196,244,128,252,
    182,185,231,80,188,208,246,253,76,216,173,225,30,55,76,31,
    1,131,73,230,187,238,47,132,85,196,138,115,53,45,183,109,
    6,177,4,99,102,96,107,240,138,238,203,144,200,68,108,96,
    74,183,207,15,245,249,132,45,10,11,76,25,19,224,109,208,
    231,228,132,182,71,54,245,249,40,90,214,27,83,69,0,120,
    0,185,78,108,80,16,119,176,227,71,191,192,69,70,235,218,
    9,76,128,207,238,223,131,139,55,149,251,55,165,121,192,154,
    101,157,116,255,97,213,116,52,36,5,0,38,2,0,204,26,
    238,194,12,220,5,214,47,91,209,180,250,247,91,122,237,96,
    214,100,85,113,52,112,184,128,164,23,197,42,217,232,154,129,
    91,142,154,79,202,138,24,196,39,173,188,47,152,176,213,33,
    70,113,102,191,123,234,192,214,217,205,38,190,253,242,87,1,
    163,140,114,69,2,129,49,244,141,61,89,177,89,13,128,3,
    67,2,251,123,184,219,241,183,190,49,170,251,65,182,92,174,
    152,59,178,250,73,25,164,243,141,237,39,79,3,232,96,63,
    236,122,32,31,82,149,128,81,9,6,160,140,26,224,197,230,
    1,54,38,114,32,102,211,109,185,126,208,243,133,124,248,139,
    61,101,182,253,23,85,9,85,25,226,99,149,154,195,120,79,
    43,149,28,38,196,25,29,45,41,167,239,229,208,50,200,198,
    224,124,4,231,117,213,79,127,22,54,120,22,182,45,68,67,
    20,98,67,20,162,33,10,177,33,10,225,16,133,103,33,184,
    106,40,112,213,136,192,131,16,60,208,213,18,138,170,182,24,
    66,37,5,149,116,60,80,90,13,236,9,25,27,52,99,163,
    115,157,215,49,124,46,219,60,200,159,82,146,74,208,39,57,
    158,154,77,204,38,233,51,72,127,9,250,208,247,20,99,131,
    83,3,83,9,252,13,205,38,166,184,126,108,42,57,53,24,
    126,198,168,213,160,68,82,156,82,191,160,29,127,102,228,247,
    247,252,224,218,138,2,181,130,153,47,229,114,230,155,210,106,
    120,55,196,70,6,119,67,255,34,46,186,27,226,20,215,67,
    124,175,75,249,20,223,13,241,189,109,130,239,134,236,36,223,
    7,217,67,130,239,97,83,124,19,196,233,56,210,9,190,33,
    226,123,212,17,190,235,177,211,162,54,205,215,67,181,25,190,
    205,169,205,242,197,80,109,142,239,131,106,243,72,23,248,98,
    168,150,65,122,131,111,133,106,139,124,49,84,91,226,251,160,
    218,77,190,18,170,221,226,107,160,218,109,190,253,169,221,225,
    43,158,218,93,190,247,169,189,197,175,145,108,165,112,232,197,
    188,190,187,229,178,204,113,218,249,151,91,46,123,22,143,121,
    147,207,234,197,182,7,156,190,61,102,70,190,144,93,99,140,
    25,81,80,149,37,5,80,244,234,38,213,200,122,41,39,241,
    60,247,102,104,45,171,114,122,54,194,214,79,131,102,107,197,
    214,38,50,197,156,202,172,81,134,27,229,75,69,149,91,87,
    152,174,115,148,133,241,179,197,231,203,116,44,241,219,67,126,
    245,49,62,224,228,67,78,62,226,228,33,39,252,234,105,112,
    124,110,20,57,121,196,73,137,19,38,179,177,206,201,6,39,
    101,78,152,160,198,19,78,248,230,208,224,155,82,227,49,39,
    159,112,178,201,201,22,39,159,115,178,195,137,193,201,11,78,
    126,135,147,26,39,191,199,73,93,92,239,117,38,209,79,60,
    34,66,122,188,130,212,64,106,240,220,39,209,243,25,184,172,
    14,94,230,203,221,178,169,23,153,238,5,253,60,221,255,254,
    98,186,15,128,238,131,160,123,18,116,31,2,221,83,160,251,
    48,232,62,2,186,167,65,247,81,208,125,12,116,31,143,209,
    125,2,116,159,4,221,167,64,247,105,208,125,24,116,159,1,
    221,103,65,247,1,208,29,55,151,76,238,37,228,111,50,161,
    231,21,131,215,137,206,204,133,128,211,76,8,189,88,218,194,
    91,37,184,25,208,155,57,146,203,174,75,14,22,178,58,229,
    64,249,28,178,12,110,100,55,66,138,174,201,106,61,159,37,
    178,227,61,55,32,232,134,148,128,26,105,53,236,84,200,110,
    156,122,6,187,84,131,163,7,252,58,114,237,252,236,163,230,
    39,189,132,228,11,13,3,207,229,187,29,227,29,78,180,107,
    166,161,79,125,127,201,52,228,171,159,30,138,13,245,80,109,
    40,204,39,227,4,140,140,173,158,47,20,46,54,182,135,87,
    177,47,48,182,3,49,99,155,4,251,134,192,190,20,216,55,
    12,246,141,128,125,105,176,111,20,236,27,3,251,198,153,125,
    196,32,92,15,13,131,23,27,133,117,105,239,216,220,233,145,
    237,203,109,40,83,87,104,123,70,35,98,152,132,215,148,229,
    42,172,201,204,234,170,50,92,145,49,219,200,43,19,86,236,
    49,97,51,189,20,249,190,246,233,255,214,238,156,81,223,127,
    77,200,123,30,105,85,34,107,210,107,89,228,225,174,154,250,
    234,101,158,244,223,46,62,220,203,60,233,247,57,220,52,14,
    55,141,195,29,133,105,25,131,105,25,135,105,153,128,105,73,
    194,180,76,194,180,76,193,180,140,192,180,76,195,180,204,124,
    95,79,106,223,224,159,49,236,69,254,233,194,94,18,181,119,
    248,167,145,218,187,252,139,72,237,61,254,33,164,246,62,83,
    139,223,183,141,35,101,148,140,99,101,134,2,131,52,29,17,
    111,157,121,37,137,87,8,57,22,176,111,53,155,15,29,173,
    30,26,153,66,220,240,4,100,204,83,6,149,69,114,192,70,
    75,72,115,23,56,95,12,149,45,149,74,231,220,111,137,153,
    202,182,140,190,141,142,162,118,105,77,162,133,208,1,231,66,
    167,28,230,54,244,144,199,250,115,100,116,38,244,189,94,66,
    95,109,243,86,196,121,195,247,233,149,236,175,132,18,248,65,
    46,122,183,87,47,63,187,68,52,134,125,205,202,249,99,234,
    251,159,172,156,47,69,159,199,78,194,60,242,191,193,80,71,
    131,212,226,74,191,45,53,198,23,0,59,123,164,50,221,172,
    174,154,249,92,169,116,177,15,255,143,80,104,206,0,36,38,
    32,177,196,37,246,51,8,86,7,32,177,161,115,193,106,10,
    18,27,134,196,82,144,216,8,36,54,2,137,165,33,177,81,
    72,108,4,18,27,131,196,198,32,177,113,72,108,2,18,155,
    132,196,166,32,177,105,72,108,6,18,155,133,196,230,32,177,
    89,72,108,30,18,91,128,196,50,144,216,13,72,108,145,37,
    198,210,187,7,233,221,135,244,150,89,116,183,67,209,105,189,
    202,75,132,146,202,174,175,149,164,68,2,99,173,199,3,213,
    128,218,28,212,70,14,29,81,64,16,3,171,120,65,69,185,
    197,48,250,13,98,94,82,209,31,10,25,148,232,202,45,176,
    189,151,241,175,132,242,145,136,40,103,252,130,167,233,133,19,
    10,43,99,158,98,253,121,40,181,229,7,61,10,67,211,39,
    86,211,115,140,213,255,93,112,145,214,46,82,88,20,80,60,
    10,197,245,121,40,174,72,87,125,113,114,159,194,170,23,200,
    236,119,67,173,253,60,20,156,249,163,169,238,207,88,31,100,
    108,189,157,184,234,146,125,177,242,0,148,22,232,47,174,182,
    11,20,23,42,143,111,249,14,92,219,121,250,181,169,179,123,
    203,235,235,231,101,247,215,63,150,236,70,32,187,52,100,151,
    134,236,70,33,187,177,152,236,198,33,187,113,200,110,2,178,
    155,132,236,166,100,208,204,158,199,15,104,172,175,82,220,252,
    71,66,250,164,192,15,188,12,194,230,40,206,201,83,88,100,
    252,57,67,161,100,244,156,20,138,158,239,17,138,193,193,76,
    76,34,198,95,10,233,244,250,35,97,222,161,255,47,186,94,
    39,209,254,134,250,254,35,19,45,119,25,209,162,87,177,126,
    250,5,100,130,25,87,215,14,63,162,13,31,6,153,70,98,
    100,10,46,28,82,32,211,40,200,52,6,50,141,131,76,105,
    144,105,2,100,154,0,153,38,65,166,41,144,105,26,54,124,
    6,54,124,22,54,124,14,54,124,30,54,124,1,54,60,3,
    27,190,0,27,126,3,54,124,1,54,124,17,54,124,9,54,
    252,38,108,248,45,216,240,219,210,134,71,129,147,38,148,13,
    255,91,161,108,120,116,71,97,252,157,34,37,232,25,217,112,
    112,215,248,11,133,5,54,60,186,128,232,231,103,200,253,117,
    105,157,75,202,96,83,108,38,33,189,68,102,250,151,138,255,
    5,142,150,126,37,212,157,71,65,189,0,232,212,220,248,181,
    144,255,187,209,85,70,251,55,18,64,31,247,215,127,168,0,
    126,251,236,245,63,80,223,127,63,103,175,35,41,13,72,107,
    61,136,136,233,135,216,107,126,23,255,130,20,86,52,233,109,
    228,146,215,145,127,18,215,126,211,49,214,255,174,201,26,154,
    65,58,11,37,205,65,73,243,80,82,24,43,205,64,103,105,
    232,44,13,157,165,163,215,17,82,198,156,162,28,121,157,248,
    141,135,241,74,168,151,139,232,202,99,90,209,58,47,175,43,
    116,249,93,80,55,116,186,122,121,96,11,255,173,34,183,94,
    136,95,118,148,148,158,34,41,5,65,214,111,81,132,127,157,
    76,252,103,234,251,152,118,201,91,237,103,98,240,25,33,254,
    41,54,14,209,135,205,250,64,140,143,137,103,203,108,213,241,
    115,90,171,152,61,225,159,206,60,176,112,243,208,243,187,86,
    221,15,126,82,194,230,110,183,79,91,248,17,11,191,87,225,
    231,41,92,145,227,158,15,183,44,120,243,198,75,4,98,26,
    248,27,168,5,19,189,198,133,227,71,159,71,193,255,80,251,
    25,159,30,126,121,29,77,140,38,51,35,153,100,102,56,243,
    95,153,231,153,74,230,86,230,231,153,131,204,118,230,197,255,
    0,57,15,216,33,
};

EmbeddedPython embedded_m5_objects_DRAMCtrl(
    "m5/objects/DRAMCtrl.py",
    "/home/oliverxyy/program/gem5-stable/src/mem/DRAMCtrl.py",
    "m5.objects.DRAMCtrl",
    data_m5_objects_DRAMCtrl,
    3989,
    11567);

} // anonymous namespace