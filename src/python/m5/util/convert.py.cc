#include "sim/init.hh"

namespace {

const uint8_t data_m5_util_convert[] = {
    120,156,237,90,205,111,28,89,17,175,158,241,140,61,31,254,
    158,56,223,73,199,89,199,19,192,25,88,109,194,42,10,38,
    113,118,227,4,173,189,86,219,178,151,129,16,218,243,58,227,
    206,244,116,79,186,123,226,76,148,72,11,201,33,151,21,28,
    64,40,23,64,220,34,144,22,14,252,1,28,88,33,164,61,
    144,3,112,225,192,105,17,43,164,69,8,193,97,217,80,85,
    175,123,122,102,226,137,157,79,150,213,166,51,245,170,95,126,
    239,245,235,170,95,87,85,191,78,9,130,63,113,252,157,84,
    1,188,21,5,64,0,20,81,42,80,84,64,196,160,24,3,
    17,135,98,28,68,15,20,123,64,36,160,152,0,145,132,98,
    18,68,47,20,123,65,244,65,177,15,68,10,138,41,16,105,
    40,166,65,100,160,152,1,145,133,98,22,140,44,41,185,98,
    63,24,253,82,27,0,99,64,106,131,96,12,74,109,8,140,
    33,169,13,131,232,135,91,184,132,17,16,3,172,140,130,153,
    6,35,7,98,16,142,81,51,36,155,97,217,140,200,102,20,
    155,109,32,114,178,217,38,155,49,217,108,151,205,14,56,86,
    28,3,177,147,39,221,14,98,23,43,59,64,236,102,101,39,
    136,61,172,236,2,177,151,149,221,32,246,177,178,7,196,126,
    86,246,130,80,89,217,7,226,0,43,251,65,140,179,162,130,
    56,200,202,1,16,47,176,50,14,98,130,149,131,32,14,193,
    98,126,18,141,92,134,223,204,151,171,191,250,233,233,50,192,
    75,135,254,89,201,146,2,63,249,222,174,234,12,41,96,188,
    251,238,41,86,222,188,117,68,42,112,242,59,39,203,31,222,
    253,251,189,185,213,133,47,151,223,250,219,47,127,252,175,119,
    126,62,93,254,254,239,15,253,37,115,100,120,186,60,252,215,
    169,111,221,249,193,229,47,149,199,150,223,251,225,219,241,123,
    39,202,63,59,125,239,119,231,221,23,79,152,208,3,80,66,
    135,2,253,80,135,211,228,98,116,41,248,0,215,81,42,112,
    51,6,222,4,248,49,114,183,31,167,206,155,10,140,125,59,
    6,54,35,46,245,16,3,176,203,61,3,126,130,122,144,10,
    42,158,251,73,200,45,6,128,30,6,148,59,0,189,17,32,
    193,128,219,29,128,190,8,144,100,192,221,14,64,42,2,244,
    50,224,183,29,128,116,4,232,99,192,251,29,128,76,4,72,
    49,32,171,4,128,180,4,100,35,64,134,1,83,29,128,254,
    8,144,101,192,124,7,96,32,2,244,51,192,233,0,12,70,
    128,1,6,124,183,3,48,20,1,6,25,240,139,14,192,112,
    4,24,98,192,189,14,192,72,4,24,102,192,63,58,0,163,
    17,96,132,1,185,88,59,32,23,1,70,25,112,180,3,176,
    45,2,228,24,176,220,1,24,67,128,236,193,179,69,12,33,
    139,249,109,200,184,121,111,47,202,117,215,177,203,170,223,168,
    25,234,228,132,55,169,122,107,78,221,18,234,170,161,122,190,
    139,228,3,120,213,52,63,186,127,255,62,235,11,38,55,75,
    178,153,149,205,156,108,42,166,175,48,252,62,161,73,93,96,
    185,196,114,150,229,28,203,10,203,42,203,58,75,155,101,141,
    229,197,252,118,148,200,32,0,211,51,109,207,215,237,146,225,
    83,24,164,21,165,104,70,92,237,171,174,235,184,62,61,57,
    180,118,100,44,62,160,182,240,214,77,127,13,239,21,103,177,
    28,221,231,127,55,174,174,154,172,212,140,64,241,67,165,108,
    6,74,53,236,169,80,79,156,71,233,193,32,95,15,6,185,
    122,48,168,172,7,131,2,165,98,90,14,95,180,106,90,150,
    25,104,37,215,225,127,180,117,91,42,53,179,36,81,23,141,
    170,239,228,233,86,249,244,138,110,213,141,60,133,19,22,222,
    41,20,133,53,167,106,20,28,203,188,98,184,87,27,141,66,
    205,117,202,174,94,45,148,141,234,209,41,180,200,170,101,20,
    60,183,84,168,53,252,53,199,46,84,143,22,234,190,105,21,
    74,142,141,3,252,35,216,221,75,211,59,103,200,10,95,164,
    89,191,66,115,43,131,202,88,108,80,25,81,158,240,23,227,
    208,21,11,178,19,135,174,151,33,12,93,72,178,27,28,192,
    2,157,149,235,10,84,226,224,190,20,196,51,236,9,35,25,
    82,50,31,99,58,238,67,89,210,109,52,151,26,220,136,164,
    164,239,168,166,237,27,101,195,205,211,229,180,195,116,41,178,
    168,133,212,101,158,44,147,9,153,17,60,149,70,134,196,48,
    8,224,26,94,221,242,159,186,113,83,108,220,115,114,81,223,
    164,121,211,108,222,44,31,67,49,73,24,183,110,48,161,125,
    38,84,195,240,248,172,193,242,11,146,9,186,229,25,218,16,
    97,104,221,182,163,13,0,155,14,224,243,77,19,55,179,67,
    9,30,41,59,220,4,246,67,2,46,37,57,26,0,141,33,
    111,40,164,87,98,224,94,160,72,143,73,164,195,27,241,173,
    5,7,111,119,119,127,173,58,142,149,39,43,105,195,36,70,
    72,140,146,200,133,172,183,156,117,28,64,102,187,64,224,11,
    194,44,73,59,149,13,249,216,206,59,182,161,125,150,28,215,
    116,169,246,185,208,143,79,211,153,73,118,230,12,46,162,74,
    147,14,52,31,148,44,146,29,189,169,60,152,167,255,240,104,
    158,248,24,229,233,119,66,64,31,3,22,209,3,152,127,3,
    2,112,106,72,111,49,53,144,175,150,206,94,51,255,67,209,
    158,78,102,207,94,227,118,46,104,43,216,146,235,16,67,249,
    195,83,187,211,229,162,107,92,174,27,118,169,145,207,116,225,
    140,70,25,75,27,35,177,159,4,77,166,29,32,49,78,130,
    121,162,132,60,121,234,20,201,200,96,26,174,114,157,102,222,
    245,144,128,58,16,251,148,51,221,202,137,154,23,149,19,152,
    220,169,169,203,166,42,3,164,199,5,196,195,178,129,165,251,
    91,37,203,33,18,19,36,94,32,113,240,57,144,69,38,135,
    215,228,26,111,111,145,42,116,255,137,144,42,63,218,42,85,
    26,83,148,8,163,60,203,47,133,148,103,231,200,155,120,186,
    27,123,131,176,190,98,15,67,15,58,178,146,198,172,8,138,
    162,216,10,188,209,200,17,165,90,39,104,195,189,21,226,240,
    92,244,52,93,158,64,151,83,212,244,118,64,152,98,85,211,
    83,117,181,100,57,165,138,90,51,92,211,17,155,211,193,36,
    179,152,236,242,131,221,157,221,58,231,124,190,183,155,203,79,
    134,158,213,206,180,229,12,14,69,88,98,61,117,55,103,113,
    22,221,110,44,133,158,190,67,83,19,105,33,78,158,142,115,
    41,208,175,244,225,175,135,207,164,246,36,222,206,117,122,187,
    205,89,175,53,157,58,213,233,212,128,21,245,135,179,226,19,
    224,109,109,246,89,60,209,3,161,171,155,25,224,237,7,157,
    29,57,58,114,251,131,89,224,207,159,156,44,208,124,163,124,
    252,124,64,102,89,90,197,140,240,33,101,4,58,155,197,51,
    86,230,66,165,66,10,61,196,132,163,98,195,203,119,167,143,
    109,248,235,142,91,81,87,117,91,172,155,194,95,251,152,22,
    20,35,156,35,230,229,106,103,194,197,254,154,46,176,231,161,
    201,34,189,1,167,254,253,127,203,169,77,119,141,222,11,1,
    169,128,116,25,218,213,104,165,90,102,107,84,243,200,90,11,
    51,5,73,53,62,91,194,51,86,102,67,101,46,84,42,164,
    16,231,104,0,115,110,178,59,231,170,70,213,113,27,45,148,
    203,110,78,57,10,165,218,78,18,84,27,104,244,38,165,237,
    121,14,188,27,102,222,205,241,138,155,180,251,99,51,152,61,
    100,127,96,163,146,246,83,226,109,177,230,93,152,137,106,222,
    165,25,185,119,38,155,57,217,84,102,184,244,157,145,165,239,
    248,166,100,243,204,107,198,38,60,251,204,255,146,103,217,22,
    158,45,226,90,223,127,4,138,197,159,56,95,98,121,67,222,
    83,72,71,15,83,233,179,76,60,139,138,26,28,112,245,36,
    33,94,57,255,50,220,224,34,7,105,118,157,71,247,196,43,
    10,56,111,18,151,80,177,99,16,83,188,243,237,195,47,159,
    167,191,43,52,68,161,177,59,233,66,189,48,29,116,244,201,
    142,20,76,207,4,61,105,217,147,137,122,178,178,39,129,61,
    139,249,254,45,114,137,204,115,196,36,227,120,228,99,211,198,
    194,214,20,170,89,83,117,33,176,52,243,212,9,79,214,89,
    247,73,144,247,185,226,50,105,139,201,36,162,153,180,81,106,
    146,137,243,125,93,232,195,219,51,94,205,50,229,126,140,101,
    216,204,22,62,49,109,95,238,193,229,67,224,106,195,55,100,
    162,38,237,25,189,121,159,171,157,146,247,247,17,205,60,210,
    198,163,161,88,191,50,174,228,226,204,29,42,24,155,133,245,
    172,242,24,220,249,58,239,145,33,37,154,12,162,18,58,198,
    128,56,159,198,185,186,102,29,25,65,251,104,119,55,224,207,
    173,54,254,220,225,157,54,70,68,59,109,205,17,23,99,24,
    96,154,115,246,241,156,199,248,59,129,164,51,242,142,99,31,
    245,255,41,160,28,141,193,230,70,130,136,140,1,78,70,190,
    87,206,191,8,55,112,38,46,224,169,40,75,194,142,233,215,
    17,132,61,215,229,98,63,160,9,174,51,155,183,211,36,151,
    63,64,34,183,45,174,191,125,177,143,84,209,145,19,10,218,
    27,77,226,49,25,213,208,107,33,97,177,76,171,234,94,133,
    216,74,204,49,169,2,227,75,108,24,206,190,74,226,27,36,
    138,36,190,22,198,47,230,159,171,219,101,67,190,40,48,39,
    153,228,53,222,254,14,174,194,193,40,208,23,116,215,247,120,
    215,49,232,152,175,87,229,86,57,146,120,89,183,120,253,230,
    51,99,241,188,188,104,63,94,133,171,88,201,226,109,248,198,
    64,76,30,87,134,240,24,225,55,136,180,146,84,70,89,39,
    134,63,24,23,111,63,78,92,220,144,219,66,242,185,133,186,
    37,230,117,72,221,171,196,83,145,232,160,110,178,133,186,252,
    118,184,85,114,28,15,226,19,6,40,111,176,133,18,53,7,
    179,220,132,215,53,42,181,176,160,73,128,40,20,105,58,4,
    223,4,104,158,103,148,206,206,213,86,76,127,109,1,47,48,
    161,180,109,20,147,3,199,55,220,40,246,31,199,77,155,87,
    72,165,16,144,8,170,147,94,122,254,91,171,147,222,45,186,
    132,247,221,150,101,117,66,43,95,222,116,243,237,138,99,249,
    58,62,114,93,29,21,85,185,207,111,183,109,89,46,170,208,
    238,151,238,149,235,226,99,249,101,166,221,47,139,109,53,1,
    155,61,241,8,79,194,169,77,77,93,170,187,174,129,25,183,
    235,54,8,155,250,240,115,179,242,105,185,158,227,74,243,131,
    151,180,242,64,108,62,207,219,171,116,227,26,221,208,134,239,
    212,76,7,198,241,134,44,111,205,78,134,207,173,238,251,142,
    44,77,185,72,229,114,149,11,87,46,97,183,55,111,116,42,
    28,176,68,159,215,200,222,103,248,35,26,125,228,212,142,67,
    184,55,68,219,66,218,89,18,231,72,188,78,98,129,196,74,
    51,140,208,127,225,208,46,145,160,239,62,154,211,102,183,167,
    105,60,122,82,78,84,29,81,183,140,105,50,137,71,139,75,
    42,205,35,214,162,199,147,24,250,131,35,158,58,156,234,139,
    43,7,149,67,241,84,58,53,140,199,24,30,163,124,12,166,
    118,96,95,230,191,232,98,71,94,
};

EmbeddedPython embedded_m5_util_convert(
    "m5/util/convert.py",
    "/home/oliverxyy/program/gem5-stable/src/python/m5/util/convert.py",
    "m5.util.convert",
    data_m5_util_convert,
    2521,
    9254);

} // anonymous namespace
