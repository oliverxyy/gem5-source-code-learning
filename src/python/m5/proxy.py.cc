#include "sim/init.hh"

namespace {

const uint8_t data_m5_proxy[] = {
    120,156,197,89,109,143,220,86,21,62,182,231,101,189,59,251,
    146,108,179,73,232,75,38,77,67,134,138,100,1,145,54,129,
    168,52,77,210,180,68,172,34,79,233,54,91,34,107,118,124,
    103,119,38,51,246,172,237,73,119,164,9,18,9,18,124,6,
    241,149,31,82,62,34,129,224,11,72,72,253,10,66,240,133,
    223,128,4,231,57,247,218,158,201,54,82,65,19,200,102,157,
    251,98,95,159,251,156,231,156,243,92,167,77,230,79,153,127,
    223,174,19,37,191,227,70,192,127,45,234,19,237,112,195,38,
    101,81,199,162,192,161,31,19,61,33,186,183,99,83,80,34,
    101,203,104,57,31,117,40,168,100,163,213,124,180,68,193,66,
    54,234,230,163,101,10,22,209,217,169,80,176,148,189,160,150,
    79,87,73,85,41,88,38,181,64,193,10,41,151,71,237,157,
    197,108,208,149,193,5,25,92,162,96,149,130,53,218,179,105,
    167,150,25,190,76,59,203,104,52,27,199,120,55,221,127,241,
    159,173,212,229,230,59,173,68,221,141,163,195,113,59,219,183,
    141,81,236,123,151,27,138,13,178,176,8,44,179,177,2,26,
    14,48,64,163,100,48,96,235,85,89,76,47,73,183,106,80,
    216,89,0,2,104,184,164,22,197,108,11,109,141,6,155,234,
    53,27,11,252,154,182,195,23,199,188,252,6,94,126,134,27,
    19,139,38,68,62,55,108,105,136,33,104,56,104,52,27,22,
    223,179,213,40,241,53,173,241,197,79,84,43,110,239,243,63,
    253,78,186,56,53,48,26,166,184,105,43,10,85,186,132,241,
    193,168,159,118,135,253,174,138,27,120,169,204,202,83,75,210,
    40,86,113,139,254,104,216,0,58,114,73,190,197,151,205,253,
    104,160,54,163,126,247,161,138,15,199,227,205,97,28,237,197,
    173,193,230,158,26,92,190,152,164,173,221,190,218,76,226,246,
    230,112,156,238,71,225,230,224,50,238,56,28,95,226,62,182,
    236,251,221,176,155,250,254,5,172,87,193,170,150,203,63,109,
    203,160,144,35,113,151,100,211,61,162,248,37,105,88,84,227,
    22,187,226,145,69,225,171,102,174,22,95,49,147,220,96,239,
    96,174,2,239,112,131,113,100,191,156,196,180,13,54,157,108,
    54,64,238,45,217,119,19,251,196,251,239,182,98,21,166,233,
    50,222,27,133,157,81,162,2,33,70,10,139,46,9,80,30,
    154,158,157,65,54,108,165,251,13,244,60,44,39,247,37,243,
    3,169,42,32,37,105,236,251,95,197,114,53,193,104,157,49,
    226,95,187,98,229,156,113,50,164,174,105,206,244,132,242,76,
    52,38,17,27,197,96,240,224,198,99,155,66,54,210,166,212,
    1,80,79,108,234,149,132,97,54,218,150,16,202,17,84,44,
    161,206,87,64,203,86,24,70,105,61,81,105,189,149,166,113,
    119,119,148,170,250,133,243,201,133,122,20,214,197,208,122,180,
    219,83,237,84,240,20,206,241,158,226,52,249,164,155,238,167,
    43,220,189,158,61,118,43,142,163,56,197,109,201,104,168,98,
    15,155,209,100,100,166,165,88,221,247,53,198,229,12,94,12,
    202,19,15,91,253,145,154,27,176,222,11,252,248,101,172,179,
    32,136,174,90,142,181,108,181,51,206,149,51,52,97,98,42,
    128,50,34,26,183,142,3,172,146,215,96,31,67,172,33,5,
    173,202,8,200,7,54,197,223,201,130,182,76,225,42,26,37,
    158,227,161,171,54,143,76,128,49,222,177,149,212,65,56,1,
    176,136,69,110,38,105,125,87,213,187,97,170,246,56,54,171,
    25,168,221,164,27,242,158,194,182,74,1,17,207,11,64,253,
    40,220,19,128,58,253,136,193,66,176,126,48,30,106,168,61,
    220,224,225,238,41,134,226,18,165,251,188,244,156,89,202,155,
    240,253,107,5,75,79,89,53,198,181,230,172,22,184,230,241,
    252,74,30,207,6,179,117,0,212,212,184,241,232,11,89,98,
    19,187,63,111,27,136,214,88,37,12,220,252,246,225,154,188,
    216,222,87,237,7,239,21,57,105,213,42,233,61,148,167,185,
    225,90,66,140,71,54,24,242,200,201,50,207,69,210,57,6,
    49,101,209,15,56,11,149,48,107,72,226,80,252,51,48,71,
    211,3,13,27,141,195,111,208,196,161,100,130,110,175,130,53,
    185,145,4,116,215,60,118,100,185,131,59,180,157,82,182,140,
    24,2,186,145,44,243,107,98,151,4,26,75,212,67,105,184,
    212,91,148,204,32,35,204,226,44,27,44,209,164,132,226,193,
    172,142,255,137,246,68,187,228,31,196,121,144,19,135,97,120,
    137,122,43,198,10,54,193,216,181,138,241,39,86,150,54,146,
    77,128,211,10,47,164,117,246,77,212,127,168,76,134,208,249,
    162,83,79,153,154,186,211,137,163,129,180,146,53,60,51,110,
    247,193,249,250,40,148,7,26,107,25,85,223,109,245,19,37,
    57,87,232,222,233,134,65,145,124,63,136,71,74,23,146,135,
    28,30,169,10,52,21,135,146,197,189,227,32,203,114,70,22,
    127,24,168,164,45,62,30,194,12,100,85,239,36,110,145,52,
    4,206,122,163,48,237,14,76,154,194,74,198,26,15,133,88,
    210,91,145,152,118,89,51,72,32,114,242,147,129,128,43,171,
    247,74,70,197,185,36,41,44,242,61,172,115,73,104,88,177,
    42,182,107,157,178,93,199,181,244,79,133,137,121,154,199,106,
    92,10,28,254,57,111,115,192,241,207,113,251,104,42,123,79,
    23,6,19,111,107,146,136,198,43,248,135,71,79,63,34,218,
    14,207,81,137,211,220,131,69,138,111,144,101,89,184,155,187,
    76,217,155,244,152,232,224,38,133,22,125,84,36,176,46,229,
    33,41,73,251,197,188,251,122,230,61,246,150,58,156,175,102,
    216,83,169,172,26,98,189,53,1,166,198,48,56,214,10,103,
    240,154,181,232,28,205,54,203,52,165,163,166,84,147,84,242,
    243,71,211,163,48,101,126,86,163,6,50,89,64,203,214,192,
    199,218,63,196,136,45,182,55,64,60,35,134,194,214,64,249,
    190,150,109,254,32,10,70,125,238,122,152,243,86,113,65,209,
    242,94,46,238,143,37,229,10,59,133,43,222,197,140,201,108,
    80,218,109,15,20,219,18,120,155,51,164,156,11,51,177,198,
    151,177,14,76,170,88,110,217,117,221,170,187,200,252,44,187,
    103,221,165,90,89,226,12,133,127,86,87,3,113,209,213,168,
    189,95,92,87,27,69,93,129,84,134,131,218,8,56,252,230,
    178,7,123,76,69,127,25,93,99,79,233,154,137,99,20,243,
    158,206,152,165,105,18,72,88,175,227,242,77,92,22,50,18,
    248,34,61,196,23,236,138,110,135,139,115,34,58,91,120,226,
    33,165,120,200,17,222,137,185,34,139,247,63,166,188,244,156,
    134,28,182,233,41,145,215,161,89,145,199,181,92,235,19,179,
    121,199,132,53,183,155,186,216,4,130,4,199,53,239,141,97,
    53,169,191,66,189,170,60,99,153,146,195,197,129,11,197,68,
    22,181,184,223,148,29,111,121,107,89,164,36,175,210,148,160,
    227,44,223,81,156,107,219,10,98,112,55,26,133,129,78,248,
    13,224,230,29,155,129,214,40,189,189,76,233,73,134,221,111,
    37,232,73,174,150,52,218,142,76,156,7,74,13,209,241,222,
    32,147,193,91,195,161,10,131,66,32,10,242,114,111,168,62,
    145,179,202,252,220,240,38,63,254,19,172,131,224,37,78,171,
    167,161,101,74,171,214,218,231,184,227,30,77,169,68,136,195,
    23,225,139,92,28,106,177,109,60,240,134,232,198,220,3,101,
    148,251,25,15,84,225,132,35,30,72,206,81,38,22,181,218,
    102,232,15,70,93,174,178,153,82,172,75,86,212,78,66,25,
    122,31,221,122,196,58,155,83,1,123,231,41,7,33,62,117,
    249,59,69,38,119,123,87,112,145,170,121,21,23,64,40,232,
    123,223,6,172,133,48,71,235,129,26,123,215,230,201,252,130,
    30,92,200,7,190,255,139,28,125,32,111,126,215,10,29,86,
    202,208,63,206,113,60,254,208,56,64,139,48,6,26,178,204,
    134,82,2,250,226,12,160,127,77,110,203,186,70,150,177,98,
    186,131,65,28,23,23,145,26,248,193,14,35,191,29,174,160,
    12,22,3,31,29,158,129,199,180,8,138,127,106,196,154,246,
    21,191,239,160,71,219,135,19,49,161,74,55,239,31,64,39,
    49,61,161,174,92,121,251,167,230,237,208,76,98,97,216,200,
    110,88,148,27,62,51,179,167,49,87,3,101,146,191,34,110,
    30,91,52,243,106,199,122,250,213,127,167,237,131,159,211,54,
    15,115,254,135,173,90,144,121,231,114,58,48,168,249,225,130,
    165,79,55,220,171,71,113,126,204,16,153,132,128,52,225,233,
    33,65,123,245,156,21,78,54,223,77,180,36,106,228,228,56,
    153,81,2,130,10,100,210,135,190,36,81,49,136,167,79,34,
    72,27,133,130,210,2,1,207,240,177,78,150,205,244,27,114,
    242,96,126,97,124,150,31,255,37,214,121,93,136,228,88,199,
    89,51,173,178,62,170,148,92,123,133,251,75,220,91,103,90,
    241,24,255,46,219,32,217,73,253,1,194,153,38,217,128,178,
    3,11,179,228,240,190,225,217,205,251,158,33,218,68,206,134,
    240,225,85,210,186,137,71,54,222,228,155,15,214,179,121,241,
    122,124,79,230,237,233,121,78,7,201,58,220,251,88,186,219,
    56,10,153,176,135,17,151,206,39,9,122,31,159,15,238,127,
    17,95,74,105,186,60,227,31,239,122,30,233,239,204,198,50,
    118,58,244,110,205,51,146,69,115,255,138,114,121,230,10,190,
    199,240,107,175,89,114,162,245,190,134,203,215,201,20,92,73,
    183,222,91,184,192,99,178,192,156,21,11,42,208,143,176,14,
    42,147,40,150,21,214,44,47,75,249,184,30,142,159,33,83,
    190,68,207,150,41,158,150,193,71,229,38,42,158,174,206,154,
    37,61,91,31,145,180,222,144,72,194,91,113,148,241,91,225,
    88,82,182,214,157,56,151,20,90,84,162,100,190,177,240,91,
    202,133,167,112,60,251,21,187,229,20,99,101,234,94,194,147,
    205,211,6,88,153,81,243,101,201,239,11,123,180,182,42,120,
    241,188,136,112,155,31,255,13,153,237,50,17,28,77,129,126,
    255,255,70,129,126,95,40,224,189,79,211,39,145,231,224,253,
    63,254,167,222,239,247,159,171,247,255,244,191,247,254,29,126,
    252,15,211,222,207,191,53,231,169,62,83,115,100,52,53,171,
    233,142,214,110,47,33,135,55,205,92,25,162,212,76,124,159,
    14,207,98,240,230,253,13,212,7,174,104,186,50,199,87,228,
    137,131,43,124,184,22,151,103,12,40,212,23,54,37,234,86,
    78,135,137,156,222,110,225,243,92,168,210,235,65,160,43,167,
    254,210,215,77,82,73,19,233,104,216,87,222,219,100,10,244,
    212,145,27,139,63,156,31,88,120,197,103,148,203,176,83,124,
    204,62,33,135,236,37,75,236,148,136,121,183,213,78,163,248,
    191,12,156,35,255,229,177,65,207,248,47,143,153,15,130,249,
    225,107,234,48,80,140,205,109,255,8,209,191,100,100,161,236,
    36,54,67,150,161,182,23,223,73,152,7,103,12,113,122,150,
    249,106,135,3,152,46,247,152,254,208,140,242,49,12,37,255,
    6,36,3,151,252,144,204,23,193,153,199,210,242,204,16,8,
    229,80,83,144,208,103,178,239,98,195,242,121,8,226,253,46,
    211,39,76,47,113,200,214,187,73,93,190,216,143,134,195,40,
    102,81,165,5,193,237,25,144,68,6,72,52,72,101,156,74,
    59,243,61,206,162,176,255,173,160,80,141,197,86,141,129,100,
    209,101,63,29,242,185,16,152,115,200,55,249,241,63,79,133,
    124,105,10,5,57,166,34,24,27,176,79,142,61,18,140,250,
    140,37,193,169,63,11,220,206,225,146,184,107,102,193,39,225,
    89,44,99,62,133,179,23,124,223,219,154,251,94,164,104,92,
    211,95,134,222,122,141,204,247,151,154,189,241,241,198,173,13,
    119,163,234,186,27,107,39,172,19,78,205,249,55,144,44,226,
    110,
};

EmbeddedPython embedded_m5_proxy(
    "m5/proxy.py",
    "/home/oliverxyy/program/gem5-stable/src/python/m5/proxy.py",
    "m5.proxy",
    data_m5_proxy,
    2657,
    7618);

} // anonymous namespace
