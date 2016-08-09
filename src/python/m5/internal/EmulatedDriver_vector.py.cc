#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_EmulatedDriver_vector[] = {
    120,156,205,92,125,104,100,87,21,63,239,205,71,118,38,201,
    38,217,108,146,205,110,186,59,187,237,182,211,106,55,126,116,
    219,106,203,182,181,93,75,43,110,237,75,101,183,105,245,245,
    101,222,77,242,146,153,247,166,239,189,236,238,212,172,82,179,
    90,63,81,241,3,21,17,17,68,69,4,65,16,4,65,16,
    4,65,16,5,65,16,4,161,32,8,130,224,127,130,80,207,
    239,188,247,230,43,51,161,187,179,153,215,77,230,236,157,59,
    119,238,61,191,123,126,247,220,123,207,187,55,21,138,255,229,
    248,245,104,137,40,120,73,35,178,249,87,163,42,81,77,163,
    101,141,52,165,145,125,152,54,115,228,223,71,118,142,174,19,
    45,235,164,116,218,225,68,134,94,208,201,29,147,239,228,169,
    154,145,28,141,26,69,82,89,90,206,209,69,119,138,178,42,
    79,155,69,242,95,34,77,211,92,141,46,217,35,100,31,160,
    235,92,59,39,10,82,225,1,66,102,81,50,11,100,143,74,
    102,145,236,49,73,140,82,99,146,212,24,45,143,163,216,242,
    65,174,246,30,174,118,66,170,253,35,170,181,249,147,25,178,
    15,162,56,235,245,60,74,102,81,82,218,155,144,90,38,201,
    150,90,86,25,207,84,179,224,20,169,12,109,28,162,229,67,
    164,248,119,138,118,24,114,12,103,154,150,15,39,208,102,218,
    210,179,109,233,57,73,31,74,42,158,110,86,124,68,42,158,
    167,229,121,82,252,123,36,170,56,79,75,229,89,238,109,231,
    13,254,87,230,222,166,112,140,197,101,229,7,142,231,154,142,
    187,234,57,58,62,207,67,192,54,21,136,145,216,72,143,195,
    72,63,38,177,144,173,199,70,186,70,92,177,6,133,170,58,
    93,147,196,53,157,26,101,218,214,104,35,75,118,134,182,185,
    153,28,20,88,211,104,71,167,23,51,40,112,141,101,150,187,
    242,56,101,195,200,66,27,210,149,81,77,35,116,45,71,219,
    57,90,186,180,173,35,99,179,64,254,15,232,149,5,169,244,
    128,84,170,211,54,203,44,237,100,233,90,158,46,114,33,206,
    218,40,0,190,118,105,155,145,114,206,82,57,203,218,94,104,
    131,11,40,182,227,187,86,77,133,232,9,243,124,109,171,106,
    133,202,126,194,119,184,27,204,203,170,18,122,126,185,152,148,
    245,130,51,117,43,92,55,228,203,25,244,74,173,30,74,165,
    158,171,194,81,78,172,58,174,109,214,60,123,171,170,194,3,
    168,209,92,117,170,202,52,229,195,167,106,117,207,15,207,251,
    190,231,27,232,88,201,172,122,86,243,27,232,214,74,213,11,
    84,25,173,73,51,6,170,15,81,122,181,46,53,66,1,81,
    24,95,182,85,80,241,157,122,200,246,138,106,68,105,212,86,
    134,165,68,4,207,179,88,92,247,106,106,209,171,2,213,213,
    70,99,177,238,123,107,190,85,91,92,83,181,179,247,6,161,
    181,82,85,139,43,91,78,213,94,188,244,224,253,139,245,70,
    184,238,185,139,181,179,139,142,27,42,238,158,234,98,207,142,
    57,195,5,15,161,137,43,206,154,233,8,56,115,93,85,235,
    202,31,71,238,81,52,175,77,106,99,90,94,203,104,101,109,
    156,83,57,126,101,180,5,125,84,187,224,0,94,5,144,193,
    175,108,59,163,96,102,141,54,117,242,23,192,151,13,254,213,
    96,96,102,205,18,62,211,229,179,103,209,47,81,238,70,6,
    44,136,50,183,133,99,76,54,46,249,48,204,238,146,16,37,
    71,27,121,138,8,196,188,139,24,229,55,32,185,56,170,209,
    185,242,44,5,95,39,238,103,166,206,54,197,180,218,201,144,
    230,78,82,88,196,56,230,220,89,110,240,147,194,204,165,50,
    212,191,32,220,8,215,157,192,187,226,138,5,144,150,177,180,
    196,61,243,161,198,51,43,27,220,95,193,9,206,120,222,219,
    42,85,44,215,245,194,146,101,219,37,43,12,125,103,101,43,
    84,65,41,244,74,167,131,50,140,106,76,37,244,106,214,215,
    168,39,116,130,233,153,78,209,27,219,169,132,252,102,90,222,
    136,21,2,21,50,53,214,61,59,224,124,84,177,166,66,3,
    74,134,232,100,79,20,17,230,152,40,138,230,185,220,65,126,
    255,88,162,137,208,179,156,79,200,20,168,234,106,88,20,94,
    90,65,96,138,38,200,23,10,162,226,203,86,117,75,73,237,
    204,163,144,21,66,50,210,97,104,36,60,2,64,9,126,1,
    229,122,174,221,96,29,157,202,93,104,254,136,80,113,76,200,
    56,195,68,28,97,153,231,255,243,218,172,94,201,198,244,203,
    39,20,132,43,8,73,8,160,197,28,96,58,238,176,3,42,
    235,226,65,4,151,140,206,83,72,225,203,198,2,196,109,16,
    199,33,78,36,208,135,129,127,188,27,255,3,104,83,23,208,
    2,15,134,202,36,240,236,142,17,54,223,26,97,236,44,151,
    48,82,116,140,167,214,72,201,194,177,250,231,32,185,168,140,
    193,12,5,207,193,141,99,68,73,101,24,60,60,12,144,106,
    13,14,233,44,99,18,157,112,32,225,181,1,178,182,51,118,
    173,141,177,6,236,36,116,53,230,19,255,104,162,68,68,84,
    227,24,170,202,245,232,237,18,196,201,97,118,121,139,114,107,
    187,40,247,16,154,159,140,41,55,46,84,43,242,107,82,175,
    100,98,59,52,231,207,233,46,170,129,103,217,30,60,187,19,
    169,204,110,228,41,80,44,198,251,254,54,138,65,69,189,29,
    214,5,78,52,230,128,166,157,92,115,188,32,184,232,206,241,
    28,175,203,28,255,14,153,227,101,157,32,107,163,200,113,103,
    196,119,71,137,28,186,101,53,67,179,241,220,29,20,88,50,
    168,171,141,146,183,90,10,5,55,252,236,195,167,131,51,167,
    131,135,216,131,150,206,137,239,138,124,104,228,37,125,85,135,
    151,195,87,207,95,173,40,153,45,229,157,105,70,78,205,20,
    7,103,198,179,48,243,108,6,157,170,39,189,45,238,61,8,
    125,120,245,161,245,119,177,217,223,80,255,105,52,88,148,206,
    206,104,115,204,169,162,38,90,153,145,71,151,117,153,124,202,
    175,247,193,0,64,174,8,107,102,99,41,210,89,224,0,152,
    241,246,14,222,12,1,140,177,200,181,127,56,225,75,190,197,
    23,188,50,201,48,120,141,100,213,170,209,167,9,140,96,195,
    199,195,160,57,106,64,129,105,20,255,40,201,120,233,177,70,
    16,255,179,132,117,129,148,96,183,20,60,32,69,163,37,195,
    211,244,153,182,193,150,76,236,153,120,93,218,62,177,103,155,
    190,75,168,244,166,38,239,108,167,147,131,129,214,173,0,197,
    34,207,213,26,191,173,41,162,185,148,100,207,61,12,94,29,
    136,154,50,161,213,139,45,86,97,106,60,166,77,235,109,92,
    121,39,196,187,154,52,209,146,188,125,86,240,4,245,159,203,
    205,104,166,120,1,90,100,69,239,137,17,89,189,68,235,172,
    167,184,110,139,171,105,14,135,92,50,28,94,111,14,7,37,
    211,218,117,217,181,64,234,176,253,142,174,241,102,145,87,119,
    216,155,101,73,229,104,57,79,106,4,155,11,108,1,115,201,
    22,48,31,111,1,91,187,198,49,73,23,36,61,46,187,70,
    194,86,47,222,53,78,36,187,70,222,239,141,75,98,42,222,
    24,242,22,47,222,10,78,99,43,136,196,225,120,43,184,60,
    131,61,28,18,179,241,30,110,121,14,187,94,36,142,96,107,
    137,196,60,217,179,146,56,74,246,156,36,142,97,164,99,86,
    146,209,149,188,196,27,195,119,119,76,242,98,204,11,145,153,
    155,124,141,168,8,113,117,104,94,14,108,124,184,106,213,86,
    108,235,220,26,154,67,155,149,196,53,232,9,128,201,118,0,
    24,214,90,63,12,242,246,108,2,228,242,208,60,220,253,92,
    123,19,128,140,103,219,171,136,91,123,110,93,149,106,170,182,
    194,187,234,117,167,94,90,173,90,107,98,161,76,12,240,153,
    4,96,40,156,236,94,62,5,247,64,122,165,138,231,242,20,
    180,133,246,74,182,226,109,166,178,75,247,150,100,254,42,57,
    65,201,90,225,79,173,74,24,13,211,78,111,35,235,117,203,
    95,11,100,105,190,121,5,201,161,90,216,52,29,215,225,141,
    138,67,157,75,134,14,146,66,73,187,197,77,193,145,107,58,
    158,5,26,226,116,5,99,214,154,108,76,180,108,178,113,60,
    50,214,134,150,44,135,219,169,40,187,240,195,187,92,146,41,
    219,164,20,176,192,157,122,173,126,143,54,221,205,185,183,125,
    128,53,33,69,243,230,110,84,211,187,81,57,110,197,111,91,
    40,225,27,238,208,168,5,198,64,1,191,141,86,183,12,153,
    173,218,145,25,231,134,104,51,1,134,246,183,6,6,54,215,
    3,152,195,186,185,21,213,6,238,236,48,193,193,33,36,58,
    92,29,24,96,143,145,166,94,222,178,170,105,161,131,199,18,
    5,62,214,195,215,221,128,7,233,193,200,138,87,111,12,223,
    129,8,25,209,244,199,111,57,32,87,93,13,83,2,132,166,
    95,29,12,80,143,161,101,10,36,211,76,1,84,28,28,148,
    230,175,223,114,96,117,95,93,118,188,173,32,37,96,73,243,
    175,13,236,46,102,119,99,179,236,203,93,238,112,168,190,30,
    155,196,88,133,207,13,12,111,166,23,39,213,203,204,200,148,
    220,97,94,104,9,13,190,184,63,224,92,149,54,56,104,240,
    229,129,193,245,244,38,142,101,219,29,240,134,202,204,120,229,
    46,58,124,117,159,0,6,91,43,169,3,20,29,190,177,31,
    174,197,52,83,53,160,4,8,35,21,190,69,180,43,88,11,
    120,79,246,130,247,133,94,179,66,79,120,221,230,123,207,240,
    225,137,10,223,161,221,51,94,199,198,114,187,181,177,20,245,
    210,154,160,29,46,102,154,223,109,169,91,150,126,107,134,101,
    37,152,21,69,191,184,233,186,242,195,70,20,131,196,35,5,
    227,12,196,221,29,14,209,86,85,21,42,179,211,50,225,36,
    53,31,175,216,42,8,125,175,97,154,113,127,241,23,76,83,
    118,133,198,35,16,143,65,60,14,113,30,226,73,136,167,32,
    62,0,241,65,136,103,32,158,133,88,130,64,32,215,184,8,
    129,30,51,16,140,51,94,236,232,202,97,236,107,239,227,218,
    87,209,12,186,45,175,29,211,11,122,94,43,104,5,189,144,
    25,227,159,66,191,31,93,186,45,170,167,235,113,255,238,176,
    161,173,189,137,176,97,116,186,36,14,30,230,147,104,225,72,
    18,45,148,227,36,72,20,36,102,24,5,18,11,73,32,49,
    10,24,142,37,1,195,241,36,96,120,48,9,24,78,36,1,
    195,201,36,96,56,149,4,12,15,37,1,195,233,36,96,120,
    56,9,24,206,36,1,195,217,36,96,56,151,4,12,143,36,
    1,195,121,178,143,36,33,196,249,56,132,104,31,149,196,2,
    217,199,36,113,27,217,11,146,56,78,246,109,146,56,65,246,
    113,73,148,200,62,33,137,147,100,151,36,113,138,236,147,146,
    184,157,236,83,146,184,131,236,219,37,113,154,236,59,36,113,
    39,169,187,104,163,76,203,119,147,125,90,114,238,65,220,18,
    143,183,6,138,91,14,117,230,151,0,209,143,232,86,134,43,
    141,7,82,211,223,120,144,226,39,48,253,66,149,55,184,129,
    56,222,111,128,137,251,131,131,74,201,255,38,205,255,132,246,
    152,46,138,77,227,237,116,69,35,43,148,198,204,33,190,245,
    167,61,52,190,1,139,156,236,107,17,19,79,87,94,81,190,
    151,202,174,53,122,190,223,212,224,103,131,161,236,207,59,211,
    92,241,188,106,138,27,243,168,249,159,15,134,239,182,61,240,
    85,149,155,10,188,104,81,33,173,255,162,13,93,251,19,91,
    65,55,77,157,107,204,232,233,235,110,140,167,246,192,184,166,
    194,160,234,84,176,227,107,61,219,208,100,92,67,110,12,13,
    244,24,197,199,82,98,117,126,217,101,215,27,95,93,239,5,
    59,104,193,78,105,165,29,225,109,233,241,171,253,182,52,47,
    82,119,89,218,216,132,168,14,31,119,75,153,95,15,108,231,
    189,28,49,183,195,19,84,45,61,51,143,38,112,35,53,126,
    179,175,104,121,244,188,21,208,54,213,248,237,190,162,13,222,
    26,104,155,106,252,142,6,154,137,230,251,66,173,123,245,20,
    102,33,24,138,91,254,125,27,170,155,11,226,44,244,5,102,
    213,235,202,181,211,12,53,70,26,252,97,48,203,29,235,11,
    80,213,234,97,26,207,151,228,137,25,218,254,211,96,200,142,
    246,69,22,56,175,164,240,228,61,58,61,205,77,255,121,96,
    86,238,1,237,138,85,111,227,228,48,55,145,17,62,110,255,
    47,251,197,200,21,181,230,184,41,49,82,218,254,235,126,121,
    73,120,146,116,188,36,183,252,183,193,80,245,119,145,126,90,
    6,131,119,140,26,255,251,126,249,16,63,29,147,97,140,161,
    233,215,247,107,140,85,170,202,74,35,28,19,221,116,226,182,
    255,49,24,178,219,251,34,91,195,33,215,106,213,171,164,20,
    112,130,238,29,58,252,115,48,164,253,163,27,188,242,49,87,
    172,202,102,90,79,231,227,230,255,213,133,239,198,87,208,123,
    44,78,124,43,80,105,173,157,101,133,2,5,254,221,133,48,
    185,201,33,8,159,104,33,100,120,209,181,199,195,18,73,76,
    174,211,224,210,229,69,247,40,101,121,88,227,250,195,35,184,
    254,176,45,71,213,77,61,186,1,209,138,56,230,168,125,26,
    113,213,21,179,103,207,68,177,100,28,54,50,112,80,180,109,
    187,140,46,146,79,135,22,168,196,51,170,255,16,37,71,202,
    39,180,140,118,88,27,191,217,69,207,137,254,124,223,10,214,
    35,194,167,180,26,151,75,40,137,18,255,221,47,215,188,234,
    123,110,26,231,163,192,60,105,251,127,131,33,235,63,153,166,
    228,172,48,153,162,233,55,218,112,221,116,248,106,143,157,98,
    16,56,107,110,219,72,60,55,116,130,202,118,81,212,208,181,
    65,157,242,30,235,61,37,59,171,148,188,178,44,250,68,131,
    220,62,98,116,220,64,249,97,154,24,35,13,14,180,97,188,
    57,127,218,255,233,1,247,162,242,47,167,122,2,46,86,97,
    84,219,167,37,82,197,170,91,21,39,149,248,6,150,72,73,
    243,7,123,224,235,56,167,242,22,186,0,49,165,197,207,135,
    203,111,163,246,35,42,198,71,32,228,80,74,235,60,10,30,
    141,202,131,74,67,65,224,38,140,129,219,30,6,174,81,24,
    184,127,96,188,12,129,10,13,28,107,55,174,64,52,40,89,
    188,92,131,248,4,196,171,16,59,16,159,130,192,153,79,227,
    179,16,159,135,192,49,66,227,75,16,95,129,144,99,49,95,
    131,192,1,46,227,155,16,223,134,192,137,32,3,231,108,140,
    239,65,124,191,99,168,199,135,102,122,47,168,76,20,125,169,
    163,183,135,209,229,22,215,254,67,52,131,171,158,121,237,152,
    150,215,113,150,229,134,126,70,186,207,186,20,52,153,134,186,
    254,138,71,132,9,211,115,116,241,176,17,24,200,49,38,154,
    157,20,29,186,141,207,31,193,222,178,228,185,96,213,162,191,
    14,32,183,222,13,108,189,228,94,178,113,87,147,12,88,171,
    202,109,207,232,126,45,47,91,229,134,146,92,72,50,222,13,
    129,115,59,178,232,233,58,76,134,195,74,62,111,225,3,206,
    8,121,56,80,237,236,153,164,215,206,212,45,238,228,46,67,
    201,223,196,168,157,149,186,118,151,93,114,106,209,95,93,8,
    167,186,62,183,125,139,211,51,93,185,236,126,28,171,202,147,
    137,24,98,143,135,83,237,122,14,149,33,209,93,185,232,138,
    242,57,132,243,131,71,89,224,15,26,20,38,10,204,22,252,
    93,141,140,86,228,5,119,54,51,54,89,200,142,141,22,178,
    133,145,140,220,61,31,215,166,245,98,182,48,58,251,222,130,
    86,228,146,209,207,236,102,65,251,63,8,117,228,74,
};

EmbeddedPython embedded_m5_internal_EmulatedDriver_vector(
    "m5/internal/EmulatedDriver_vector.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/EmulatedDriver_vector.py",
    "m5.internal.EmulatedDriver_vector",
    data_m5_internal_EmulatedDriver_vector,
    3502,
    18295);

} // anonymous namespace