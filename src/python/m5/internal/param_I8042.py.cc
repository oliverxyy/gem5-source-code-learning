#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_I8042[] = {
    120,156,189,88,109,115,219,198,17,222,3,64,74,164,36,235,
    93,242,139,108,49,113,108,179,158,70,76,93,59,246,76,92,
    183,142,147,206,56,211,200,46,232,142,29,38,83,20,34,78,
    20,40,2,224,0,39,219,204,72,157,105,229,105,243,173,159,
    250,3,250,161,31,250,111,250,59,250,39,218,221,61,0,4,
    41,41,205,76,75,89,228,249,120,183,183,183,123,251,236,203,
    93,27,210,127,37,252,254,162,6,144,252,75,0,120,248,17,
    208,3,8,4,180,4,8,41,192,91,129,253,18,196,119,193,
    43,193,59,128,150,1,210,128,99,236,152,240,181,1,225,44,
    175,41,67,207,228,17,1,131,42,72,11,90,37,120,25,46,
    130,37,203,176,95,133,248,119,32,132,8,5,188,242,166,192,
    155,134,119,200,29,59,21,102,56,13,52,88,229,193,10,120,
    51,60,88,5,111,150,59,51,48,88,0,57,11,173,57,34,
    107,93,64,182,183,145,237,60,179,253,39,177,245,112,102,21,
    188,11,68,142,114,125,69,148,22,81,242,126,243,204,101,33,
    147,114,17,90,75,89,127,185,208,95,41,244,87,11,253,181,
    66,127,189,208,191,88,232,95,42,244,47,23,250,87,10,253,
    141,66,255,106,161,127,173,208,223,228,62,106,187,4,221,26,
    116,223,131,238,251,176,139,6,88,204,53,187,14,210,132,238,
    7,208,250,0,36,126,174,195,49,218,200,91,42,172,184,193,
    43,150,243,21,55,121,197,45,104,221,2,137,159,155,122,69,
    25,154,245,53,180,187,255,111,252,87,71,187,131,154,197,230,
    181,140,19,63,10,29,63,220,141,124,131,230,203,212,16,74,
    218,212,76,165,112,121,66,112,249,7,48,86,60,35,133,203,
    17,32,99,65,186,244,12,56,226,206,145,1,131,58,28,10,
    232,90,224,153,112,136,219,148,72,128,142,128,99,3,190,49,
    137,224,8,91,11,141,122,13,44,165,177,210,101,163,106,78,
    83,112,84,130,195,18,52,95,29,26,52,176,95,129,248,239,
    240,237,6,51,157,102,166,6,28,98,107,193,177,5,71,101,
    120,137,68,56,212,173,144,250,226,213,33,106,138,35,205,186,
    133,210,110,23,212,37,85,60,63,14,221,64,178,234,78,223,
    141,221,192,121,250,224,163,187,119,234,213,140,34,74,182,250,
    174,218,179,121,137,73,103,17,244,21,179,138,66,169,102,176,
    179,235,135,158,19,68,222,65,79,170,105,226,227,236,250,61,
    233,56,60,249,52,232,71,177,250,60,142,163,216,166,227,228,
    193,94,228,230,43,232,48,219,189,40,145,117,218,141,183,177,
    137,189,34,234,221,62,115,36,1,88,76,90,236,201,164,29,
    251,125,133,86,210,28,137,154,184,213,201,62,220,36,207,176,
    105,236,69,129,108,68,61,31,77,250,118,48,104,244,227,168,
    131,250,53,58,50,184,247,97,162,220,157,158,108,236,28,248,
    61,175,241,234,193,199,141,254,64,237,69,97,35,184,215,240,
    67,37,241,80,122,141,194,113,108,225,244,18,49,126,227,119,
    28,159,85,114,246,100,175,47,227,57,26,189,76,155,138,5,
    49,43,202,194,20,117,49,135,189,18,126,77,177,97,204,136,
    109,159,148,106,147,162,132,37,171,136,30,50,169,128,125,3,
    226,13,194,70,23,63,130,140,137,8,105,210,156,193,115,191,
    166,211,208,163,93,147,44,174,7,15,25,79,8,44,164,124,
    72,38,14,129,65,81,130,110,25,52,88,16,99,26,61,241,
    128,90,36,39,54,6,50,183,32,249,43,224,233,34,76,14,
    33,133,208,177,9,34,92,0,85,165,232,129,163,107,184,225,
    31,25,133,205,58,137,191,205,136,80,123,126,18,189,9,249,
    220,169,207,224,105,226,201,60,31,60,219,233,202,182,74,54,
    113,224,171,232,160,214,118,195,48,82,53,215,243,106,174,82,
    177,191,115,160,100,82,83,81,237,70,82,39,83,218,139,25,
    168,114,126,131,126,6,34,50,56,130,72,255,240,252,182,194,
    31,203,252,131,173,144,72,133,128,216,139,188,4,199,137,69,
    71,42,155,132,84,116,200,17,11,194,120,113,136,148,182,71,
    186,11,248,251,113,38,9,131,178,94,206,32,148,200,222,174,
    170,50,26,221,36,113,88,18,26,103,224,17,227,215,110,239,
    64,50,119,68,143,66,129,168,171,101,152,48,244,46,146,26,
    153,214,172,74,24,133,222,0,37,243,219,183,104,211,139,12,
    192,89,134,224,42,194,111,10,219,50,254,95,22,107,70,219,
    74,65,87,206,128,71,97,79,1,155,93,164,150,71,16,30,
    99,136,169,27,28,35,88,27,246,196,247,169,71,139,237,13,
    106,174,82,115,141,154,205,76,225,201,105,61,55,174,245,125,
    218,201,96,85,89,41,50,138,153,41,229,141,120,211,165,161,
    55,97,16,108,146,87,24,228,59,67,175,176,40,96,198,143,
    168,69,82,246,55,19,146,23,20,158,201,123,152,25,57,10,
    66,158,122,67,71,224,35,178,23,72,245,233,12,195,54,1,
    179,136,206,78,1,157,54,89,135,161,105,95,202,34,160,67,
    20,26,148,246,21,98,85,58,229,140,107,212,188,55,249,131,
    30,194,171,115,2,94,159,208,166,11,41,188,230,24,86,85,
    252,46,24,109,51,61,253,60,27,46,143,193,138,48,101,157,
    130,169,155,212,51,79,234,123,110,112,74,181,252,101,1,78,
    36,152,81,84,102,27,59,131,117,210,161,8,164,117,76,234,
    47,195,117,204,211,6,231,233,143,56,79,115,174,231,74,75,
    7,100,147,99,178,238,148,232,48,118,77,88,75,243,111,82,
    193,22,85,121,59,168,69,187,53,197,218,82,252,124,120,35,
    217,186,145,124,130,145,177,246,136,99,146,142,141,58,250,197,
    178,79,209,139,150,126,254,182,45,57,247,241,47,199,209,193,
    202,225,192,229,164,57,21,49,181,74,71,105,100,103,204,97,
    59,81,49,69,235,9,159,114,53,63,101,18,250,11,218,166,
    202,71,108,138,117,196,79,85,176,44,142,142,207,92,81,241,
    44,126,63,165,99,39,125,37,80,221,109,55,181,164,172,4,
    169,99,255,120,4,35,19,83,193,110,32,207,223,100,216,40,
    15,177,65,95,51,3,250,159,129,171,76,1,127,2,178,62,
    26,57,5,122,238,23,100,238,101,34,255,45,176,71,156,146,
    231,57,174,52,41,183,51,5,134,155,228,62,147,234,180,255,
    5,124,87,112,167,44,57,155,105,29,89,76,206,86,30,147,
    24,54,63,40,1,91,163,193,139,204,178,231,38,68,166,35,
    210,208,67,135,1,63,47,2,49,34,79,14,67,211,122,3,
    135,100,249,102,136,32,74,111,87,196,178,81,192,197,79,168,
    185,147,67,66,100,99,19,17,107,19,206,206,194,142,142,246,
    95,211,222,22,75,59,63,197,249,0,89,63,109,62,118,158,
    60,251,213,179,237,166,230,149,163,190,148,161,254,78,142,122,
    201,249,233,29,95,43,168,53,200,216,199,134,192,123,37,150,
    100,116,141,179,64,150,160,85,38,255,224,210,89,164,238,35,
    178,200,69,113,110,36,249,241,177,108,235,3,203,237,173,77,
    73,205,219,9,71,4,178,230,195,158,27,236,120,238,163,3,
    218,132,118,106,103,14,101,100,98,47,20,197,38,103,16,103,
    73,206,63,239,101,226,191,158,112,52,248,24,121,230,98,51,
    246,189,168,205,33,224,197,158,172,5,50,216,193,27,227,158,
    223,175,237,246,220,14,91,195,76,213,122,150,169,165,216,156,
    227,37,68,114,155,218,168,214,142,66,12,205,7,109,21,197,
    53,79,226,101,74,122,181,15,107,28,215,107,126,82,115,119,
    112,214,109,43,13,238,81,207,228,250,212,141,59,9,151,162,
    251,111,168,123,14,214,116,240,118,236,99,57,254,6,242,4,
    170,47,111,121,152,230,66,91,251,10,110,136,215,36,53,208,
    209,137,202,9,123,139,154,31,193,249,68,243,187,218,0,144,
    208,17,149,197,21,163,98,232,139,41,17,60,39,218,228,164,
    67,254,237,135,56,164,126,226,73,221,178,76,148,114,138,110,
    228,212,86,40,152,183,170,217,224,12,183,179,60,56,151,13,
    94,224,118,158,7,23,178,193,69,110,151,120,112,57,123,111,
    90,225,193,85,104,173,209,35,9,141,172,147,255,79,253,175,
    254,207,94,116,14,254,243,251,255,171,219,219,247,207,89,106,
    251,1,164,89,255,44,151,23,69,149,230,52,226,186,34,187,
    88,20,245,225,23,139,165,81,252,57,237,88,186,74,106,147,
    108,76,92,57,14,22,122,203,63,12,61,248,100,9,252,56,
    215,227,152,107,156,193,10,91,74,223,163,216,82,226,101,120,
    25,107,97,139,107,225,135,84,11,31,178,210,142,161,203,225,
    33,248,74,185,238,243,216,132,242,141,83,208,95,151,186,36,
    149,219,239,203,208,179,111,67,177,122,229,233,9,219,155,226,
    210,119,80,40,53,76,177,130,229,234,73,239,162,136,91,208,
    139,77,86,202,253,105,242,198,99,100,254,37,67,102,157,175,
    150,121,216,181,31,82,195,129,54,143,177,246,207,243,163,223,
    24,135,93,20,4,110,232,57,252,156,133,151,162,239,39,192,
    162,135,95,122,138,131,234,210,216,10,207,85,238,144,223,217,
    179,196,140,46,48,249,136,170,141,209,238,203,193,78,228,198,
    30,230,26,229,244,253,144,25,254,87,34,226,75,250,143,79,
    168,171,99,43,131,232,32,145,35,188,191,159,130,24,147,99,
    143,140,50,108,216,155,61,217,147,74,22,65,205,98,164,175,
    1,158,196,28,30,13,240,154,198,151,31,252,221,115,156,243,
    72,126,63,67,158,71,196,124,5,56,249,137,50,166,191,85,
    193,127,70,165,92,17,92,75,140,189,115,107,153,168,213,165,
    254,32,177,57,10,206,231,64,226,55,217,44,175,19,230,216,
    150,219,110,160,223,212,248,213,200,190,14,233,93,223,190,149,
    3,146,0,193,247,43,125,123,197,56,192,117,14,151,53,246,
    79,105,156,158,59,130,123,91,153,66,91,90,33,170,162,67,
    213,140,14,226,182,124,142,115,6,83,169,43,167,18,55,253,
    64,191,67,170,197,177,121,47,118,177,191,58,54,154,200,216,
    119,123,254,183,146,225,117,146,223,167,110,226,183,159,251,209,
    103,242,181,223,150,103,108,58,156,191,120,186,80,131,68,201,
    224,196,98,25,30,4,206,151,50,136,226,193,151,145,39,217,
    3,139,243,143,61,47,182,221,176,35,157,215,146,42,196,19,
    231,243,56,45,15,53,143,140,234,116,69,70,105,207,80,4,
    39,211,211,219,60,117,254,73,47,106,239,75,47,165,185,122,
    54,205,103,81,64,167,77,88,201,166,21,193,240,212,67,165,
    122,113,116,72,23,110,164,200,41,87,40,118,171,88,118,124,
    60,212,152,185,142,46,78,243,10,193,159,237,81,244,235,226,
    210,115,240,65,125,255,209,15,51,143,168,240,76,94,96,67,
    15,181,149,249,10,250,35,165,26,83,84,49,217,88,230,236,
    66,197,154,157,169,88,149,41,147,223,217,230,240,182,91,181,
    42,51,179,226,236,191,77,244,226,170,177,185,92,17,255,1,
    36,201,117,52,
};

EmbeddedPython embedded_m5_internal_param_I8042(
    "m5/internal/param_I8042.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_I8042.py",
    "m5.internal.param_I8042",
    data_m5_internal_param_I8042,
    2452,
    7375);

} // anonymous namespace