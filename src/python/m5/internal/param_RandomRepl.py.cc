#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_RandomRepl[] = {
    120,156,197,88,235,110,220,198,21,62,67,114,87,218,149,100,
    173,172,155,47,178,197,182,112,179,53,26,109,155,70,113,128,
    168,110,157,52,5,26,160,74,202,117,97,71,9,202,82,228,
    104,197,21,47,11,114,100,121,13,233,79,101,180,125,129,246,
    13,250,163,111,211,55,106,207,57,67,114,169,27,16,160,245,
    86,94,142,135,195,153,51,231,242,157,203,140,15,197,95,3,
    159,95,218,0,249,223,5,64,128,63,1,17,64,44,96,79,
    128,144,2,130,21,56,106,64,246,33,4,13,120,11,176,103,
    128,52,224,28,59,38,124,99,64,50,207,107,154,16,153,60,
    34,96,220,6,105,193,94,3,94,36,75,96,201,38,28,181,
    33,251,35,8,33,18,1,47,131,25,8,102,225,45,82,199,
    78,139,9,206,2,13,182,121,176,5,193,28,15,182,33,152,
    231,206,28,140,59,32,231,97,111,129,166,237,221,66,178,143,
    145,236,34,147,253,23,145,13,240,203,42,4,183,104,58,242,
    245,53,205,180,104,38,239,183,200,84,58,37,151,75,176,119,
    187,236,47,215,250,43,181,254,106,173,191,86,235,175,215,250,
    119,184,143,156,221,134,225,93,24,222,131,225,125,56,64,101,
    45,85,92,108,128,52,97,248,0,246,30,128,196,223,6,156,
    163,62,131,219,181,21,15,121,197,114,181,98,147,87,216,176,
    103,131,196,223,166,94,209,132,126,119,13,109,20,254,27,255,
    186,104,35,80,243,216,188,146,89,30,166,137,27,38,7,105,
    104,208,247,38,53,100,81,159,154,153,194,180,159,145,105,255,
    9,108,215,192,40,76,123,6,72,88,144,44,145,1,103,220,
    57,51,96,220,133,83,1,67,11,2,19,78,113,155,6,49,
    48,16,112,110,192,183,38,77,56,195,214,66,3,60,4,75,
    105,187,14,217,0,154,210,12,156,53,224,180,1,253,151,167,
    6,13,28,181,32,251,7,188,217,96,162,179,76,212,128,83,
    108,45,56,183,224,172,9,47,112,18,14,13,91,36,190,120,
    121,138,146,226,72,191,107,33,183,187,53,113,73,148,32,204,
    18,47,150,106,9,251,238,200,203,188,216,117,188,36,72,99,
    71,142,162,110,187,156,150,230,91,35,79,29,58,188,206,36,
    133,196,35,197,244,210,68,170,57,236,28,132,73,224,198,105,
    112,28,73,53,75,196,220,131,48,146,174,203,31,127,19,143,
    210,76,125,158,101,105,230,144,78,121,48,74,189,106,5,105,
    212,143,210,92,118,105,55,222,198,33,242,138,102,31,140,152,
    34,49,192,188,210,226,64,230,126,22,142,20,154,74,83,164,
    217,68,173,75,70,226,38,127,142,77,239,48,141,101,47,141,
    66,180,235,235,241,184,55,202,210,1,10,217,27,200,120,251,
    253,92,121,251,145,236,237,31,135,81,208,123,249,241,71,189,
    209,88,29,166,73,47,222,238,133,137,146,168,153,168,119,89,
    39,91,56,231,54,81,63,9,7,110,200,114,185,135,50,26,
    201,108,129,70,239,209,206,162,35,230,69,83,152,162,43,22,
    176,215,192,199,20,27,198,156,216,13,73,50,159,164,37,84,
    89,117,28,145,113,5,28,25,144,109,16,74,134,248,19,100,
    86,196,74,159,190,25,252,237,119,164,18,61,58,52,201,246,
    122,240,148,145,133,16,195,153,59,100,236,4,24,30,13,24,
    54,65,195,6,209,166,113,148,141,169,197,233,68,198,64,226,
    22,228,127,3,84,49,2,230,20,10,48,157,155,32,146,14,
    168,54,249,60,142,174,225,134,127,98,60,246,187,196,254,46,
    195,66,29,134,121,122,146,176,242,169,207,30,212,71,205,124,
    53,254,114,127,40,125,149,111,226,192,215,233,177,237,123,73,
    146,42,219,11,2,219,83,42,11,247,143,149,204,109,149,218,
    143,242,46,217,211,89,42,145,85,209,27,143,74,36,145,213,
    17,73,250,37,8,125,133,47,203,252,194,86,200,165,66,84,
    28,166,65,142,227,68,98,32,149,67,76,42,82,114,202,140,
    48,104,92,154,74,219,227,188,91,248,254,172,228,132,145,217,
    109,150,56,202,101,116,160,218,12,73,47,207,93,230,132,198,
    25,125,68,248,149,23,29,75,166,142,16,82,200,16,117,53,
    15,211,192,223,29,146,165,20,157,229,73,210,36,24,35,123,
    161,255,30,237,124,135,81,56,207,56,92,69,12,206,96,219,
    196,255,155,98,205,240,173,2,121,205,18,125,20,5,21,176,
    237,69,97,126,68,226,57,70,156,174,193,33,131,69,98,159,
    252,62,245,104,177,179,65,205,3,106,30,82,179,89,74,253,
    142,69,95,184,44,250,19,218,206,96,121,89,50,50,143,89,
    74,22,92,240,171,187,19,191,194,192,216,39,255,48,200,139,
    38,254,97,81,16,205,158,82,139,83,217,243,76,18,70,105,
    63,98,98,228,50,8,126,234,77,92,130,245,228,116,72,254,
    217,18,205,14,65,180,142,211,65,13,167,14,153,136,65,234,
    220,45,3,162,75,51,52,60,157,251,68,170,113,141,162,109,
    106,190,55,37,109,79,128,54,184,2,180,79,104,231,78,1,
    180,5,6,88,27,159,142,225,155,133,9,170,52,185,124,9,
    96,132,46,235,26,116,253,144,122,230,85,161,167,11,172,66,
    212,95,215,128,69,220,25,117,137,118,177,51,94,39,65,234,
    144,90,199,148,255,34,89,199,44,110,112,22,255,9,103,113,
    174,4,184,102,210,65,218,228,56,173,59,13,210,200,129,9,
    107,69,118,206,91,216,162,60,175,199,118,122,96,43,22,153,
    98,234,206,163,124,235,81,254,9,70,75,251,41,199,41,29,
    47,117,68,204,228,136,34,26,45,253,252,181,47,57,41,242,
    155,235,234,0,230,114,48,115,139,100,139,232,90,37,125,26,
    165,162,57,148,231,42,163,8,62,13,85,183,43,85,19,231,
    95,208,94,109,214,179,41,214,17,73,109,193,12,185,58,112,
    115,209,197,95,241,249,148,116,79,66,75,160,50,218,233,107,
    118,89,18,146,201,249,241,5,180,188,91,57,156,30,18,254,
    125,137,146,230,4,37,244,152,37,238,255,2,92,141,10,248,
    51,16,14,208,220,5,238,43,55,33,195,47,211,244,63,0,
    59,200,53,85,0,199,154,62,101,126,158,129,33,40,127,194,
    83,117,81,240,5,252,181,230,93,101,234,54,139,122,179,158,
    186,173,42,78,49,128,190,83,122,182,46,6,52,178,205,161,
    151,211,52,29,165,38,14,59,201,4,85,157,136,81,250,29,
    163,105,86,239,226,18,67,223,78,176,68,201,239,190,88,54,
    106,8,249,41,53,31,84,224,16,229,216,187,227,109,19,110,
    78,212,174,206,5,223,16,3,22,179,188,56,195,174,49,33,
    81,97,191,81,98,255,131,10,251,146,147,214,91,62,127,80,
    107,144,181,207,13,129,135,69,172,216,232,108,102,129,108,192,
    94,147,188,132,203,107,81,56,145,40,131,24,133,188,11,25,
    145,85,178,171,149,85,25,92,219,146,154,215,211,8,14,100,
    206,157,200,139,247,3,239,105,66,59,209,118,126,233,86,70,
    201,123,167,206,59,185,132,184,137,125,126,221,46,101,120,53,
    141,192,240,17,18,174,120,103,55,8,82,159,163,193,243,67,
    105,199,50,222,199,67,230,97,56,178,15,34,111,192,118,49,
    11,217,190,44,101,83,108,216,203,21,70,254,152,218,212,246,
    211,4,227,245,177,175,210,204,14,36,30,189,100,96,191,111,
    115,176,183,195,220,246,246,241,171,231,43,13,241,139,78,202,
    133,172,151,13,114,174,89,143,78,168,59,45,187,186,120,170,
    14,177,120,31,65,149,90,245,121,175,138,221,92,150,107,143,
    193,93,241,80,165,198,58,90,81,181,225,108,81,243,35,152,
    98,136,255,16,9,199,180,3,41,171,41,238,27,45,67,117,
    46,248,232,87,180,42,191,234,169,191,253,46,158,170,47,116,
    10,127,109,150,55,65,51,32,249,48,70,151,53,205,226,178,
    6,157,120,230,191,117,98,246,130,105,225,255,228,127,234,187,
    206,147,255,7,235,206,199,80,100,241,155,252,86,212,229,90,
    208,126,59,20,229,225,161,46,20,95,82,172,95,3,29,215,
    207,164,167,164,182,208,198,116,196,100,223,215,251,190,153,248,
    226,213,50,247,89,37,209,57,87,47,227,21,54,156,62,53,
    177,225,196,139,228,30,214,187,22,215,187,59,84,239,158,178,
    248,174,161,75,222,9,32,27,149,22,86,40,60,202,19,247,
    178,38,116,77,75,172,121,163,145,76,2,231,49,212,203,84,
    254,60,13,12,80,172,57,131,90,37,97,138,21,172,75,175,
    250,30,133,210,154,132,108,193,70,229,109,83,178,37,67,246,
    109,9,217,46,157,22,38,241,212,217,161,134,35,86,21,60,
    157,95,64,25,80,43,80,6,50,146,74,94,177,8,199,186,
    226,244,26,72,76,42,233,24,15,19,92,157,227,123,228,186,
    83,11,196,63,71,194,175,160,56,13,97,32,22,77,163,101,
    182,154,45,193,153,237,210,69,173,102,136,14,201,186,6,29,
    231,14,187,243,98,37,51,223,39,150,9,134,116,196,231,165,
    93,47,214,87,65,124,207,225,252,0,138,51,169,243,94,165,
    64,58,172,115,225,175,15,88,8,99,206,186,156,100,157,159,
    209,56,21,193,241,246,86,41,205,150,150,230,83,47,151,125,
    169,158,229,121,234,243,165,102,188,173,238,221,56,243,185,135,
    249,121,243,218,207,159,69,169,127,36,3,125,189,166,30,220,
    60,231,87,105,236,225,248,253,107,103,244,195,184,160,176,116,
    233,123,144,209,170,213,75,163,185,204,66,47,10,223,232,27,
    185,114,152,47,66,175,145,143,108,84,31,224,52,122,41,248,
    49,168,50,57,8,115,164,197,132,234,43,138,128,64,86,103,
    25,175,4,205,250,234,105,129,80,215,166,250,16,253,148,46,
    108,114,186,35,160,59,182,214,98,11,1,73,129,194,196,227,
    235,130,176,204,249,78,203,154,159,107,89,173,25,147,47,70,
    22,240,40,210,182,90,115,243,162,254,111,19,1,220,54,54,
    113,245,127,0,115,218,219,86,
};

EmbeddedPython embedded_m5_internal_param_RandomRepl(
    "m5/internal/param_RandomRepl.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_RandomRepl.py",
    "m5.internal.param_RandomRepl",
    data_m5_internal_param_RandomRepl,
    2216,
    6711);

} // anonymous namespace
