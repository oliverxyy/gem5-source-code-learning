#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_RubyWireBuffer[] = {
    120,156,197,88,123,111,227,198,17,159,37,41,218,146,229,179,
    125,126,221,157,221,88,105,113,141,122,104,172,54,141,115,1,
    226,186,77,138,20,104,128,58,41,149,194,142,18,148,165,197,
    149,68,153,34,5,114,125,119,10,100,160,168,15,109,191,64,
    63,66,255,232,183,233,55,106,103,102,73,138,126,1,1,146,
    168,62,113,111,185,220,157,157,199,111,30,187,93,200,254,42,
    248,252,186,1,144,246,4,128,143,63,1,33,192,72,64,71,
    128,144,2,252,13,56,175,64,242,46,248,21,120,13,208,49,
    64,26,112,133,29,19,190,52,32,170,243,26,27,66,147,71,
    4,76,106,32,45,232,84,224,36,90,3,75,218,112,94,131,
    228,207,32,132,136,4,156,250,11,224,47,194,107,164,142,157,
    42,19,92,4,26,172,241,96,21,252,37,30,172,129,95,231,
    206,18,76,86,65,214,161,179,76,211,58,15,144,236,51,36,
    187,194,100,255,67,100,125,252,178,9,254,3,154,142,124,125,
    65,51,45,154,201,251,173,48,149,213,156,203,53,232,60,204,
    251,235,165,254,6,247,113,167,135,48,220,132,225,22,12,183,
    1,21,226,175,21,84,31,129,52,97,248,24,58,143,65,226,
    239,17,92,161,126,252,135,165,21,79,120,197,122,177,98,135,
    87,236,66,103,23,36,254,118,244,10,27,218,205,45,212,121,
    240,95,252,107,162,206,65,213,177,121,33,147,52,136,35,55,
    136,122,113,96,208,119,155,26,178,80,151,154,133,204,84,191,
    33,83,253,27,216,78,190,145,153,234,18,144,176,32,89,66,
    3,46,185,115,105,192,164,9,83,1,67,11,124,19,166,184,
    77,133,24,232,11,184,50,224,43,147,38,92,98,107,161,66,
    223,0,75,105,59,13,89,161,154,210,2,92,86,96,90,129,
    246,233,212,160,129,243,42,36,255,130,175,119,153,232,34,19,
    53,96,138,173,5,87,22,92,218,112,130,147,112,104,88,37,
    241,197,233,20,37,197,145,118,211,66,110,143,75,226,146,40,
    126,144,68,222,72,170,77,236,187,99,47,241,70,174,115,113,
    54,57,9,18,249,209,69,175,39,147,102,45,159,26,167,251,
    99,79,13,28,94,107,146,82,70,99,197,52,227,72,170,37,
    236,244,130,200,119,71,177,127,17,74,181,72,4,221,94,16,
    74,215,229,143,191,27,141,227,68,125,156,36,113,226,144,94,
    121,48,140,189,98,5,105,181,27,198,169,108,210,110,188,141,
    67,228,21,205,238,141,153,34,49,192,252,210,98,95,166,221,
    36,24,43,52,151,166,72,179,137,90,147,12,197,77,122,138,
    77,107,16,143,100,43,14,3,180,237,171,201,164,53,78,226,
    62,10,218,234,203,209,193,219,169,242,206,66,217,58,187,8,
    66,191,117,250,254,123,173,241,68,13,226,168,53,58,104,5,
    145,146,168,157,176,117,151,94,246,113,222,67,218,225,101,208,
    119,3,150,205,29,200,112,44,147,101,26,125,66,187,139,85,
    81,23,182,48,69,83,44,99,175,130,143,41,118,141,37,113,
    28,144,116,93,146,152,208,101,149,241,68,70,22,112,110,64,
    178,75,104,25,226,79,144,121,17,51,109,250,102,240,183,63,
    144,90,244,232,208,36,12,232,193,41,35,12,161,134,51,15,
    201,232,17,48,76,42,48,180,65,195,7,81,167,241,148,76,
    168,197,233,68,198,64,226,22,164,255,4,84,51,2,103,10,
    25,168,174,76,16,209,42,168,26,249,50,142,110,225,134,127,
    101,92,182,155,196,254,49,67,67,13,130,52,126,25,177,1,
    168,207,158,212,70,205,124,54,249,244,108,40,187,42,221,195,
    129,47,226,139,70,215,139,162,88,53,60,223,111,120,74,37,
    193,217,133,146,105,67,197,141,167,105,147,108,234,172,229,232,
    42,232,77,198,57,154,200,242,136,38,253,226,7,93,133,47,
    235,252,194,86,72,165,66,100,12,98,63,197,113,34,209,151,
    202,33,38,21,41,57,102,70,24,56,46,77,165,237,113,222,
    3,124,255,48,231,132,209,217,180,115,44,165,50,236,169,26,
    195,210,75,83,151,57,161,113,70,32,17,126,225,133,23,146,
    169,35,140,20,50,68,93,205,195,188,48,248,136,228,201,197,
    103,153,162,56,242,39,200,98,208,125,139,118,127,196,72,172,
    51,22,55,17,135,11,216,218,248,191,45,182,140,174,149,161,
    207,206,17,72,17,81,1,219,95,100,16,64,52,94,97,244,
    105,26,28,62,88,44,246,205,31,82,143,22,59,187,212,252,
    128,154,55,168,217,203,37,159,131,248,203,55,197,127,78,91,
    26,44,51,75,71,102,50,115,233,252,107,254,245,120,230,95,
    24,40,219,228,39,6,121,211,204,79,44,10,170,201,17,181,
    56,149,61,208,132,244,115,10,225,228,79,76,140,92,7,157,
    128,122,51,215,96,93,57,171,164,131,197,28,213,14,65,181,
    140,215,126,9,175,14,153,137,193,234,60,206,131,163,75,51,
    52,76,157,29,34,85,185,67,217,13,106,222,156,163,198,103,
    128,235,223,2,220,7,180,251,106,6,184,101,6,90,13,159,
    85,163,107,102,102,40,82,231,250,13,160,17,202,172,59,80,
    246,99,234,153,183,5,159,63,192,50,113,127,91,2,24,113,
    104,148,165,58,198,206,100,155,132,41,67,107,27,75,129,147,
    104,27,179,187,193,217,253,103,156,221,185,66,224,218,72,7,
    109,147,227,182,238,84,72,43,61,19,182,178,172,157,86,177,
    69,153,94,77,26,113,175,161,88,108,138,177,135,79,211,253,
    167,233,7,24,61,27,71,28,183,116,252,212,17,50,145,99,
    138,112,180,244,227,87,93,201,137,146,223,92,87,7,52,151,
    131,155,155,37,96,68,25,85,1,108,1,86,54,135,246,84,
    37,20,209,231,165,238,90,161,110,226,254,19,218,175,198,186,
    54,197,54,34,170,38,152,41,87,7,115,46,200,248,43,62,
    31,145,254,73,112,9,84,50,59,109,205,50,75,67,114,57,
    63,189,134,154,239,95,22,167,133,196,255,152,163,197,158,161,
    133,30,51,247,129,191,3,87,171,2,254,6,132,7,52,123,
    230,3,133,203,16,0,214,105,250,159,128,157,229,142,234,128,
    99,79,155,42,2,158,129,33,41,125,206,83,117,177,240,9,
    252,163,228,105,121,74,55,179,122,180,156,210,173,34,110,49,
    144,190,81,218,182,174,7,56,178,207,192,75,105,154,142,90,
    51,231,157,101,135,162,134,196,168,61,7,84,45,234,157,92,
    98,234,171,25,166,40,41,238,136,117,163,132,148,159,83,243,
    78,1,18,145,143,125,191,252,237,193,253,73,220,213,57,226,
    75,98,194,98,182,87,22,216,77,102,36,10,63,168,228,126,
    240,78,225,7,146,147,217,107,62,167,80,107,144,213,175,12,
    129,135,68,172,232,232,76,102,129,172,64,199,38,143,225,18,
    92,100,14,37,242,160,70,33,240,90,166,100,181,28,107,133,
    21,134,215,54,165,230,213,188,130,5,153,245,48,244,70,103,
    190,119,52,164,221,104,203,110,238,98,70,206,255,106,153,127,
    114,15,113,159,8,252,122,144,203,241,98,94,129,226,61,96,
    238,52,255,236,22,126,220,229,232,240,249,64,54,70,114,116,
    134,135,210,65,48,110,244,66,175,207,246,49,51,249,62,205,
    229,83,108,224,155,21,72,250,140,218,184,209,141,35,140,227,
    23,93,21,39,13,95,226,49,77,250,141,183,27,156,4,26,
    65,218,240,206,240,171,215,85,26,238,215,157,150,11,94,47,
    233,167,92,219,158,191,164,238,60,237,235,226,73,60,192,66,
    63,132,34,237,234,243,97,17,211,185,132,215,222,131,59,227,
    1,76,77,116,4,163,106,196,217,167,230,39,48,231,208,255,
    46,232,59,131,148,148,102,139,29,163,106,168,13,226,228,218,
    204,207,104,117,122,219,123,127,255,77,188,87,95,238,100,62,
    108,231,183,66,11,32,249,0,71,23,55,118,118,113,131,142,
    189,240,109,29,155,189,98,158,254,144,126,167,254,236,60,255,
    127,177,239,188,15,89,166,191,207,151,69,89,182,101,237,203,
    67,145,31,56,202,130,241,37,199,206,61,48,114,187,137,244,
    148,212,214,218,157,159,184,28,23,244,222,47,102,62,122,187,
    52,254,176,144,236,138,43,157,201,6,27,81,159,184,216,136,
    226,36,122,130,53,178,197,53,242,33,213,200,83,86,131,107,
    232,50,121,6,208,74,161,13,58,141,68,242,165,123,151,70,
    116,45,76,236,121,227,177,140,124,231,25,148,203,91,254,60,
    47,76,80,44,154,64,169,242,48,197,6,214,179,183,125,146,
    194,109,73,82,182,102,165,240,194,57,218,149,97,252,151,28,
    198,77,58,109,204,98,174,115,72,13,71,217,34,192,58,191,
    130,60,232,22,64,245,101,40,149,188,211,58,138,86,103,39,
    97,95,98,2,138,39,120,32,225,234,30,223,67,215,157,107,
    192,254,37,18,79,32,59,85,97,192,22,182,81,53,171,118,
    85,112,38,188,113,17,172,153,162,163,168,174,97,39,169,195,
    174,190,82,200,206,119,149,121,50,34,93,241,185,235,216,27,
    233,43,38,190,59,113,126,4,217,249,214,121,171,80,36,29,
    254,249,224,160,15,106,8,109,206,210,156,148,157,95,228,106,
    29,29,236,231,18,237,107,137,218,193,72,223,180,241,109,233,
    232,64,173,221,152,230,39,30,246,55,111,140,166,50,9,188,
    48,248,90,223,176,229,195,138,4,185,73,151,248,41,222,56,
    193,169,109,40,87,161,108,202,68,246,131,20,169,48,137,98,
    122,230,143,164,100,245,230,125,241,171,188,124,158,182,215,101,
    164,62,3,31,209,189,75,122,132,13,93,151,85,87,170,136,
    3,242,85,19,79,158,203,194,50,235,171,85,171,190,84,181,
    170,11,38,223,109,44,227,233,161,102,85,151,234,130,254,237,
    33,94,106,198,30,174,250,31,204,184,204,65,
};

EmbeddedPython embedded_m5_internal_param_RubyWireBuffer(
    "m5/internal/param_RubyWireBuffer.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_RubyWireBuffer.py",
    "m5.internal.param_RubyWireBuffer",
    data_m5_internal_param_RubyWireBuffer,
    2172,
    6614);

} // anonymous namespace
