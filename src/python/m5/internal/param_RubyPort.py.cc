#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_RubyPort[] = {
    120,156,189,89,109,115,219,198,17,222,3,72,74,164,36,139,
    178,44,201,182,100,139,126,145,205,184,177,216,166,113,236,153,
    184,158,184,105,58,211,204,68,113,161,116,236,40,153,162,16,
    113,162,64,145,0,11,156,108,51,35,125,169,50,109,63,244,
    91,167,63,161,31,250,111,242,11,250,87,218,221,61,28,9,
    190,89,142,91,201,34,207,135,197,222,222,190,60,183,183,119,
    172,67,250,47,143,223,79,42,0,201,191,5,128,143,31,1,
    45,128,182,128,29,1,66,10,240,47,193,65,30,226,15,193,
    207,195,247,0,59,22,72,11,78,176,99,195,55,22,132,179,
    60,166,0,45,155,41,2,186,37,144,57,216,201,195,243,112,
    1,114,178,0,7,37,136,255,0,66,136,80,192,11,127,10,
    252,105,248,30,165,99,167,200,2,167,129,136,37,38,22,193,
    159,97,98,9,252,89,238,204,64,183,12,114,22,118,230,136,
    109,231,2,138,189,135,98,231,89,236,15,36,214,199,55,75,
    224,95,32,118,212,235,107,226,204,17,39,207,55,207,82,202,
    70,203,5,216,185,104,250,139,153,254,165,76,127,41,211,95,
    206,244,87,50,253,203,153,254,149,76,255,106,166,191,154,233,
    175,101,250,215,50,253,235,220,71,11,47,66,115,29,154,21,
    104,222,128,61,116,250,66,207,154,155,32,109,104,222,130,157,
    91,32,241,115,19,78,48,46,254,197,204,136,219,60,98,177,
    55,98,131,71,220,129,157,59,32,241,179,161,71,20,96,187,
    186,140,177,14,254,131,255,170,24,107,80,179,216,188,148,113,
    18,68,161,27,132,123,81,96,209,251,2,53,132,140,58,53,
    83,41,68,62,37,136,252,11,24,31,190,149,66,228,24,80,
    176,32,91,90,22,28,115,231,216,130,110,21,142,4,52,115,
    224,219,112,132,211,228,73,129,134,128,19,11,190,181,137,225,
    24,219,28,6,242,58,228,148,198,71,147,3,169,37,77,193,
    113,30,142,242,176,253,226,200,34,194,65,17,226,127,194,119,
    107,44,116,154,133,90,112,132,109,14,78,114,112,92,128,231,
    200,132,164,102,145,204,23,47,142,208,82,164,108,87,115,168,
    237,86,198,92,50,197,15,226,208,107,75,53,143,125,183,227,
    197,94,219,117,14,119,187,207,162,88,85,75,134,41,74,54,
    59,158,218,119,120,148,77,238,104,119,20,75,139,66,169,102,
    176,179,23,132,190,219,142,252,195,150,84,211,36,202,221,11,
    90,210,117,249,229,111,218,29,20,247,89,28,71,177,67,30,
    101,98,43,242,122,35,200,159,245,86,148,200,42,205,198,211,
    56,36,94,17,247,94,135,37,146,2,172,41,13,246,101,82,
    143,131,142,194,64,105,137,196,77,210,170,20,34,110,18,7,
    155,218,126,212,150,181,168,21,96,84,95,119,187,181,78,28,
    53,208,196,90,67,182,31,220,79,148,183,219,146,181,221,195,
    160,229,215,94,60,250,168,214,233,170,253,40,172,181,31,212,
    130,80,73,244,75,171,54,232,145,77,228,184,72,178,95,5,
    13,55,96,171,220,125,217,234,200,120,142,168,87,105,94,81,
    22,179,162,32,108,81,21,115,216,203,227,215,22,107,214,140,
    216,10,200,174,58,217,74,136,202,101,49,68,129,21,112,96,
    65,188,70,8,105,226,71,80,72,17,39,219,244,206,226,119,
    191,37,135,104,106,211,166,184,107,226,17,163,10,225,133,156,
    143,41,208,33,48,52,242,208,44,128,134,12,34,77,99,40,
    238,82,139,236,36,198,66,225,57,72,254,1,232,96,4,203,
    17,164,64,58,177,65,132,101,80,37,202,27,72,93,198,9,
    255,196,88,220,174,146,250,91,12,10,181,31,36,209,171,144,
    93,79,125,94,61,219,232,153,103,221,47,119,155,178,174,146,
    117,36,124,29,29,86,234,94,24,70,170,226,249,126,197,83,
    42,14,118,15,149,76,42,42,170,108,36,85,138,166,179,96,
    112,213,147,215,237,24,28,81,204,17,71,250,193,15,234,10,
    31,22,249,129,163,144,72,133,152,216,143,252,4,233,36,162,
    33,149,67,74,42,114,114,196,138,48,100,92,98,165,233,145,
    239,2,62,63,53,154,48,46,171,5,131,162,68,182,246,84,
    137,1,233,37,137,203,154,16,157,177,71,130,95,122,173,67,
    201,210,17,64,10,21,162,174,214,225,236,209,119,153,44,49,
    134,179,53,97,20,250,93,84,46,168,223,165,121,47,51,6,
    103,25,133,75,136,192,41,108,11,248,127,65,44,91,245,92,
    138,187,130,193,30,229,63,5,28,121,145,6,31,113,120,130,
    185,166,106,113,178,96,131,120,61,222,164,30,13,118,214,168,
    185,70,205,117,106,214,141,205,103,106,248,220,176,225,15,105,
    50,139,173,101,187,40,52,182,177,203,31,88,83,87,250,107,
    10,19,226,54,173,13,139,86,80,127,109,228,40,121,198,79,
    168,69,86,94,117,54,36,95,81,170,166,53,196,194,104,185,
    32,240,169,215,95,14,236,37,167,76,214,79,27,36,59,4,
    207,44,70,27,25,140,58,20,32,6,168,115,197,164,66,151,
    56,52,52,157,85,18,149,31,227,230,10,53,55,206,197,215,
    125,144,53,70,64,246,49,205,91,78,65,54,199,224,42,225,
    183,108,213,237,52,0,189,205,113,113,8,92,132,172,220,24,
    100,221,161,158,61,106,242,121,130,42,53,244,215,25,80,145,
    110,86,214,158,45,236,116,87,200,140,44,156,86,112,155,127,
    30,174,224,206,109,241,206,253,83,222,185,121,247,231,122,75,
    39,103,155,243,179,238,228,201,31,123,54,44,167,59,114,82,
    196,22,173,121,221,173,68,123,21,197,6,83,46,125,188,145,
    108,110,36,31,99,150,172,60,225,252,164,243,164,206,132,177,
    236,80,38,163,161,159,189,174,75,222,10,249,201,117,117,226,
    114,57,137,185,233,22,139,200,90,34,111,90,198,205,156,194,
    19,21,83,230,62,123,71,151,122,142,38,189,63,167,153,74,
    236,101,91,172,32,138,74,130,213,113,117,186,230,50,139,223,
    226,247,151,228,121,50,89,2,21,224,206,182,86,150,237,32,
    139,156,247,7,144,114,150,86,56,53,20,251,59,131,144,66,
    31,33,244,181,13,226,255,2,92,125,10,248,51,16,6,48,
    212,41,226,123,11,132,130,190,72,236,191,7,94,26,99,118,
    126,206,49,219,180,219,51,7,166,158,228,33,179,234,66,224,
    115,248,107,102,93,153,237,218,78,235,203,236,118,157,235,229,
    39,6,207,91,109,201,185,193,68,70,145,217,247,18,98,211,
    217,169,191,84,251,249,191,87,25,98,118,62,83,36,77,235,
    57,92,82,231,219,62,142,104,195,91,21,139,86,6,29,63,
    163,230,131,30,48,132,161,157,149,102,235,48,121,107,118,117,
    254,255,134,166,207,177,194,243,83,108,138,17,208,195,124,222,
    96,254,131,30,230,37,111,83,223,243,73,131,90,139,226,124,
    98,9,60,94,98,125,70,167,185,28,200,60,236,20,104,117,
    112,41,45,210,197,35,76,234,162,68,55,176,7,178,59,182,
    180,163,122,161,214,81,164,230,245,217,167,4,178,254,113,203,
    107,239,250,222,19,202,121,9,77,86,55,203,201,50,154,151,
    179,154,211,82,16,147,148,231,199,7,198,130,151,103,159,14,
    62,2,222,223,180,230,12,126,63,170,115,14,248,106,95,86,
    218,178,189,139,71,201,253,160,83,217,107,121,13,142,137,157,
    90,246,165,177,76,113,80,135,235,137,228,30,181,81,165,30,
    133,152,161,15,235,42,138,43,190,196,35,150,244,43,247,43,
    156,222,43,65,82,241,118,241,173,87,87,26,218,131,75,147,
    75,86,47,110,36,92,157,30,188,162,238,249,196,212,197,147,
    115,128,69,250,75,232,109,165,250,84,215,203,214,92,126,235,
    149,130,115,226,225,73,117,117,134,162,218,194,217,164,230,61,
    56,183,164,254,97,26,193,132,28,85,16,171,86,209,98,5,
    13,207,51,26,145,140,174,207,79,196,91,172,79,125,241,131,
    12,178,0,205,41,110,167,41,155,239,20,13,177,196,237,12,
    19,103,13,113,142,219,11,76,156,55,196,50,183,11,76,188,
    104,136,139,220,94,98,226,146,33,46,115,187,194,196,203,134,
    120,133,219,171,76,92,53,196,53,110,175,49,241,186,33,174,
    115,91,97,226,13,67,188,201,237,45,38,222,54,196,13,110,
    239,48,241,174,33,86,185,125,143,137,247,76,134,250,9,19,
    223,135,157,251,230,218,108,147,178,85,225,127,205,86,188,224,
    207,103,169,31,255,95,147,148,243,240,252,21,119,30,65,90,
    164,76,74,80,3,245,238,83,157,160,116,12,176,148,233,94,
    98,35,245,209,137,141,20,207,195,171,88,248,230,184,240,125,
    76,133,239,17,23,199,174,165,107,223,126,232,248,68,195,55,
    53,116,127,18,202,87,238,224,250,210,165,45,193,193,235,116,
    100,232,59,247,32,91,173,242,235,179,247,22,37,160,191,65,
    166,168,176,197,37,44,79,71,17,74,217,53,99,29,35,49,
    223,195,228,218,185,133,246,239,38,180,213,218,64,138,117,30,
    83,83,30,200,167,218,253,215,71,82,155,27,227,163,155,116,
    19,37,219,116,14,58,141,5,171,27,62,181,102,104,234,246,
    232,144,228,176,195,247,99,190,167,60,172,250,255,152,176,236,
    183,98,164,25,184,90,29,126,243,166,225,1,238,149,111,53,
    79,159,113,120,158,222,27,117,101,204,240,190,131,38,191,37,
    137,124,51,52,209,45,135,73,16,54,180,63,177,226,198,224,
    78,82,119,148,209,168,59,242,70,93,29,29,110,174,177,73,
    250,27,94,147,76,218,157,211,103,181,57,202,202,174,73,90,
    222,75,233,98,65,18,226,1,141,6,214,163,195,80,177,244,
    31,57,132,102,164,90,229,13,60,234,225,4,145,157,32,74,
    135,240,227,88,117,222,117,108,146,134,225,109,152,213,163,9,
    147,96,78,117,219,30,71,107,178,134,239,60,152,84,220,48,
    42,158,198,173,106,147,166,209,131,198,170,246,99,199,144,70,
    107,61,141,198,51,77,180,151,156,252,206,206,58,117,240,128,
    179,78,227,158,136,26,242,242,187,34,238,180,177,3,136,59,
    133,153,183,38,69,249,221,151,45,169,228,208,246,169,40,213,
    167,183,141,62,230,132,56,234,186,174,190,83,193,231,150,235,
    158,83,49,253,11,93,8,65,66,183,197,88,76,139,130,88,
    26,255,103,21,11,69,193,231,149,161,31,217,180,146,116,151,
    173,239,19,186,172,43,56,243,189,45,140,127,13,50,7,7,
    218,232,248,222,107,203,107,235,171,124,190,169,118,110,65,122,
    179,232,220,237,237,130,156,183,107,102,128,139,85,8,159,165,
    248,232,228,252,156,232,132,229,246,131,77,99,225,102,223,194,
    109,157,209,45,102,224,45,114,148,111,59,104,119,90,242,11,
    217,142,226,174,170,140,101,121,154,158,217,82,166,213,177,76,
    248,82,255,130,194,119,10,163,239,63,109,69,245,3,233,167,
    60,215,38,243,252,42,106,123,72,31,63,11,106,155,74,88,
    24,122,239,199,52,106,105,136,154,200,56,240,90,193,119,146,
    239,135,199,200,211,30,26,158,76,134,135,108,17,154,251,69,
    228,203,17,23,63,245,253,216,241,194,134,196,61,137,78,186,
    234,198,48,195,128,203,12,23,129,192,176,240,143,151,195,174,
    163,16,247,158,248,148,199,22,25,180,242,90,137,101,35,224,
    13,116,62,203,156,86,164,132,101,182,102,120,171,207,12,60,
    159,101,165,239,74,244,69,238,147,25,51,15,253,194,83,156,
    47,138,130,69,117,170,45,74,88,169,230,236,217,114,49,55,
    59,83,204,21,167,108,190,154,159,19,139,86,41,87,156,153,
    21,147,254,214,113,33,150,172,245,203,69,241,95,65,94,56,
    0,
};

EmbeddedPython embedded_m5_internal_param_RubyPort(
    "m5/internal/param_RubyPort.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_RubyPort.py",
    "m5.internal.param_RubyPort",
    data_m5_internal_param_RubyPort,
    2593,
    8451);

} // anonymous namespace