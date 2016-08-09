#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_AtomicSimpleCPU[] = {
    120,156,197,89,221,115,219,198,17,223,3,72,74,164,36,75,
    178,190,252,161,88,180,19,199,140,27,139,142,29,199,153,137,
    235,86,118,210,25,103,18,69,5,157,177,173,100,138,66,192,
    73,2,5,2,28,224,100,155,30,105,166,173,60,253,154,233,
    91,251,212,167,62,244,161,255,77,255,163,118,119,15,128,32,
    138,156,218,211,150,177,196,243,242,110,111,239,118,247,183,123,
    123,39,23,210,127,101,252,252,180,14,144,188,50,0,60,252,
    21,16,0,116,4,108,10,16,82,128,55,15,123,101,136,63,
    6,175,12,175,1,54,13,144,6,28,33,97,194,119,6,132,
    147,60,167,2,129,201,61,2,122,53,144,37,216,44,195,147,
    112,22,74,178,2,123,53,136,127,9,66,136,80,192,83,111,
    12,188,113,120,141,210,145,168,178,192,113,160,206,26,119,86,
    193,155,224,206,26,120,147,76,76,64,111,6,228,36,108,78,
    17,219,230,25,20,123,29,197,78,179,216,127,146,88,15,71,
    22,192,59,67,236,184,175,103,196,89,34,78,94,111,154,165,
    204,100,187,156,133,205,179,25,61,87,160,231,11,244,66,129,
    94,44,208,75,5,250,92,129,62,95,160,47,20,232,139,5,
    122,185,64,191,83,160,47,21,232,149,2,93,47,208,151,11,
    244,149,2,253,110,129,126,175,64,95,45,208,239,23,232,107,
    5,186,81,160,63,40,208,215,11,244,143,152,70,235,159,133,
    246,135,208,190,1,237,85,216,70,64,204,230,150,110,130,52,
    161,125,19,54,111,130,196,223,38,28,33,102,188,179,133,25,
    31,241,140,185,124,198,45,158,113,27,54,111,131,196,223,91,
    122,70,5,90,141,69,196,161,255,47,252,215,16,72,169,73,
    108,158,203,56,241,163,208,246,195,237,200,55,104,188,66,13,
    161,214,165,102,44,133,239,67,130,239,63,128,177,235,25,41,
    124,15,1,5,11,210,37,48,224,144,137,67,3,122,13,56,
    16,208,46,129,103,194,1,46,83,166,13,236,8,56,50,224,
    123,147,24,14,177,45,33,200,46,65,73,105,236,182,25,100,
    90,210,24,28,150,225,160,12,173,167,7,6,117,236,85,33,
    254,59,188,90,102,161,227,44,212,128,3,108,75,112,84,130,
    195,10,60,65,38,236,106,87,73,125,241,244,0,53,197,158,
    86,163,132,187,93,47,168,75,170,120,126,28,58,29,169,200,
    18,118,215,137,157,142,189,166,162,142,239,182,252,78,55,144,
    15,55,190,109,212,50,222,40,89,237,58,106,215,226,201,38,
    89,165,211,85,44,52,10,165,154,64,98,219,15,61,187,19,
    121,251,129,84,227,36,209,222,246,3,105,219,60,248,168,211,
    141,98,245,69,28,71,177,69,134,229,206,32,114,242,25,100,
    86,55,136,18,217,160,213,120,25,139,196,43,226,222,238,178,
    68,218,0,111,152,38,123,50,113,99,191,171,208,95,90,34,
    113,147,180,6,121,138,155,228,25,54,205,221,168,35,155,81,
    224,163,115,95,246,122,205,110,28,237,160,166,205,29,217,185,
    115,35,81,206,86,32,155,91,251,126,224,53,159,126,250,73,
    179,219,83,187,81,216,236,220,105,250,161,146,104,158,160,57,
    208,48,171,200,120,150,150,120,225,239,216,62,43,103,239,202,
    160,43,227,41,234,189,64,203,139,25,49,41,42,194,20,13,
    49,133,84,25,63,166,88,54,38,196,186,79,234,185,164,50,
    225,171,84,68,20,185,89,192,158,1,241,50,225,165,141,191,
    130,28,140,168,105,209,152,193,99,63,39,187,232,222,182,73,
    40,208,157,7,140,49,4,27,114,222,35,183,135,192,64,41,
    67,187,2,26,64,136,59,141,168,184,71,45,178,147,24,3,
    133,151,32,249,11,160,157,17,58,7,144,194,234,200,4,17,
    206,128,170,81,134,195,222,69,92,240,55,140,204,86,131,182,
    191,206,216,80,187,126,18,189,8,217,3,68,115,44,181,208,
    50,27,189,111,182,218,210,85,201,10,118,60,139,246,235,174,
    19,134,145,170,59,158,87,119,148,138,253,173,125,37,147,186,
    138,234,87,147,6,57,213,154,205,224,149,203,235,117,51,56,
    145,235,17,78,250,139,231,187,10,191,204,241,23,246,66,34,
    21,66,99,55,242,18,236,39,17,59,82,89,180,73,69,70,
    142,120,35,140,28,155,88,105,121,228,59,131,223,215,178,157,
    48,60,27,149,12,76,137,12,182,85,141,113,233,36,137,205,
    59,161,126,134,32,9,126,238,4,251,146,165,35,142,20,110,
    136,72,189,135,145,129,240,28,41,148,233,207,74,133,81,232,
    245,112,143,190,123,141,150,63,199,80,156,100,48,46,32,16,
    199,176,173,224,255,21,177,104,184,165,20,126,149,12,130,148,
    10,20,48,0,68,138,1,132,227,17,38,160,134,193,25,132,
    245,226,232,188,66,20,77,182,150,169,121,135,154,75,212,172,
    100,170,143,66,255,169,126,253,239,210,154,6,43,205,234,145,
    163,204,76,61,239,68,132,157,63,142,48,76,150,45,138,20,
    131,226,233,56,82,74,148,88,227,251,212,34,43,199,160,9,
    201,99,74,227,20,81,44,140,130,7,195,128,168,227,224,96,
    99,89,51,100,132,241,12,215,22,129,181,136,216,157,2,98,
    45,242,19,195,213,58,159,229,71,155,56,52,80,173,139,36,
    170,60,192,218,117,106,46,143,210,228,199,144,219,57,5,185,
    207,104,249,153,20,114,83,12,181,26,126,102,12,215,76,253,
    144,159,159,115,125,80,35,156,149,6,224,236,125,162,204,211,
    154,255,0,16,75,245,253,89,1,98,180,69,163,168,214,58,
    18,189,37,210,166,8,174,37,44,8,158,132,75,120,198,27,
    124,198,223,228,51,158,235,4,174,26,117,226,54,57,119,107,
    162,76,102,217,54,97,49,61,187,147,42,182,168,212,203,94,
    61,218,174,43,214,155,242,236,189,171,201,234,213,228,51,204,
    160,245,251,156,187,116,14,213,89,50,150,93,202,114,52,245,
    139,151,174,228,211,146,191,217,182,78,106,54,39,56,59,61,
    133,17,103,11,100,84,35,179,54,167,247,68,197,148,213,71,
    102,239,90,110,111,218,254,151,180,96,141,141,109,138,37,196,
    84,77,240,174,108,157,209,185,46,227,81,252,60,32,7,144,
    230,18,232,54,97,181,244,158,89,29,82,204,250,240,4,110,
    70,160,140,213,68,233,223,102,120,169,28,227,133,62,102,22,
    6,191,3,174,90,5,252,22,8,17,232,248,52,12,242,168,
    33,8,204,17,251,47,128,227,101,64,141,192,249,167,69,117,
    1,115,96,90,74,238,50,171,46,25,190,132,223,23,130,45,
    59,216,205,180,46,45,30,236,165,60,119,49,148,222,232,240,
    46,157,76,114,228,160,93,39,33,54,157,185,142,227,247,248,
    136,200,75,73,204,220,163,192,213,184,94,202,166,93,125,127,
    140,42,58,26,47,138,57,163,128,149,143,168,185,149,195,68,
    100,125,255,231,13,174,192,240,179,220,214,39,197,119,180,139,
    18,239,123,122,76,77,3,85,47,39,228,228,241,80,206,226,
    225,86,30,15,146,207,181,215,124,109,161,214,32,231,31,25,
    2,239,209,88,222,209,181,181,4,178,12,155,21,138,28,46,
    200,69,26,88,34,203,110,148,11,79,28,154,108,156,117,109,
    182,220,255,218,181,212,188,28,89,214,32,239,222,11,156,206,
    150,231,220,255,53,45,71,107,186,89,168,25,153,2,51,69,
    5,40,76,196,48,29,248,235,157,76,145,231,35,203,24,159,
    160,244,92,1,142,15,47,114,57,77,60,222,149,245,142,236,
    108,225,45,117,215,239,214,183,3,103,135,61,100,166,10,126,
    147,41,168,216,197,253,229,72,114,157,218,168,238,70,33,166,
    244,125,87,69,113,221,147,120,109,147,94,253,70,157,207,131,
    186,159,212,157,45,28,117,92,165,97,127,50,122,185,254,117,
    226,157,132,75,221,189,23,68,142,212,195,54,222,205,125,44,
    252,143,32,63,130,245,133,49,79,239,92,210,235,40,194,165,
    241,66,166,122,58,151,81,105,98,173,82,243,1,140,250,20,
    248,24,165,255,138,150,33,179,85,196,69,163,106,168,133,211,
    193,187,65,243,147,211,33,252,183,55,9,97,253,8,150,6,
    114,133,56,229,24,189,17,80,91,165,227,96,179,150,117,78,
    112,59,201,157,83,89,231,25,110,167,185,115,38,235,156,229,
    246,44,119,206,101,47,114,243,220,185,0,155,139,244,108,67,
    61,75,148,49,198,254,219,140,193,209,54,210,56,251,227,255,
    52,81,88,119,127,176,253,91,159,66,90,75,12,75,18,162,
    168,220,148,78,18,109,145,93,107,138,154,241,107,202,242,48,
    116,218,110,44,29,37,181,195,150,71,168,48,167,28,189,248,
    159,142,163,255,116,1,190,150,235,118,196,213,84,111,158,253,
    168,111,118,236,71,241,36,188,128,149,120,137,43,241,123,84,
    137,31,176,33,108,67,23,227,199,32,45,231,246,160,75,89,
    40,95,244,239,75,219,68,151,220,180,63,167,219,149,161,103,
    93,135,98,21,205,195,35,195,5,229,185,63,67,161,188,49,
    197,60,150,205,167,35,147,114,121,65,87,118,104,57,143,197,
    81,186,150,177,252,215,12,203,13,190,10,231,9,221,186,71,
    13,167,240,60,123,91,63,201,29,115,121,40,80,183,157,4,
    239,215,29,186,181,189,1,23,150,92,92,182,166,223,213,234,
    208,25,137,223,217,15,16,133,182,231,40,199,70,189,131,32,
    225,69,222,114,10,173,56,79,74,15,24,124,19,89,62,30,
    227,111,185,124,113,202,169,229,11,131,92,138,14,150,245,194,
    247,212,46,175,247,159,120,104,1,194,19,127,99,88,114,86,
    241,100,32,113,181,129,51,21,185,57,125,29,241,36,214,33,
    81,15,175,168,124,221,195,239,129,109,143,246,216,254,49,74,
    255,3,45,67,118,194,99,91,84,240,224,94,16,252,99,84,
    43,85,193,149,81,223,95,10,244,238,232,196,215,151,155,94,
    98,113,134,158,206,33,203,111,217,89,109,66,232,230,43,249,
    186,211,209,47,144,252,178,102,189,11,233,219,135,117,45,135,
    62,37,33,190,81,234,59,60,230,35,174,218,184,72,179,110,
    83,63,249,164,115,103,53,83,109,85,171,246,192,73,100,174,
    24,191,168,119,238,112,72,12,96,141,157,208,221,221,136,37,
    61,176,70,177,186,56,144,11,133,233,151,93,53,219,55,238,
    197,14,210,11,125,189,137,140,125,39,240,95,73,206,163,131,
    55,72,91,59,55,112,20,221,246,248,171,7,234,189,97,131,
    27,14,61,204,160,155,159,56,193,30,26,117,176,148,86,47,
    81,24,215,253,10,201,112,191,99,127,45,59,81,220,251,58,
    242,36,67,180,56,190,230,121,177,229,132,59,210,126,46,217,
    34,253,118,91,75,235,101,45,35,227,170,15,220,195,73,222,
    33,198,197,193,212,184,131,189,249,48,136,220,61,233,165,60,
    239,12,231,249,60,234,144,51,134,155,156,172,122,105,152,85,
    191,138,92,39,88,235,34,12,6,235,130,2,124,119,195,143,
    62,151,207,125,87,14,209,229,120,188,127,23,184,194,163,214,
    90,102,174,161,142,71,30,117,161,111,240,97,180,79,100,54,
    183,223,99,122,238,35,204,102,143,209,214,136,135,254,249,27,
    113,228,202,36,201,230,15,182,79,202,196,193,154,13,241,227,
    237,160,144,162,122,234,68,15,87,252,131,138,40,206,109,177,
    220,241,17,140,49,203,59,49,47,173,39,40,241,168,43,195,
    243,121,65,198,72,51,162,190,101,235,199,194,251,244,68,157,
    4,216,208,159,22,170,211,85,204,142,84,108,152,162,134,229,
    70,201,156,156,169,150,38,39,170,165,234,152,201,175,192,83,
    98,206,168,149,170,19,147,226,237,127,86,48,215,214,140,149,
    185,170,248,55,86,204,172,3,
};

EmbeddedPython embedded_m5_internal_param_AtomicSimpleCPU(
    "m5/internal/param_AtomicSimpleCPU.py",
    "/home/oliverxyy/program/gem5-stable/build/X86/python/m5/internal/param_AtomicSimpleCPU.py",
    "m5.internal.param_AtomicSimpleCPU",
    data_m5_internal_param_AtomicSimpleCPU,
    2600,
    8263);

} // anonymous namespace