#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_I8259[] = {
    120,156,173,84,221,107,219,64,12,215,57,137,211,164,237,90,
    54,40,108,48,48,219,139,25,91,205,6,13,89,25,99,107,
    55,216,160,45,193,233,96,245,139,113,237,107,125,157,191,240,
    93,178,4,250,214,253,223,155,36,39,253,96,175,113,98,161,
    211,233,36,253,244,211,57,134,197,211,194,247,147,3,160,175,
    80,73,240,47,32,3,56,93,104,162,209,44,200,44,200,91,
    16,180,64,208,186,5,89,27,242,14,4,29,200,109,8,108,
    180,182,65,118,225,66,64,210,129,63,0,55,0,103,193,26,
    36,54,200,22,91,187,183,214,30,36,107,48,118,123,152,78,
    253,197,199,21,168,25,18,175,26,245,17,138,131,72,171,120,
    164,202,47,114,170,98,233,90,100,223,66,241,115,56,248,94,
    152,113,57,169,99,57,82,133,217,188,179,169,226,23,89,158,
    44,44,195,119,123,239,15,35,29,71,137,60,46,19,25,223,
    199,123,64,120,95,162,34,1,2,1,170,181,0,62,64,160,
    136,109,128,96,16,198,32,176,192,31,187,54,21,74,39,205,
    58,10,142,123,28,105,35,107,197,213,246,151,198,113,22,77,
    165,178,30,56,98,81,151,153,116,41,167,89,67,17,134,69,
    148,203,48,228,83,97,152,151,201,36,163,37,57,228,81,229,
    82,154,59,161,135,40,188,180,204,165,87,102,106,42,235,217,
    124,238,85,117,121,89,71,185,119,41,243,189,55,218,68,231,
    153,244,116,29,123,137,156,122,179,225,192,227,188,187,213,220,
    111,227,225,23,20,133,0,216,162,47,186,194,116,150,133,253,
    223,142,179,219,118,96,47,16,57,182,3,217,198,134,4,200,
    108,7,174,144,73,155,8,196,254,220,160,177,203,70,100,184,
    13,55,232,217,227,101,31,228,58,117,142,28,54,136,124,228,
    60,216,164,38,118,49,186,79,109,209,75,198,198,159,247,247,
    185,20,189,77,131,183,168,94,113,245,105,170,159,162,241,52,
    149,78,165,10,199,164,74,59,236,235,36,53,246,65,235,103,
    184,251,173,252,125,127,7,149,184,161,59,209,52,3,76,71,
    179,247,218,81,23,78,84,204,99,34,135,94,106,195,33,129,
    38,71,195,212,95,243,148,95,11,132,104,141,153,176,19,99,
    55,133,225,0,178,90,76,242,115,89,243,140,250,228,208,76,
    37,181,89,203,236,130,149,76,21,114,69,252,241,76,32,250,
    183,20,136,50,129,112,233,102,248,52,71,126,111,153,218,204,
    43,105,104,21,207,102,97,156,69,90,243,108,209,42,149,216,
    140,154,57,31,69,152,210,231,122,9,73,57,49,213,196,240,
    132,112,16,156,67,201,236,240,234,228,199,209,17,159,210,212,
    66,127,251,193,76,174,96,48,215,23,97,152,120,91,52,191,
    13,209,19,91,226,177,117,226,82,249,140,40,223,219,173,168,
    110,205,87,135,86,117,57,155,51,128,230,179,224,139,165,107,
    243,9,192,235,207,24,153,29,134,242,21,57,99,152,183,73,
    87,133,130,107,250,208,92,224,143,207,41,26,213,221,199,107,
    182,45,118,172,29,251,31,88,161,21,251,
};

EmbeddedPython embedded_m5_objects_I8259(
    "m5/objects/I8259.py",
    "/home/oliverxyy/program/gem5-stable/src/dev/x86/I8259.py",
    "m5.objects.I8259",
    data_m5_objects_I8259,
    683,
    1376);

} // anonymous namespace
