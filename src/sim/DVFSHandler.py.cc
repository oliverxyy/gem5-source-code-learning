#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_DVFSHandler[] = {
    120,156,173,82,77,111,211,64,16,29,59,137,155,164,161,173,
    194,141,147,79,96,33,17,151,67,47,8,85,168,45,21,7,
    4,40,65,149,200,197,218,218,235,120,147,93,111,180,187,13,
    241,185,252,111,152,89,187,144,31,128,99,79,102,222,206,199,
    155,153,205,161,123,122,248,125,136,1,236,37,42,5,190,1,
    72,128,239,164,133,32,3,80,33,44,67,8,186,147,30,158,
    244,128,135,80,6,80,244,225,23,192,35,192,143,101,31,138,
    1,44,146,8,83,136,223,248,36,1,106,142,196,235,86,29,
    161,88,8,245,245,126,205,115,231,142,209,186,185,187,93,124,
    98,117,33,185,201,15,185,92,17,151,12,21,14,176,12,136,
    17,150,199,210,75,44,219,135,245,0,86,158,217,35,210,138,
    128,31,17,194,135,176,30,1,242,34,112,236,193,99,224,19,
    34,72,200,51,143,156,16,197,34,242,200,41,204,23,201,17,
    214,152,135,40,236,115,18,66,165,197,174,180,89,213,114,154,
    85,149,61,69,92,10,235,98,93,198,133,86,76,212,214,166,
    136,93,203,77,103,199,248,254,172,68,94,197,174,226,113,23,
    26,11,139,184,117,172,118,130,57,94,216,23,24,243,177,102,
    247,146,167,55,194,210,255,161,187,27,224,241,219,243,243,7,
    107,95,162,86,138,61,47,98,137,129,117,222,196,165,54,241,
    150,155,50,150,124,199,101,172,196,202,48,39,116,157,156,209,
    84,135,40,178,172,102,138,103,153,27,123,67,233,226,65,146,
    217,39,135,102,203,61,158,239,247,89,197,89,129,213,104,246,
    119,184,4,109,190,49,195,148,59,161,205,152,252,90,234,124,
    115,227,155,114,52,153,174,95,79,174,117,140,90,141,215,174,
    77,41,55,89,235,228,83,216,198,102,7,16,85,191,210,90,
    250,248,91,38,45,247,241,220,79,193,23,248,220,118,232,166,
    196,211,176,218,10,234,43,235,26,79,232,58,252,19,246,29,
    138,180,210,138,167,90,138,29,55,251,166,73,183,70,227,52,
    84,186,226,234,226,13,142,155,230,107,77,158,210,42,15,110,
    215,108,219,248,53,211,230,44,141,45,10,240,215,159,246,38,
    193,40,156,14,71,193,151,100,240,116,69,213,197,108,75,205,
    90,55,105,173,191,119,118,30,60,13,156,124,140,222,183,73,
    255,47,79,159,255,125,187,194,203,87,148,143,134,54,14,206,
    130,113,244,7,37,128,228,117,
};

EmbeddedPython embedded_m5_objects_DVFSHandler(
    "m5/objects/DVFSHandler.py",
    "/home/oliverxyy/program/gem5-stable/src/sim/DVFSHandler.py",
    "m5.objects.DVFSHandler",
    data_m5_objects_DVFSHandler,
    536,
    945);

} // anonymous namespace
