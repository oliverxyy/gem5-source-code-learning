#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_VirtIO[] = {
    120,156,173,83,77,79,219,64,16,29,59,137,67,2,148,148,
    86,61,244,196,49,170,84,204,5,137,3,170,74,224,146,67,
    11,114,40,82,125,137,28,123,74,54,178,227,200,187,137,146,
    94,233,79,238,189,157,183,139,3,61,244,134,29,143,103,158,
    55,59,31,239,109,74,143,87,67,158,207,71,68,218,136,147,
    201,207,163,156,168,240,40,246,200,67,236,83,238,211,237,163,
    215,112,94,131,242,38,21,45,138,91,110,77,147,242,128,138,
    54,197,109,137,91,196,30,253,240,40,11,232,23,209,3,209,
    247,120,135,178,54,113,219,162,59,91,180,67,89,135,70,253,
    174,36,86,127,228,234,123,226,153,142,152,145,42,174,39,51,
    78,141,131,96,62,60,125,189,81,229,21,175,84,202,207,160,
    84,57,200,244,36,186,83,149,25,94,59,96,144,104,78,159,
    55,59,64,179,23,226,48,161,71,41,63,246,209,117,220,32,
    110,162,35,14,104,214,70,183,210,230,131,79,82,60,144,14,
    113,151,102,187,232,21,224,30,69,163,126,75,118,137,154,98,
    244,33,134,199,171,112,37,153,85,25,78,36,233,241,116,170,
    144,83,191,217,86,116,164,151,19,189,209,134,139,163,225,149,
    222,199,71,23,149,174,89,32,102,71,204,120,60,79,10,30,
    143,77,215,6,69,153,45,115,132,200,101,54,11,182,120,186,
    94,143,167,156,100,92,89,252,182,90,178,253,115,50,209,166,
    74,82,99,80,222,77,82,37,133,245,190,13,231,230,204,14,
    107,91,133,9,48,235,39,87,22,243,220,24,76,41,153,111,
    44,228,22,246,209,200,147,209,231,98,194,105,89,112,88,230,
    106,197,213,122,179,9,23,85,121,47,185,194,123,46,78,63,
    106,147,76,114,14,117,149,134,207,198,226,166,112,188,216,216,
    161,133,216,9,141,4,158,189,253,67,191,230,210,45,220,210,
    230,215,180,125,249,31,109,179,22,8,123,144,48,0,109,177,
    85,27,100,39,140,138,204,68,169,93,232,49,222,133,16,133,
    60,81,97,188,15,10,145,48,2,123,250,245,191,20,46,82,
    37,12,90,146,30,201,203,172,158,212,239,247,34,87,234,65,
    179,16,46,234,51,40,240,100,160,32,199,254,43,236,104,149,
    129,249,69,109,24,208,18,237,214,106,177,3,94,169,210,178,
    117,199,243,172,172,134,87,6,249,71,53,51,91,20,75,156,
    144,37,64,177,151,121,162,245,101,153,177,37,125,112,17,157,
    216,53,112,70,234,39,155,61,9,132,105,174,170,229,194,220,
    168,249,139,83,135,97,157,97,167,183,53,117,254,129,239,222,
    1,222,254,87,123,162,109,33,197,233,241,246,40,71,84,31,
    86,65,23,144,165,182,149,35,170,202,181,147,155,107,53,242,
    235,41,137,22,162,70,61,55,155,250,165,251,177,53,156,187,
    19,246,9,91,106,20,222,243,186,114,247,188,94,227,93,231,
    47,91,111,23,87,
};

EmbeddedPython embedded_m5_objects_VirtIO(
    "m5/objects/VirtIO.py",
    "/home/oliverxyy/program/gem5-stable/src/dev/virtio/VirtIO.py",
    "m5.objects.VirtIO",
    data_m5_objects_VirtIO,
    645,
    1323);

} // anonymous namespace
