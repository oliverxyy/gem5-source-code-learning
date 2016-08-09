#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_O3Checker[] = {
    120,156,173,80,177,78,195,48,16,61,39,37,130,78,32,177,
    34,117,180,144,136,135,210,9,132,16,221,1,165,48,208,197,
    10,182,105,42,98,18,217,41,74,102,248,111,122,231,36,240,
    3,88,246,211,59,251,124,239,221,41,24,86,140,231,118,6,
    224,47,145,104,220,12,74,128,39,98,17,148,12,44,131,53,
    3,166,99,48,12,222,24,232,9,124,3,124,1,188,172,35,
    208,7,176,226,9,126,220,254,224,226,12,89,67,112,222,211,
    41,194,178,48,234,221,184,229,227,115,115,132,225,195,124,184,
    80,163,1,202,188,35,3,167,72,12,144,26,138,83,113,84,
    142,33,91,241,8,31,50,2,127,130,160,234,157,168,230,66,
    245,101,210,162,224,19,210,58,68,144,242,35,183,70,202,32,
    44,165,173,244,174,164,48,36,116,181,9,247,170,109,101,97,
    114,109,28,39,249,63,240,87,8,162,168,172,17,85,185,253,
    52,174,237,58,81,187,106,227,114,43,54,198,46,46,124,147,
    191,150,70,120,167,196,224,226,183,157,180,238,130,197,25,21,
    34,189,132,37,236,158,211,116,67,223,118,145,214,57,22,242,
    25,27,187,249,103,249,48,129,235,190,231,155,179,209,198,148,
    29,71,123,53,127,104,127,
};

EmbeddedPython embedded_m5_objects_O3Checker(
    "m5/objects/O3Checker.py",
    "/home/oliverxyy/program/gem5-stable/src/cpu/o3/O3Checker.py",
    "m5.objects.O3Checker",
    data_m5_objects_O3Checker,
    279,
    492);

} // anonymous namespace