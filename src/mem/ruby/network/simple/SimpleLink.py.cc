#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_SimpleLink[] = {
    120,156,181,82,77,79,131,64,16,157,229,171,180,209,196,131,
    233,209,152,24,35,49,177,123,234,205,24,211,196,131,137,31,
    9,245,34,23,66,97,109,105,217,66,128,42,156,245,127,235,
    204,66,91,189,217,67,9,188,204,60,150,153,121,195,11,161,
    189,116,124,110,79,1,138,57,6,17,222,12,18,128,151,54,
    98,77,164,65,162,129,212,193,211,129,81,174,67,98,128,52,
    193,51,65,90,224,89,200,26,32,44,120,99,16,153,240,5,
    240,9,240,234,117,32,178,64,152,138,237,108,88,27,34,27,
    198,78,23,219,197,223,120,57,12,163,146,224,178,9,233,205,
    56,150,207,147,185,8,75,71,35,234,0,97,20,20,113,120,
    191,44,31,226,229,98,75,220,85,13,113,216,124,148,37,162,
    101,194,181,64,170,57,34,129,199,24,8,0,143,145,76,79,
    35,125,168,199,29,171,22,174,129,80,156,35,72,33,121,190,
    154,212,124,41,202,143,52,95,240,66,149,229,77,117,42,61,
    152,205,28,58,94,218,8,190,191,12,164,240,253,178,167,18,
    153,70,171,132,82,117,160,206,132,226,195,170,242,103,34,136,
    68,238,208,72,91,40,30,17,248,44,149,130,167,73,252,46,
    242,170,174,121,150,167,211,60,144,124,42,228,240,170,40,131,
    9,118,47,242,144,255,99,178,172,86,66,206,168,52,5,22,
    179,216,175,221,180,235,219,117,55,221,221,119,227,154,4,22,
    65,135,192,254,35,123,95,218,105,208,139,223,218,159,148,207,
    148,165,228,112,144,5,88,186,80,191,141,178,60,173,106,229,
    36,76,54,134,115,55,30,84,254,162,202,174,90,130,190,118,
    137,234,178,119,45,106,202,235,198,78,55,39,212,130,140,212,
    99,61,118,196,250,90,223,248,1,120,1,220,242,
};

EmbeddedPython embedded_m5_objects_SimpleLink(
    "m5/objects/SimpleLink.py",
    "/home/oliverxyy/program/gem5-stable/src/mem/ruby/network/simple/SimpleLink.py",
    "m5.objects.SimpleLink",
    data_m5_objects_SimpleLink,
    397,
    961);

} // anonymous namespace