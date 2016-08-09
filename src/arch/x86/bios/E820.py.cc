#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_E820[] = {
    120,156,173,83,75,111,211,64,16,158,117,30,78,82,14,37,
    18,28,56,249,132,92,36,226,170,162,81,84,21,68,43,245,
    80,164,2,114,10,130,92,44,199,222,196,174,252,136,118,23,
    20,115,13,255,27,102,198,117,154,34,144,56,212,143,209,204,
    183,187,51,223,60,54,130,219,167,133,255,91,7,64,191,67,
    37,198,79,64,6,112,77,154,5,153,128,220,130,153,5,34,
    110,129,180,96,33,32,110,195,79,128,13,192,215,89,11,226,
    78,131,118,183,104,27,98,27,166,110,15,221,165,191,240,113,
    5,106,134,196,139,90,237,163,152,166,249,135,249,141,140,140,
    121,132,214,151,201,248,98,114,116,120,81,24,85,69,187,196,
    206,137,216,21,42,18,96,38,136,30,114,65,134,20,26,121,
    181,65,118,224,166,11,72,14,105,109,16,177,25,233,17,49,
    164,68,72,159,145,1,145,218,224,193,61,240,107,110,190,133,
    66,63,174,163,95,78,207,78,78,182,20,244,19,68,67,21,
    37,222,122,50,246,230,105,169,61,137,107,163,36,73,137,152,
    126,78,203,113,172,164,214,78,185,112,76,34,157,185,92,166,
    69,145,22,203,6,80,8,148,133,161,32,135,231,122,72,199,
    210,31,242,254,42,195,166,90,253,1,187,84,18,67,28,131,
    160,8,115,25,4,102,192,70,94,198,223,50,50,219,183,231,
    184,148,209,122,29,68,89,168,53,239,34,43,145,97,44,149,
    233,160,249,49,84,97,206,251,207,144,48,43,196,156,183,94,
    201,188,84,213,20,105,49,78,252,76,23,149,79,151,133,25,
    191,226,45,42,44,150,50,160,80,46,101,126,39,244,107,20,
    94,82,230,210,43,179,244,187,84,235,170,242,86,170,92,98,
    56,111,41,243,227,151,218,132,243,76,122,90,69,222,253,82,
    82,153,71,171,138,235,127,64,174,40,221,174,224,215,26,138,
    161,216,29,137,107,114,178,29,9,171,25,137,131,255,25,9,
    106,183,77,237,166,236,252,253,191,181,155,221,255,187,221,250,
    25,197,193,145,72,165,118,22,165,226,38,209,154,195,201,213,
    115,196,222,169,214,62,21,207,183,73,208,130,217,67,241,25,
    71,188,84,220,5,78,216,216,119,30,31,190,164,148,227,17,
    185,234,237,148,244,189,219,105,110,93,126,60,90,17,21,205,
    37,70,107,123,13,125,209,92,9,118,242,224,212,120,156,79,
    235,1,126,67,247,71,83,177,6,98,223,122,218,255,13,181,
    216,252,138,
};

EmbeddedPython embedded_m5_objects_E820(
    "m5/objects/E820.py",
    "/home/oliverxyy/program/gem5-stable/src/arch/x86/bios/E820.py",
    "m5.objects.E820",
    data_m5_objects_E820,
    547,
    1163);

} // anonymous namespace