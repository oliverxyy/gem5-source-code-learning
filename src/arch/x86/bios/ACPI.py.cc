#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_ACPI[] = {
    120,156,181,85,235,75,27,65,16,159,203,59,81,171,104,77,
    105,161,112,133,126,72,139,122,208,162,4,233,203,86,40,1,
    177,114,209,98,253,18,46,183,107,114,146,187,11,187,171,36,
    126,181,255,119,59,51,121,157,164,167,95,210,92,178,217,153,
    157,157,157,199,111,127,231,195,248,147,197,223,23,27,64,223,
    224,68,224,215,130,30,192,41,205,50,208,179,32,204,192,69,
    6,44,145,5,153,129,75,11,68,14,126,3,220,1,252,186,
    200,130,200,131,204,178,182,48,213,230,64,20,39,218,210,84,
    155,7,81,158,120,168,76,181,5,16,75,208,172,45,227,209,
    193,31,252,212,44,156,25,26,222,142,166,101,28,154,65,248,
    163,125,37,125,99,54,80,58,175,239,29,124,59,105,52,135,
    250,80,106,255,212,107,247,164,159,204,229,43,229,114,139,19,
    9,112,97,81,70,24,62,38,69,209,98,42,57,144,121,192,
    115,101,17,174,74,128,89,97,62,119,168,47,39,52,121,214,
    84,88,179,68,169,97,66,164,89,78,216,148,88,179,146,176,
    41,179,230,9,184,205,90,5,143,119,51,56,232,23,163,136,
    27,205,131,253,125,10,123,127,63,25,184,222,196,101,79,249,
    93,103,80,223,115,218,65,172,29,207,239,7,59,221,174,129,
    201,110,109,84,16,117,236,64,200,200,4,151,67,154,155,174,
    180,99,25,106,42,27,254,219,134,124,217,141,195,128,55,189,
    26,107,149,188,9,116,16,71,118,116,29,182,165,178,47,99,
    197,59,217,90,111,167,187,238,200,72,42,207,160,121,124,153,
    216,177,133,59,210,124,250,74,206,109,168,173,82,3,75,56,
    180,90,145,23,202,86,203,84,88,8,99,113,221,35,49,71,
    6,195,190,228,46,251,131,65,203,239,121,90,179,21,73,93,
    233,9,169,216,234,84,93,75,118,229,181,49,104,15,161,144,
    71,225,196,83,94,104,10,4,17,206,132,167,152,122,43,16,
    102,92,155,22,135,66,10,90,59,107,68,230,253,187,233,218,
    36,155,209,137,163,20,200,116,45,33,78,108,106,84,219,217,
    160,63,226,224,116,227,80,58,113,47,184,145,106,48,28,58,
    125,21,119,48,34,167,35,195,221,109,205,39,59,90,249,206,
    253,14,19,12,118,250,67,198,199,27,114,85,197,161,96,141,
    159,204,186,133,79,166,108,209,99,150,102,128,119,155,135,167,
    243,64,223,122,24,232,87,5,232,0,225,149,160,89,36,104,
    82,53,93,186,71,122,125,14,154,116,68,58,36,245,115,218,
    53,212,6,177,37,16,193,42,232,27,130,194,8,29,53,234,
    142,203,222,169,53,46,149,219,45,210,64,11,156,200,79,188,
    194,177,226,166,113,242,134,150,17,122,42,192,237,11,47,47,
    229,88,39,87,165,89,121,51,201,130,158,47,176,160,213,127,
    23,244,252,191,21,212,125,58,161,24,119,243,94,229,22,85,
    62,202,232,243,67,229,67,176,156,204,151,79,62,90,190,25,
    229,22,89,83,34,190,69,254,28,147,48,106,144,120,151,18,
    148,139,26,164,217,21,122,97,140,89,151,57,150,56,195,125,
    150,10,228,147,244,186,187,217,199,185,53,96,246,126,147,164,
    60,228,54,242,110,183,37,217,93,107,41,182,236,91,169,98,
    59,136,132,28,72,161,95,146,117,28,27,59,181,167,68,205,
    114,96,36,110,16,169,86,204,155,233,141,167,22,184,196,96,
    238,10,221,161,252,152,214,234,76,143,147,96,25,253,204,156,
    199,103,71,71,60,81,90,24,238,42,75,3,148,22,15,26,
    106,199,119,114,181,150,0,205,122,118,29,73,109,217,58,174,
    81,18,204,245,225,238,78,159,104,64,51,21,163,52,125,197,
    187,214,20,215,27,147,139,197,110,23,30,44,215,235,195,232,
    93,244,233,53,185,164,183,64,197,90,203,85,87,171,197,106,
    233,47,133,77,7,87,
};

EmbeddedPython embedded_m5_objects_ACPI(
    "m5/objects/ACPI.py",
    "/home/oliverxyy/program/gem5-stable/src/arch/x86/bios/ACPI.py",
    "m5.objects.ACPI",
    data_m5_objects_ACPI,
    854,
    2337);

} // anonymous namespace
