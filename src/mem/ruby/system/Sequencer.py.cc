#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Sequencer[] = {
    120,156,181,86,91,79,227,86,16,62,118,66,18,18,32,33,
    176,236,189,107,181,47,105,171,37,79,60,84,170,170,118,65,
    43,245,129,22,153,109,165,242,98,25,123,32,206,218,113,234,
    115,66,201,91,37,250,71,250,135,250,151,218,249,198,151,152,
    133,173,84,169,16,56,204,204,57,103,206,92,191,73,160,138,
    159,6,255,125,235,40,165,127,103,34,228,95,75,197,74,189,
    43,40,43,167,108,21,219,42,177,213,153,173,172,176,161,200,
    86,23,150,10,155,234,15,165,110,148,250,229,172,161,194,53,
    69,13,145,182,42,105,83,133,237,82,218,169,164,107,42,92,
    47,53,116,43,105,75,133,61,117,58,218,96,35,162,191,249,
    103,100,49,101,176,124,145,147,235,188,28,83,242,227,249,148,
    2,99,58,204,185,139,243,229,73,154,153,160,238,202,27,184,
    242,23,19,164,212,153,5,135,216,102,182,1,198,88,138,31,
    167,150,154,182,225,17,251,113,195,30,117,20,173,195,149,27,
    222,237,42,234,193,19,208,27,138,54,225,12,232,173,26,221,
    87,52,128,95,160,183,133,238,8,61,20,205,59,138,118,75,
    205,143,68,178,39,44,31,120,44,236,19,69,79,213,244,25,
    98,128,51,207,203,91,13,4,3,146,23,53,73,79,36,47,
    149,155,7,198,181,121,209,207,121,73,40,25,103,236,254,88,
    47,181,97,186,12,197,254,100,18,33,18,6,139,222,226,229,
    240,228,39,71,199,254,21,57,115,222,215,253,66,148,248,124,
    47,203,101,187,69,44,29,214,122,107,99,167,220,152,71,105,
    93,201,160,118,33,205,150,185,112,19,47,138,53,78,42,57,
    210,47,81,80,190,241,157,192,15,38,228,100,244,235,130,180,
    209,142,94,204,113,131,66,57,17,205,180,249,232,137,209,39,
    112,6,201,246,188,153,159,144,231,153,174,48,73,26,46,98,
    176,77,28,88,206,73,136,119,217,130,228,180,127,174,77,230,
    115,157,224,116,112,125,237,77,200,15,41,51,107,204,158,248,
    153,159,24,212,202,247,51,99,218,252,255,138,50,29,165,51,
    131,224,252,204,150,167,217,41,156,69,64,229,134,184,110,6,
    213,238,177,196,72,182,91,72,134,176,242,84,109,7,202,56,
    110,94,190,235,205,75,25,199,236,150,12,117,189,122,110,171,
    184,37,79,230,7,182,138,75,53,17,124,125,147,166,177,88,
    247,214,143,53,153,109,166,22,58,154,93,122,168,11,207,80,
    101,19,50,117,42,137,49,61,102,101,59,79,148,88,127,186,
    34,57,50,196,33,65,104,252,217,82,68,197,65,104,47,146,
    226,33,165,30,167,74,223,146,34,141,34,29,161,244,86,139,
    62,226,101,60,73,19,26,167,113,196,145,190,94,46,199,243,
    44,189,228,36,140,47,41,57,120,173,141,127,30,211,88,103,
    193,248,195,178,62,69,65,204,2,202,246,231,75,41,254,79,
    161,17,53,209,178,242,207,208,222,176,170,143,61,180,250,214,
    35,107,104,153,205,26,60,156,100,233,245,178,194,8,171,196,
    136,221,15,48,2,232,208,64,167,225,29,87,30,114,254,165,
    211,68,43,183,219,8,185,112,101,65,50,92,4,205,93,191,
    21,130,255,57,14,48,237,0,26,155,69,28,42,111,171,99,
    119,17,113,118,175,183,140,130,140,139,180,166,166,173,18,165,
    218,183,217,142,176,2,144,192,69,91,48,146,37,61,192,33,
    80,208,22,164,100,201,166,162,173,18,249,250,136,163,64,244,
    103,37,116,87,182,233,23,247,68,117,229,224,100,226,194,232,
    8,221,166,191,148,238,186,94,1,195,40,154,5,241,190,51,
    207,232,130,12,67,134,254,220,73,23,134,35,55,11,185,244,
    35,231,207,182,210,111,139,75,181,13,39,88,6,49,105,231,
    34,205,28,191,212,230,156,19,243,228,132,12,13,113,26,188,
    31,35,45,32,88,18,196,220,10,225,104,112,55,183,226,12,
    16,133,143,104,45,153,118,187,165,28,73,56,4,148,73,239,
    68,65,69,134,66,186,232,63,243,36,183,207,171,217,231,149,
    14,202,225,67,49,214,12,113,175,176,205,51,147,140,244,36,
    141,67,247,49,222,129,14,179,91,181,252,140,204,111,105,246,
    190,232,250,7,171,60,36,243,43,85,244,78,209,129,118,159,
    123,110,189,248,24,76,169,163,227,239,254,115,33,82,83,77,
    215,106,67,185,165,168,93,14,229,142,236,242,87,134,110,185,
    219,19,137,12,232,105,173,230,32,28,148,194,188,88,183,81,
    136,210,146,175,97,247,171,123,74,175,110,110,57,62,165,4,
    53,224,237,136,174,162,128,234,195,239,206,156,27,13,63,130,
    0,82,23,146,115,87,166,183,204,89,160,249,42,135,238,83,
    44,207,176,96,22,186,128,54,23,3,222,69,151,184,175,30,
    18,71,16,17,244,138,124,77,200,115,57,100,20,29,50,130,
    254,48,106,151,37,157,28,236,207,49,48,181,204,85,112,0,
    62,23,48,234,86,72,41,149,33,10,31,202,90,121,253,235,
    124,234,127,131,40,105,132,180,107,117,173,129,189,183,179,215,
    220,219,252,7,85,121,134,177,
};

EmbeddedPython embedded_m5_objects_Sequencer(
    "m5/objects/Sequencer.py",
    "/home/oliverxyy/program/gem5-stable/src/mem/ruby/system/Sequencer.py",
    "m5.objects.Sequencer",
    data_m5_objects_Sequencer,
    1128,
    2769);

} // anonymous namespace