#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_ArmSystem[] = {
    120,156,181,85,219,110,219,70,16,93,202,23,93,124,141,81,
    24,45,80,160,251,212,10,69,35,54,15,177,209,164,104,18,
    249,82,4,136,10,131,86,130,212,47,236,138,28,73,91,147,
    92,98,119,25,89,207,233,71,244,243,250,39,237,204,80,150,
    149,20,109,209,7,139,210,104,121,184,151,179,103,206,44,19,
    177,248,172,225,239,185,20,194,253,130,141,20,191,129,200,132,
    24,82,171,33,178,64,228,129,184,10,68,144,174,9,104,136,
    113,32,210,117,241,155,16,239,133,248,249,106,77,164,27,2,
    2,70,55,151,232,186,72,155,2,214,25,109,45,209,13,145,
    182,197,101,183,131,139,232,63,241,211,13,176,229,41,124,93,
    55,55,49,92,206,157,135,220,239,98,243,133,205,7,42,153,
    234,2,134,243,18,146,85,182,125,98,251,8,27,32,136,154,
    222,88,208,62,66,194,72,243,8,25,34,47,254,219,172,255,
    154,226,232,170,33,162,203,110,139,150,127,138,147,248,45,108,
    69,160,178,55,26,102,241,89,95,231,77,4,183,87,193,139,
    254,91,253,71,107,129,190,57,187,41,45,56,23,159,189,26,
    126,8,156,188,248,238,163,30,131,65,151,104,122,90,44,142,
    11,149,67,28,251,14,223,228,38,173,50,186,165,14,185,42,
    187,180,167,187,224,206,49,132,83,147,67,104,85,30,158,154,
    89,145,25,149,186,112,2,249,227,135,206,171,81,6,15,149,
    26,127,251,232,24,148,74,143,211,208,217,36,84,54,153,98,
    200,67,212,172,86,176,87,206,163,6,73,75,115,210,202,155,
    65,39,104,242,229,219,181,184,117,199,191,235,58,90,234,138,
    162,162,104,168,43,229,185,33,48,175,176,33,126,221,20,208,
    36,149,223,35,210,98,164,45,106,193,9,233,48,178,69,194,
    163,232,132,108,175,32,45,70,118,40,17,196,34,162,28,184,
    3,12,203,29,184,154,254,116,250,146,60,194,146,28,98,24,
    84,153,215,165,53,9,234,107,172,172,123,61,243,252,188,135,
    225,92,103,32,253,84,121,153,152,194,43,93,56,188,3,57,
    50,198,75,18,16,44,62,72,65,234,177,84,197,92,243,184,
    47,72,135,52,197,148,73,51,230,238,63,190,60,145,39,23,
    175,165,46,60,216,177,194,197,190,89,246,89,118,26,103,106,
    226,164,133,137,70,14,86,142,145,206,224,130,87,210,197,164,
    75,206,141,40,239,17,109,208,175,83,64,251,114,246,147,155,
    155,120,10,68,134,13,78,188,98,133,147,199,185,114,215,126,
    3,161,11,133,89,231,65,125,99,50,110,12,109,85,143,206,
    73,130,152,52,168,43,197,91,92,143,93,76,107,199,245,46,
    121,8,17,102,71,78,116,18,39,101,197,139,240,28,204,157,
    111,239,209,119,68,137,138,211,237,178,239,234,235,128,47,222,
    247,43,93,84,55,255,98,192,223,255,211,128,232,62,116,28,
    89,169,201,72,139,220,135,254,34,164,205,72,135,220,183,176,
    219,22,35,104,195,29,58,129,8,217,93,65,58,140,236,173,
    32,91,140,236,175,32,219,140,60,32,219,238,80,106,63,163,
    221,125,186,106,219,140,246,116,103,94,58,224,246,34,202,133,
    123,70,238,173,207,49,169,83,57,182,38,151,83,239,203,39,
    97,56,155,205,122,56,184,199,131,123,198,78,122,213,117,152,
    194,59,200,76,9,54,204,235,81,46,212,2,207,71,119,188,
    98,197,217,20,44,200,20,198,10,77,33,149,39,71,58,111,
    171,196,87,136,187,169,169,178,84,142,64,206,172,246,30,138,
    136,204,224,158,255,115,157,156,194,59,157,128,28,90,0,217,
    207,204,168,39,79,77,241,149,151,149,195,103,195,62,149,13,
    228,165,159,247,184,32,160,32,59,72,80,54,155,203,107,176,
    5,100,88,145,249,200,100,146,141,226,112,105,44,11,144,131,
    193,107,247,244,110,0,218,200,187,208,163,217,177,194,198,70,
    166,85,94,162,135,229,130,13,220,120,233,102,218,39,83,172,
    166,170,72,149,213,184,215,144,107,64,79,38,88,107,74,146,
    29,101,169,10,157,16,39,162,62,169,192,249,91,22,252,228,
    127,142,49,166,116,224,186,7,31,20,110,196,121,230,98,222,
    163,176,79,129,14,84,174,171,69,102,98,170,236,136,142,39,
    46,46,78,3,23,87,244,201,109,207,212,143,226,49,106,78,
    239,128,232,1,161,84,230,231,42,115,224,169,19,107,24,215,
    76,226,90,67,100,182,148,44,94,232,18,215,186,196,172,96,
    76,186,113,37,241,166,98,83,196,220,240,59,171,16,109,235,
    30,139,156,74,224,9,205,249,249,74,145,183,63,186,14,26,
    7,141,159,186,116,90,241,75,39,127,220,43,233,128,115,81,
    112,123,54,158,21,85,206,186,242,169,193,179,222,31,105,126,
    31,127,95,191,129,127,248,146,230,38,160,211,216,111,28,54,
    15,219,127,1,140,15,63,214,
};

EmbeddedPython embedded_m5_objects_ArmSystem(
    "m5/objects/ArmSystem.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/arch/arm/ArmSystem.py",
    "m5.objects.ArmSystem",
    data_m5_objects_ArmSystem,
    1080,
    2320);

} // anonymous namespace
