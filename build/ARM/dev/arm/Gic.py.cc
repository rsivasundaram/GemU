#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Gic[] = {
    120,156,181,84,91,111,211,48,20,62,73,239,221,133,105,32,
    144,120,193,240,128,2,98,77,183,105,154,144,184,142,73,104,
    18,140,41,219,36,232,75,228,198,110,235,41,55,197,238,214,
    190,33,141,95,192,31,134,115,156,150,150,33,30,151,40,206,
    57,159,143,125,190,115,177,35,152,61,21,252,222,49,0,45,
    80,192,79,56,16,3,156,205,36,167,148,92,136,93,72,42,
    208,171,128,67,122,5,226,42,36,85,232,85,81,175,130,172,
    192,192,1,81,131,31,0,215,0,223,122,53,16,117,144,53,
    139,54,254,160,117,16,77,56,245,90,232,72,253,194,199,115,
    80,50,52,60,47,69,154,57,81,217,161,188,84,145,44,161,
    38,65,49,55,131,172,72,76,3,149,3,174,229,71,21,69,
    203,252,15,136,191,143,130,4,232,57,20,69,207,37,82,72,
    15,131,64,54,178,14,23,13,144,77,184,104,81,48,215,46,
    244,218,16,156,122,180,56,160,65,223,165,232,229,165,207,139,
    196,239,163,139,112,168,162,206,104,164,217,146,127,102,70,74,
    51,97,217,49,148,114,94,24,150,13,58,222,202,156,105,24,
    166,60,145,97,104,218,86,73,50,49,142,73,173,146,193,52,
    151,86,56,43,198,210,90,243,190,54,5,143,140,181,142,38,
    147,112,36,185,144,133,169,145,79,94,240,36,112,105,93,189,
    84,101,106,12,81,229,233,212,174,206,103,172,60,202,194,98,
    208,111,113,240,71,89,34,125,220,193,63,204,174,210,56,227,
    66,251,67,153,236,109,105,195,251,177,220,226,124,208,221,222,
    151,156,139,125,225,235,34,242,231,161,99,102,59,249,212,166,
    228,5,109,70,142,234,142,125,221,146,87,188,251,178,251,111,
    242,127,222,72,62,165,189,2,178,10,23,53,74,56,54,12,
    229,188,62,71,170,212,44,132,52,230,72,157,26,133,144,166,
    69,90,212,42,162,53,171,212,2,105,91,100,101,9,89,177,
    200,170,69,214,64,172,130,88,179,200,58,213,119,157,234,75,
    131,190,183,84,95,44,109,152,83,28,88,96,5,27,240,72,
    63,192,217,247,66,20,82,107,134,57,101,66,97,101,84,127,
    108,178,66,129,131,6,119,110,24,68,249,88,217,108,63,188,
    49,241,249,244,104,235,43,43,228,16,119,192,82,82,189,183,
    187,169,182,125,116,40,99,62,181,86,39,71,95,88,225,95,
    49,147,45,187,210,79,254,103,132,238,152,74,113,195,1,143,
    164,126,252,151,153,197,139,113,110,200,112,40,237,239,195,201,
    185,250,78,244,186,56,28,143,147,190,44,176,81,151,76,99,
    149,74,205,244,56,207,179,194,72,193,188,132,79,216,107,182,
    221,221,233,62,243,40,216,128,152,7,84,241,128,250,47,160,
    163,25,80,159,218,144,40,98,123,90,137,125,200,73,163,78,
    65,154,165,66,83,137,86,147,82,163,99,251,137,27,153,70,
    83,179,62,95,148,171,44,20,20,132,89,155,173,92,32,116,
    158,144,107,24,207,22,17,131,243,163,212,236,238,88,55,10,
    103,136,255,109,116,62,241,219,165,205,54,231,157,239,110,58,
    139,247,216,107,204,47,170,100,175,147,211,33,213,150,19,105,
    69,54,41,185,150,23,88,64,23,152,61,194,246,60,217,173,
    111,129,178,117,255,170,188,106,222,60,165,77,169,76,109,167,
    237,110,56,27,238,253,198,111,114,14,85,218,
};

EmbeddedPython embedded_m5_objects_Gic(
    "m5/objects/Gic.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/dev/arm/Gic.py",
    "m5.objects.Gic",
    data_m5_objects_Gic,
    748,
    1514);

} // anonymous namespace
