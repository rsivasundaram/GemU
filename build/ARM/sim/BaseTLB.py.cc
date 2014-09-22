#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_BaseTLB[] = {
    120,156,181,80,193,78,195,48,12,117,218,110,192,196,129,51,
    18,130,99,132,180,6,14,211,46,8,80,197,17,129,212,237,
    66,47,85,218,120,180,168,89,167,38,19,219,25,254,27,236,
    178,193,23,224,36,79,182,242,228,247,236,18,118,17,210,187,
    191,0,112,151,148,24,186,2,26,0,43,32,19,32,76,0,
    40,96,33,192,132,240,9,240,1,240,146,5,96,34,152,201,
    1,209,235,47,10,41,40,243,71,4,179,218,62,23,111,88,
    122,127,64,85,162,29,206,31,147,114,47,196,180,132,133,78,
    41,65,224,254,36,71,237,48,132,44,98,221,108,0,233,76,
    6,244,157,50,217,141,24,106,171,124,83,196,85,37,135,172,
    115,72,144,231,75,109,49,207,253,168,47,108,107,214,13,151,
    17,19,182,43,236,147,121,183,198,158,173,11,231,59,77,166,
    152,93,110,54,121,133,218,96,39,217,211,31,184,59,2,85,
    181,22,85,167,173,122,104,223,151,77,171,141,83,175,104,39,
    99,231,117,209,224,88,235,197,213,245,20,181,54,83,163,92,
    87,42,118,183,155,51,94,109,123,215,231,220,140,173,14,5,
    159,39,201,11,246,199,4,118,18,255,110,40,133,253,148,255,
    224,163,31,251,230,103,45,183,103,220,148,119,122,18,124,3,
    17,251,107,50,
};

EmbeddedPython embedded_m5_objects_BaseTLB(
    "m5/objects/BaseTLB.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/sim/BaseTLB.py",
    "m5.objects.BaseTLB",
    data_m5_objects_BaseTLB,
    292,
    500);

} // anonymous namespace