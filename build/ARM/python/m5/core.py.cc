#include "sim/init.hh"

namespace {

const uint8_t data_m5_core[] = {
    120,156,173,143,61,111,2,49,12,134,157,131,129,129,177,11,
    3,42,227,45,52,101,56,177,32,84,1,51,149,96,235,22,
    46,225,75,201,229,148,248,132,144,216,224,127,83,59,2,241,
    7,176,18,231,113,156,215,118,74,120,88,70,251,103,0,16,
    123,4,154,150,0,11,240,71,144,193,141,64,240,205,58,111,
    81,246,112,39,91,150,130,80,60,148,115,86,126,16,32,192,
    81,192,49,131,11,192,85,128,160,74,235,156,95,45,147,20,
    59,172,175,208,132,74,89,108,83,80,250,96,176,75,16,13,
    254,54,88,55,184,56,132,164,64,22,104,10,120,188,228,226,
    140,156,220,123,103,100,80,78,46,252,169,178,94,233,40,119,
    198,21,195,136,106,99,205,80,169,237,247,104,108,148,210,99,
    45,99,40,101,125,198,189,175,164,43,36,247,250,170,207,43,
    158,248,147,235,49,128,200,249,88,49,166,204,171,223,59,155,
    166,159,79,156,215,141,53,211,254,179,121,55,251,7,153,217,
    83,16,
};

EmbeddedPython embedded_m5_core(
    "m5/core.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/python/m5/core.py",
    "m5.core",
    data_m5_core,
    226,
    385);

} // anonymous namespace
