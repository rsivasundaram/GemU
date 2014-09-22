#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_DummyChecker[] = {
    120,156,181,80,65,78,195,48,16,92,39,37,130,138,3,7,
    30,192,9,25,164,198,32,81,245,130,16,162,57,35,148,194,
    129,94,44,99,187,4,17,55,81,156,136,246,12,255,134,93,
    167,17,124,0,203,30,141,237,245,204,120,53,236,70,140,235,
    246,4,192,95,33,49,56,25,148,0,143,196,34,40,25,56,
    6,75,6,204,196,96,25,172,24,152,17,124,1,124,2,60,
    47,35,48,123,176,224,9,62,124,251,198,193,25,178,150,224,
    188,167,99,132,121,97,245,187,109,230,15,79,237,33,110,179,
    206,185,237,238,76,15,25,168,248,142,50,28,35,177,64,134,
    232,79,250,104,30,67,190,224,17,94,228,4,161,68,215,157,
    48,36,36,117,175,148,22,5,31,145,227,62,130,148,107,229,
    172,148,193,94,74,87,153,174,164,109,40,216,214,54,156,235,
    205,70,22,86,25,219,112,74,240,11,62,67,16,69,229,172,
    104,148,19,89,245,177,46,43,101,188,120,181,110,58,241,173,
    122,41,237,68,169,213,197,229,204,42,101,102,70,248,70,11,
    74,244,247,107,105,189,13,113,207,72,145,140,19,150,176,123,
    78,205,110,15,16,220,52,173,21,234,251,156,13,63,251,175,
    28,161,39,215,125,23,110,78,135,60,99,118,20,253,0,76,
    237,113,117,
};

EmbeddedPython embedded_m5_objects_DummyChecker(
    "m5/objects/DummyChecker.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/cpu/DummyChecker.py",
    "m5.objects.DummyChecker",
    data_m5_objects_DummyChecker,
    291,
    516);

} // anonymous namespace