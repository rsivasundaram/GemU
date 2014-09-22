#include "sim/init.hh"

namespace {

const uint8_t data_m5_trace[] = {
    120,156,181,81,77,75,3,49,16,157,236,110,87,11,197,139,
    224,77,16,65,216,75,27,61,148,94,68,132,22,143,61,164,
    158,246,34,113,19,251,65,178,41,73,86,241,238,255,214,153,
    32,109,189,138,134,100,120,25,102,222,123,147,52,240,189,50,
    60,247,23,0,97,134,64,225,102,96,0,234,29,102,80,179,
    132,51,48,25,216,28,234,28,108,1,117,1,76,229,240,129,
    165,61,80,69,2,37,53,44,170,30,18,173,63,113,205,43,
    34,143,37,6,215,197,109,23,19,92,47,91,231,117,115,168,
    63,37,253,83,170,197,205,96,147,193,38,135,167,130,100,23,
    21,195,252,60,145,70,10,15,210,4,29,143,137,168,141,218,
    183,210,164,116,244,178,209,177,64,212,188,74,31,143,16,232,
    86,62,27,173,42,82,217,135,48,197,192,87,206,106,238,165,
    229,51,247,214,26,39,85,224,75,109,199,195,16,169,103,40,
    229,203,245,205,68,75,169,38,138,7,223,240,237,123,92,185,
    150,219,49,79,66,35,188,147,132,90,7,170,191,36,94,26,
    4,216,175,230,34,219,143,190,211,34,71,32,232,38,40,47,
    202,31,206,255,214,126,185,123,161,171,189,251,138,134,74,46,
    146,167,46,174,77,60,57,120,234,81,106,23,84,43,200,191,
    72,245,253,127,180,73,31,125,107,157,234,140,190,59,39,98,
    74,12,216,32,59,203,250,249,23,1,250,146,84,
};

EmbeddedPython embedded_m5_trace(
    "m5/trace.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/python/m5/trace.py",
    "m5.trace",
    data_m5_trace,
    317,
    709);

} // anonymous namespace
