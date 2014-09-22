#include "sim/init.hh"

namespace {

const uint8_t data_m5_defines[] = {
    120,156,141,83,75,111,218,64,16,30,59,73,9,143,196,9,
    61,244,208,67,171,158,184,36,110,43,161,92,162,170,142,49,
    5,5,108,138,13,141,104,163,213,194,46,196,212,143,212,94,
    242,56,231,71,228,143,245,255,180,179,11,164,225,214,213,122,
    252,205,55,59,223,204,142,229,9,172,86,25,159,207,111,1,
    242,223,8,24,110,13,34,128,145,182,198,186,196,92,131,249,
    22,204,183,129,233,192,119,96,170,3,219,90,131,109,224,47,
    20,56,0,86,88,227,221,117,244,16,88,121,77,86,97,86,
    132,7,13,70,5,37,184,11,243,34,204,75,48,42,1,47,
    195,3,22,173,192,93,251,89,104,15,230,251,146,111,92,126,
    132,31,216,135,1,163,3,224,6,204,81,116,95,10,101,143,
    210,101,6,188,25,85,129,99,172,2,188,10,167,191,30,1,
    247,183,239,21,121,1,191,246,18,47,22,254,193,229,138,87,
    8,93,143,12,29,59,240,250,228,204,27,184,13,159,216,45,
    199,62,247,197,46,198,6,190,67,154,142,59,20,7,232,248,
    24,242,186,61,43,104,159,117,144,238,137,18,146,129,213,255,
    226,4,164,237,91,98,11,93,154,197,66,78,208,238,17,203,
    117,189,192,10,28,97,172,132,122,158,223,190,32,118,199,179,
    207,149,120,175,239,5,158,237,117,148,78,183,77,248,29,141,
    175,35,46,246,208,109,89,67,76,144,7,206,6,77,241,250,
    137,113,250,77,98,5,65,159,56,23,118,103,208,112,72,203,
    243,131,80,195,184,216,65,51,141,232,140,132,18,213,116,52,
    249,102,135,249,170,195,101,236,121,11,249,102,11,203,3,255,
    83,85,13,83,221,56,174,31,135,137,224,89,66,35,161,43,
    95,20,150,244,66,132,145,216,70,172,64,81,78,50,166,153,
    104,132,19,161,232,32,91,112,213,125,147,70,57,87,163,25,
    47,194,136,57,201,141,114,158,100,229,225,73,154,113,85,111,
    146,198,215,97,196,27,84,112,85,104,22,165,99,204,87,25,
    100,195,33,12,43,17,162,42,135,40,133,79,156,171,175,245,
    147,223,171,247,13,138,203,9,228,2,251,202,111,67,113,165,
    106,201,105,214,228,63,241,207,228,231,104,204,171,52,230,102,
    70,99,179,145,222,38,81,74,89,110,206,120,92,63,194,252,
    113,196,143,40,157,190,255,112,194,41,101,39,204,84,87,49,
    173,126,215,188,190,23,87,105,98,198,117,147,241,105,152,240,
    252,24,25,217,225,105,156,178,69,196,63,201,129,230,85,52,
    21,173,162,127,213,13,173,168,189,211,12,173,164,29,106,127,
    1,67,40,208,181,
};

EmbeddedPython embedded_m5_defines(
    "m5/defines.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/defines.py",
    "m5.defines",
    data_m5_defines,
    581,
    927);

} // anonymous namespace