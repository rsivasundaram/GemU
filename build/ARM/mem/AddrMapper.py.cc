#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_AddrMapper[] = {
    120,156,181,146,77,111,19,65,12,134,61,249,216,36,165,21,
    168,7,206,123,66,11,82,179,128,84,245,82,85,180,226,132,
    84,168,54,21,18,185,172,38,25,39,155,106,39,179,154,153,
    64,115,46,255,27,108,39,75,79,112,64,234,38,59,177,157,
    25,251,153,215,158,195,254,233,210,251,33,5,8,159,200,48,
    244,85,80,3,220,178,213,129,90,129,85,48,85,160,76,23,
    80,193,66,129,233,193,79,128,7,128,111,211,14,152,62,96,
    71,162,201,159,104,23,204,0,38,217,144,210,173,126,209,147,
    41,178,34,47,111,118,230,136,150,107,180,95,102,119,56,143,
    241,128,188,75,99,252,181,110,26,244,243,22,171,67,239,21,
    99,189,39,3,129,25,8,137,75,18,15,177,244,96,74,181,
    19,134,124,160,200,0,112,8,196,200,246,8,138,73,214,163,
    99,5,39,9,199,180,88,180,185,166,34,165,149,42,227,170,
    10,207,152,66,135,136,62,109,156,143,129,57,38,181,254,142,
    226,102,236,70,190,67,89,174,181,197,178,20,208,178,180,206,
    108,106,118,185,64,220,54,40,241,249,253,125,89,161,54,232,
    37,126,235,55,40,135,245,44,68,175,247,183,220,85,187,161,
    236,49,97,38,113,69,13,169,43,127,244,153,152,189,140,69,
    120,92,194,21,45,121,229,44,230,94,219,252,163,251,177,174,
    157,54,33,95,162,61,61,9,81,207,106,60,209,122,241,246,
    221,25,106,109,206,76,30,252,60,231,107,63,106,59,110,182,
    162,72,206,249,24,40,81,242,233,30,170,248,156,229,210,235,
    37,254,87,43,238,250,109,31,146,214,237,238,219,210,182,226,
    232,31,173,200,218,226,33,117,139,148,54,56,191,77,99,165,
    99,26,42,183,169,13,133,82,143,178,223,132,215,127,221,172,
    61,166,51,92,173,151,233,110,107,26,157,76,97,193,35,94,
    8,5,203,91,176,248,145,219,255,149,230,207,249,27,77,138,
    74,27,248,238,146,90,228,112,126,181,92,173,117,93,122,169,
    38,177,150,98,31,123,162,38,177,86,231,156,239,176,109,210,
    32,81,35,250,253,44,90,10,171,61,29,55,12,30,10,213,
    78,186,156,123,26,36,25,230,243,221,236,95,188,226,188,44,
    226,129,122,49,124,121,244,27,254,199,251,66,
};

EmbeddedPython embedded_m5_objects_AddrMapper(
    "m5/objects/AddrMapper.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/mem/AddrMapper.py",
    "m5.objects.AddrMapper",
    data_m5_objects_AddrMapper,
    508,
    1098);

} // anonymous namespace