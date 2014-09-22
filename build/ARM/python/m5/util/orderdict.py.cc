#include "sim/init.hh"

namespace {

const uint8_t data_m5_util_orderdict[] = {
    120,156,205,150,251,111,211,48,16,199,207,78,218,46,29,136,
    177,65,17,72,131,128,152,40,143,17,64,2,36,132,38,4,
    67,226,7,30,82,198,67,84,130,40,139,189,45,44,143,146,
    184,176,73,240,11,227,103,254,101,184,187,36,93,199,16,191,
    109,208,205,214,213,110,238,238,243,245,217,78,4,245,199,194,
    246,192,5,40,239,160,161,0,214,5,12,208,16,160,36,36,
    2,82,9,3,9,2,39,180,5,90,194,154,4,101,193,119,
    128,29,128,183,3,27,148,13,43,253,22,62,106,28,236,242,
    66,233,66,197,145,137,127,226,167,47,154,241,101,28,122,22,
    111,197,89,212,132,165,169,135,20,118,21,13,13,48,16,20,
    28,253,98,52,12,78,134,69,41,144,97,215,33,7,45,10,
    71,70,27,84,139,141,14,168,54,27,83,160,58,108,56,160,
    166,216,232,130,114,216,152,6,127,165,223,197,48,145,168,121,
    109,108,47,40,248,19,74,16,224,139,128,29,65,97,55,109,
    40,174,130,97,248,241,120,15,219,55,1,25,106,131,67,0,
    129,164,254,131,69,211,95,36,252,0,150,103,165,79,158,159,
    199,20,163,92,32,168,173,161,142,140,86,110,104,220,52,47,
    141,155,103,218,13,139,245,81,170,51,115,205,93,207,141,187,
    160,250,148,138,161,39,19,28,37,169,94,110,15,245,227,162,
    200,11,67,170,6,155,122,187,52,109,180,70,67,21,26,205,
    65,12,61,84,234,100,141,13,116,89,253,98,243,51,153,125,
    82,151,187,242,41,118,222,70,158,106,175,8,83,111,57,255,
    156,37,121,168,74,111,93,167,183,23,75,19,174,38,122,49,
    12,215,110,220,188,171,195,80,221,85,94,89,68,222,112,219,
    108,228,153,151,222,246,70,38,78,188,241,138,94,199,137,41,
    202,40,136,179,216,4,193,5,138,208,229,165,156,21,109,49,
    39,28,17,89,181,190,212,30,145,190,247,129,69,68,185,54,
    59,80,156,99,221,240,95,212,202,10,212,212,72,226,199,137,
    29,9,31,236,90,211,29,89,107,74,106,62,231,250,242,89,
    41,226,12,135,67,157,41,150,167,28,13,117,225,83,18,102,
    154,115,43,181,137,141,78,131,128,133,242,219,141,186,40,35,
    139,69,147,7,168,144,127,4,29,94,38,207,109,86,230,8,
    234,18,73,180,228,164,42,87,170,170,51,162,193,150,141,30,
    85,93,85,50,208,247,223,53,32,200,73,92,165,147,10,119,
    87,157,66,167,249,39,221,151,13,190,127,180,161,61,32,228,
    99,232,240,58,121,182,25,185,39,120,155,137,73,96,119,63,
    48,30,31,162,222,78,22,76,96,218,123,49,137,58,74,116,
    88,48,32,255,134,169,14,18,232,56,58,188,181,11,52,87,
    1,53,171,24,17,208,60,26,91,189,186,156,151,223,77,195,
    87,46,233,215,226,99,23,222,76,208,136,166,112,15,111,61,
    120,147,98,81,20,116,114,220,217,197,152,217,197,176,26,140,
    243,132,113,102,140,113,140,49,120,199,158,254,47,80,186,53,
    202,167,48,25,233,242,222,31,97,236,6,230,18,193,184,99,
    152,147,245,154,212,60,120,111,253,23,72,78,141,68,219,182,
    92,218,71,212,52,222,54,180,146,21,206,217,253,41,31,198,
    86,224,35,147,234,232,33,121,151,156,232,30,225,57,205,179,
    208,220,140,44,252,204,68,21,189,151,128,154,255,75,189,41,
    72,85,62,143,255,198,112,113,47,67,111,130,1,27,22,207,
    191,39,105,85,149,147,242,75,75,13,210,167,251,166,190,150,
    179,48,213,65,192,123,38,8,210,92,141,18,252,234,83,189,
    241,173,196,231,52,159,109,254,44,117,115,212,157,160,238,36,
    117,61,234,78,237,65,56,160,227,149,188,210,193,83,82,240,
    182,112,58,78,203,177,199,127,150,99,85,119,157,233,48,72,
    152,36,8,69,132,175,74,93,208,123,164,207,111,149,36,62,
    121,244,225,16,114,230,248,247,43,77,151,230,155,77,235,200,
    25,249,11,176,119,81,160,
};

EmbeddedPython embedded_m5_util_orderdict(
    "m5/util/orderdict.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/python/m5/util/orderdict.py",
    "m5.util.orderdict",
    data_m5_util_orderdict,
    887,
    2887);

} // anonymous namespace