#include "sim/init.hh"

namespace {

const uint8_t data_m5_ticks[] = {
    120,156,181,148,223,111,27,69,16,199,103,239,28,59,231,184,
    109,104,73,136,10,136,83,69,136,165,54,57,120,136,34,16,
    10,165,9,13,60,80,85,151,60,69,72,214,250,118,237,92,
    114,63,210,221,117,139,165,248,133,240,7,240,208,39,30,121,
    227,223,225,63,225,29,9,102,102,125,142,168,120,164,39,251,
    110,111,118,111,230,59,51,159,221,12,230,87,11,255,143,99,
    0,59,192,129,194,159,128,2,224,212,143,3,40,4,148,1,
    156,6,32,84,8,50,4,221,2,185,4,170,5,63,227,162,
    54,168,37,30,116,64,181,225,116,25,84,135,95,35,80,56,
    198,123,23,212,10,140,91,112,218,37,191,199,253,30,198,200,
    255,198,235,89,95,224,208,81,240,87,210,84,99,124,254,246,
    250,126,249,36,35,73,52,21,226,255,128,100,29,2,107,66,
    53,40,107,134,223,8,176,143,193,5,32,5,92,1,156,135,
    112,222,2,183,4,174,13,215,130,126,2,85,47,222,215,143,
    190,173,56,149,227,62,121,124,198,193,237,67,28,30,21,245,
    80,22,241,200,232,23,19,93,101,211,216,106,23,75,23,111,
    170,216,229,217,133,141,47,181,65,91,86,87,170,223,33,173,
    203,164,189,114,218,84,178,112,17,89,46,237,96,148,255,168,
    21,167,113,98,38,154,7,89,109,180,123,7,7,232,240,160,
    168,179,139,167,77,8,23,122,23,252,196,175,185,6,41,221,
    250,208,220,236,1,222,146,179,186,212,137,145,101,114,88,191,
    170,138,90,42,155,140,117,185,187,109,157,28,22,122,91,202,
    209,167,159,237,105,41,213,158,74,172,201,146,203,169,59,171,
    171,164,220,77,88,251,14,190,223,69,63,168,206,167,185,144,
    240,49,133,232,114,145,123,65,91,180,197,186,200,72,64,48,
    39,129,75,254,235,188,228,1,149,188,20,48,195,162,98,193,
    205,39,128,202,17,3,76,115,253,167,0,176,176,88,103,236,
    1,150,218,117,96,20,192,53,46,58,36,139,108,65,149,54,
    179,203,108,255,161,177,127,209,216,35,182,95,131,235,194,149,
    128,243,21,50,250,22,210,178,123,224,122,196,153,187,53,183,
    83,72,110,228,210,162,145,30,162,14,23,189,122,169,141,179,
    219,255,213,89,89,24,45,213,52,230,102,113,143,71,190,199,
    137,221,177,15,8,65,83,87,227,216,77,47,117,188,181,105,
    183,226,81,109,252,130,231,218,28,123,4,110,55,145,202,221,
    157,137,203,139,148,8,72,169,108,142,166,190,118,206,228,195,
    137,211,223,24,83,155,148,129,161,50,231,54,175,176,105,85,
    166,211,118,3,124,129,193,28,37,49,194,206,122,24,172,51,
    108,49,245,164,82,236,80,86,211,147,250,134,28,138,118,130,
    250,216,61,123,33,181,253,69,252,127,171,101,113,111,137,41,
    164,250,13,166,30,145,117,131,153,90,109,181,69,40,110,5,
    107,34,18,119,248,191,17,132,98,252,215,239,127,254,241,253,
    240,249,87,89,179,179,23,168,189,38,237,192,56,8,194,193,
    110,210,214,70,248,176,44,55,109,175,184,114,118,143,200,65,
    42,189,5,167,145,197,11,68,232,136,6,199,30,196,207,103,
    220,39,132,110,65,211,140,15,10,68,108,3,31,239,207,2,
    184,10,8,190,139,22,152,95,168,75,8,153,159,191,14,65,
    84,60,154,67,102,63,34,180,100,181,229,226,57,96,158,15,
    87,123,88,184,48,251,184,230,187,42,174,141,194,3,3,103,
    84,61,95,108,243,186,178,143,98,119,166,227,241,155,72,150,
    19,235,226,161,246,72,230,220,164,164,233,127,142,44,106,234,
    115,188,143,254,10,109,8,159,110,140,23,114,203,43,144,98,
    12,180,169,250,43,4,33,29,170,41,65,144,82,127,210,123,
    13,152,41,207,18,138,204,94,186,74,181,166,217,133,140,193,
    194,125,202,176,240,185,68,153,191,148,5,158,102,145,63,174,
    6,254,141,38,81,214,91,192,106,133,53,213,165,135,215,62,
    33,223,31,48,79,119,196,58,19,21,5,61,209,10,187,193,
    93,113,91,244,144,171,174,72,215,40,37,58,147,211,251,116,
    91,39,85,228,200,239,167,169,77,105,251,113,90,126,55,82,
    90,79,101,97,181,175,13,127,185,214,124,201,62,120,123,15,
    6,178,40,6,131,155,44,255,223,84,41,236,151,101,173,38,
    133,222,255,144,28,83,203,122,98,149,206,226,48,90,142,222,
    109,7,209,123,145,248,7,152,115,180,162,
};

EmbeddedPython embedded_m5_ticks(
    "m5/ticks.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/python/m5/ticks.py",
    "m5.ticks",
    data_m5_ticks,
    987,
    1950);

} // anonymous namespace
