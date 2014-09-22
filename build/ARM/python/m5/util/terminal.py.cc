#include "sim/init.hh"

namespace {

const uint8_t data_m5_util_terminal[] = {
    120,156,181,85,221,79,27,71,16,159,189,243,55,103,236,144,
    128,19,212,72,167,68,180,86,37,106,34,148,228,165,68,149,
    177,4,18,42,82,15,85,36,86,209,245,240,46,206,153,251,
    112,110,215,9,150,204,19,60,246,181,143,125,235,191,213,191,
    165,157,153,243,145,164,239,128,111,119,118,119,102,119,231,247,
    155,153,29,193,242,207,198,239,39,23,64,191,16,0,18,127,
    2,34,128,33,10,22,76,4,220,160,108,65,88,5,105,131,
    44,193,43,89,6,89,193,14,39,106,216,213,65,54,176,91,
    1,233,96,215,4,185,138,93,11,100,27,94,13,109,80,54,
    76,74,188,5,90,61,128,91,20,42,48,127,94,28,83,133,
    97,21,84,21,38,53,210,17,114,141,53,234,112,154,52,65,
    8,161,42,52,72,4,188,149,15,65,53,224,66,128,124,68,
    42,168,252,110,184,2,10,127,117,184,17,48,116,88,174,176,
    220,4,5,48,89,133,73,139,244,178,127,64,57,48,108,67,
    82,1,213,36,1,15,150,235,112,139,154,107,32,55,248,68,
    220,253,17,200,14,92,90,144,109,11,249,24,14,14,213,67,
    178,195,253,4,118,147,117,156,145,79,150,211,77,158,78,216,
    133,147,238,38,194,23,254,139,127,199,186,139,98,63,10,70,
    151,174,167,164,123,144,41,149,184,239,84,20,165,159,220,126,
    52,83,238,207,193,88,37,38,112,247,231,65,98,74,168,124,
    158,70,146,133,62,9,68,132,12,99,238,7,216,151,73,35,
    10,147,75,150,250,44,145,178,142,103,145,169,163,240,107,34,
    85,134,211,138,77,50,245,209,84,177,247,212,71,149,105,181,
    84,213,169,169,161,112,98,130,68,166,51,147,207,142,179,29,
    83,65,225,56,205,226,32,26,33,239,96,225,71,253,1,69,
    2,41,177,115,52,123,108,40,74,186,220,208,216,144,154,230,
    141,130,108,172,187,119,171,250,8,155,222,251,52,86,189,44,
    136,123,131,244,83,18,165,129,212,189,177,138,95,110,107,19,
    156,71,106,59,8,46,118,94,188,86,65,32,95,203,158,206,
    70,189,233,220,188,79,147,94,252,178,55,51,97,212,51,42,
    139,195,36,136,126,192,249,22,238,151,204,162,200,31,5,83,
    95,155,44,76,198,125,58,135,110,1,130,175,109,47,63,190,
    246,54,221,14,40,104,23,64,28,93,91,176,64,70,183,120,
    146,229,133,128,63,208,47,242,13,67,112,233,159,183,74,30,
    208,46,12,202,104,134,232,105,70,205,132,99,101,240,96,38,
    192,76,131,44,102,53,143,46,230,181,105,142,134,120,187,123,
    4,161,145,159,176,244,255,144,142,168,177,255,45,81,17,109,
    203,56,40,239,167,81,154,157,176,130,30,21,121,77,240,244,
    9,150,7,40,96,74,12,5,165,221,45,167,179,119,210,165,
    229,17,249,79,62,83,220,236,147,110,140,194,213,46,1,134,
    52,35,132,131,179,45,248,205,34,36,175,109,48,22,1,187,
    176,9,70,132,16,241,188,193,159,13,226,67,19,78,175,118,
    8,141,60,219,7,103,207,216,170,4,215,229,194,170,68,86,
    139,50,109,202,38,251,112,250,153,2,6,24,99,54,184,232,
    50,212,20,221,42,153,197,42,11,140,50,43,116,85,114,209,
    79,130,24,169,169,46,149,13,50,179,154,195,19,156,135,81,
    104,230,126,28,76,217,56,68,8,241,139,117,151,220,203,163,
    94,69,23,222,70,17,192,33,183,35,94,161,93,189,245,187,
    32,191,23,26,137,51,223,15,147,208,248,190,247,153,196,39,
    226,41,126,121,240,229,42,116,25,223,103,218,125,63,78,229,
    44,194,161,247,236,171,219,221,203,21,189,14,238,247,75,145,
    95,149,60,191,138,210,192,193,65,151,64,42,179,6,197,199,
    201,130,95,139,203,26,100,155,196,241,9,178,159,39,21,217,
    28,119,9,88,102,138,142,64,134,24,233,227,52,201,233,52,
    200,85,177,64,174,38,105,49,100,115,158,155,105,229,51,237,
    247,89,99,232,50,152,228,197,233,191,211,25,13,166,166,34,
    74,194,177,74,22,227,80,94,126,140,195,223,148,36,127,17,
    8,131,179,63,225,90,80,68,81,136,11,202,135,235,162,208,
    116,22,92,117,58,7,135,87,71,148,26,131,179,125,74,34,
    76,31,204,29,122,105,142,224,195,46,36,80,88,219,108,93,
    226,76,65,51,28,119,36,245,95,238,132,6,167,52,40,145,
    197,164,156,175,226,195,220,193,202,245,133,86,117,153,91,12,
    185,215,44,98,222,213,84,184,232,173,113,239,158,14,183,203,
    111,198,102,193,214,56,207,171,220,168,253,117,114,113,246,121,
    180,147,199,70,46,177,82,46,202,96,122,62,201,235,39,235,
    113,70,143,168,106,113,62,210,158,249,124,173,24,225,210,61,
    242,234,112,228,233,59,98,67,58,100,141,137,109,98,233,108,
    99,235,32,197,45,225,90,203,196,139,131,48,241,125,77,89,
    176,183,183,231,46,237,92,149,208,217,210,197,57,253,248,127,
    107,50,212,119,139,221,141,2,8,61,215,121,61,155,34,106,
    140,172,247,77,81,132,46,213,92,123,244,78,121,84,149,189,
    181,162,216,97,57,155,77,105,219,188,66,85,114,60,213,200,
    112,86,122,91,212,116,139,21,109,248,41,39,17,47,128,169,
    228,125,71,235,223,82,243,61,53,187,212,60,167,166,121,255,
    117,131,225,251,49,175,84,111,158,210,1,111,176,113,154,142,
    99,139,234,23,255,13,219,177,234,182,141,176,55,68,179,98,
    139,134,181,81,117,48,195,90,162,110,85,44,167,94,119,28,
    81,198,181,26,182,255,1,196,68,67,229,
};

EmbeddedPython embedded_m5_util_terminal(
    "m5/util/terminal.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/python/m5/util/terminal.py",
    "m5.util.terminal",
    data_m5_util_terminal,
    1243,
    2717);

} // anonymous namespace
