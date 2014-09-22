#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_NoncoherentBus[] = {
    120,156,197,88,109,111,220,198,17,158,37,121,39,221,73,103,
    73,150,252,42,217,98,16,8,189,26,177,46,113,170,186,104,
    12,163,113,92,32,41,96,37,229,5,176,115,9,194,80,228,
    222,29,79,60,242,64,174,236,92,32,125,169,140,182,232,151,
    254,136,162,31,250,63,250,191,218,153,89,146,162,100,9,8,
    208,226,78,58,46,150,203,125,155,153,231,153,153,93,31,242,
    191,26,62,191,179,1,178,191,9,128,0,127,2,34,128,177,
    128,158,0,33,5,4,235,112,88,131,244,87,16,212,224,45,
    64,207,0,105,192,41,86,76,248,214,128,120,153,199,212,33,
    50,185,69,192,180,9,210,130,94,13,94,198,107,96,201,58,
    28,54,33,253,1,132,16,177,128,87,193,2,4,139,240,22,
    103,199,74,131,39,92,132,160,201,149,6,4,75,92,105,194,
    116,21,228,18,244,112,242,5,232,181,112,170,7,56,213,53,
    158,234,223,52,85,128,95,54,32,104,81,119,220,203,55,212,
    211,162,158,188,198,53,158,101,165,216,217,42,244,214,138,250,
    245,74,125,189,82,223,168,212,111,84,234,55,43,245,91,92,
    95,1,185,6,163,219,48,186,3,163,187,208,71,5,173,150,
    187,216,4,105,194,104,11,122,91,32,241,183,9,167,168,195,
    96,173,50,226,30,143,184,94,142,184,207,35,182,161,183,13,
    18,127,247,245,136,58,116,219,55,208,46,225,127,240,175,141,
    118,1,181,140,197,107,153,102,97,18,187,97,220,79,66,131,
    190,215,169,32,43,250,84,44,228,230,252,140,204,249,47,96,
    91,6,70,110,206,19,192,137,5,201,18,25,112,194,149,19,
    3,166,109,56,22,48,178,32,48,225,24,151,169,209,6,6,
    2,78,13,248,206,164,14,39,88,90,104,128,251,96,41,109,
    203,17,27,64,207,180,0,39,53,56,174,65,247,213,177,65,
    13,135,13,72,255,9,63,109,241,164,139,60,169,1,199,88,
    90,112,106,193,73,29,94,98,39,108,26,53,72,124,241,234,
    24,37,197,150,110,219,194,221,238,87,196,37,81,130,48,141,
    189,177,84,164,9,119,226,165,222,216,221,79,98,63,25,202,
    84,198,234,217,81,214,110,22,93,147,108,119,226,169,161,195,
    99,77,82,202,120,162,120,206,36,150,106,9,43,253,48,14,
    220,113,18,28,69,82,45,210,132,110,63,140,164,235,242,199,
    47,198,147,36,85,191,79,211,36,117,72,175,220,24,37,94,
    57,130,180,234,71,73,38,219,180,26,47,227,208,244,138,122,
    247,39,60,35,109,128,247,75,131,3,153,249,105,56,81,104,
    46,61,35,245,166,217,218,100,40,46,50,15,139,206,48,25,
    203,14,138,214,121,158,188,137,105,201,172,51,144,227,189,135,
    153,242,14,34,249,208,243,250,31,126,244,88,122,94,240,56,
    232,28,28,133,81,208,249,212,121,209,153,76,213,48,137,59,
    227,189,78,24,43,137,106,138,58,151,41,104,23,251,93,167,
    165,222,132,3,55,100,33,221,161,140,38,50,109,81,235,93,
    218,134,88,21,203,162,46,76,209,22,45,172,213,240,49,197,
    150,177,36,246,67,18,211,39,209,9,102,102,1,172,127,0,
    155,16,17,112,104,64,186,69,176,25,225,79,144,157,17,60,
    93,250,102,240,183,63,146,126,116,235,200,36,48,232,198,99,
    134,26,98,14,123,62,33,235,199,192,120,169,193,168,14,26,
    71,8,63,13,172,116,74,37,118,167,105,12,156,220,130,236,
    239,231,103,136,87,1,245,143,244,199,166,155,184,212,159,24,
    154,221,54,109,124,159,209,161,134,97,134,218,101,27,80,157,
    201,212,69,157,124,53,253,242,96,36,125,149,109,99,195,55,
    201,145,237,123,113,156,40,219,11,2,219,83,42,13,15,142,
    148,204,108,149,216,59,89,187,65,70,95,43,0,86,206,55,
    157,20,128,34,227,35,160,244,75,16,250,10,95,214,249,133,
    245,159,73,133,224,24,38,65,134,237,52,197,64,42,135,54,
    169,174,97,241,105,177,28,163,176,93,47,48,147,201,168,175,
    154,12,63,47,203,92,94,142,218,25,105,52,250,181,23,29,
    73,69,253,17,50,10,87,165,170,94,104,230,88,187,77,210,
    22,194,146,2,221,56,137,131,41,238,53,244,119,104,27,183,
    25,113,203,64,152,187,129,120,91,192,178,14,45,196,223,170,
    225,147,88,86,142,54,70,218,77,82,2,176,245,69,238,72,
    16,117,167,232,110,218,6,251,11,150,143,201,104,83,141,6,
    59,4,106,103,147,138,45,42,238,21,42,152,165,30,90,23,
    245,240,136,214,54,88,120,223,204,197,44,9,181,127,142,80,
    119,206,8,133,46,178,75,196,48,136,62,103,196,48,73,17,
    233,211,156,5,68,57,4,2,126,174,96,159,213,227,172,146,
    216,245,2,182,14,97,177,10,200,65,5,144,14,89,134,209,
    232,220,185,74,149,219,243,85,229,64,171,114,143,214,94,206,
    113,212,98,252,52,133,79,32,48,114,197,178,82,159,99,101,
    122,139,148,90,85,231,45,12,124,47,227,22,71,48,142,130,
    156,47,104,111,162,181,172,43,22,33,174,111,194,205,60,50,
    101,68,254,73,154,252,56,181,147,190,173,160,216,195,147,157,
    108,119,39,251,4,221,131,253,148,29,142,118,16,218,5,164,
    114,146,34,213,27,252,162,233,235,50,149,221,60,172,160,218,
    41,182,177,181,88,217,236,173,50,149,146,147,154,185,166,155,
    165,166,105,227,159,208,194,77,86,179,9,183,240,105,10,222,
    157,155,176,199,228,124,131,191,226,243,140,20,78,50,75,160,
    172,209,233,234,189,179,88,36,160,243,139,115,200,153,161,80,
    206,3,92,229,121,65,190,58,148,56,161,199,164,109,19,31,
    254,2,156,149,9,248,51,16,38,208,244,57,131,152,171,244,
    144,105,215,169,251,247,192,206,232,146,224,103,104,38,26,185,
    187,66,162,102,143,185,171,142,133,127,128,191,86,60,217,169,
    9,130,226,150,153,231,93,213,184,101,149,220,101,48,253,172,
    216,100,157,39,57,25,106,232,101,212,77,211,217,44,233,124,
    230,20,203,92,9,125,212,44,113,182,168,151,116,105,119,95,
    156,161,140,130,194,166,88,55,42,216,249,128,138,135,37,108,
    68,209,54,163,141,110,95,244,226,149,104,230,106,207,249,57,
    237,198,226,253,175,212,57,142,159,159,166,164,73,173,160,201,
    163,146,38,146,29,250,91,206,210,169,52,8,11,167,134,192,
    99,20,38,51,116,130,177,64,214,160,87,39,66,113,2,42,
    114,190,137,194,201,145,115,60,23,45,88,71,251,90,123,37,
    28,180,165,169,248,113,230,78,133,140,253,36,242,198,7,129,
    247,180,71,203,210,218,126,193,64,163,16,100,181,42,8,177,
    71,92,37,11,191,126,88,8,244,122,230,14,229,35,92,165,
    20,132,233,19,36,62,123,145,175,135,210,30,203,241,1,30,
    210,134,225,196,238,71,222,128,45,102,230,130,126,89,8,170,
    216,228,23,163,117,246,128,209,99,251,73,140,17,224,200,87,
    73,106,7,18,143,45,50,176,31,218,28,62,236,48,179,189,
    3,252,234,249,74,179,225,60,185,57,49,244,210,65,198,57,
    224,225,27,170,206,197,226,46,30,81,67,76,127,191,45,20,
    165,143,77,101,44,96,166,104,106,97,80,197,227,136,154,106,
    135,247,107,42,126,73,197,14,204,43,100,116,200,223,210,114,
    164,196,58,250,164,134,80,27,239,48,251,43,26,125,9,191,
    95,252,28,126,235,11,146,156,229,245,226,102,101,1,228,34,
    157,140,123,77,186,110,160,150,37,162,254,194,255,74,125,166,
    203,92,136,226,253,95,25,239,60,154,187,28,206,199,144,231,
    12,87,177,93,84,133,108,105,182,143,68,145,168,87,37,228,
    107,129,205,43,128,229,250,169,244,148,212,246,187,59,7,185,
    217,133,232,77,248,165,21,11,225,202,163,203,227,82,192,83,
    78,157,166,27,149,84,154,141,42,94,98,158,131,233,246,49,
    235,192,53,116,198,125,134,87,171,84,5,157,64,98,249,198,
    189,76,29,58,173,166,45,121,147,137,140,131,179,148,153,191,
    204,28,20,228,167,2,56,203,93,48,63,222,192,231,93,154,
    90,186,99,33,45,155,179,86,18,115,30,134,101,64,135,133,
    73,219,116,138,57,115,204,14,25,84,187,226,210,11,59,191,
    133,194,51,151,144,13,100,36,149,188,212,84,138,70,231,39,
    204,64,98,176,74,166,120,236,225,163,3,190,71,174,59,31,
    175,254,27,92,229,7,200,143,109,228,213,235,232,215,27,86,
    67,112,248,188,112,155,170,119,103,67,145,32,79,51,135,217,
    191,82,42,129,47,252,138,208,197,44,160,99,222,190,55,214,
    247,55,124,31,225,188,71,197,251,133,50,25,190,250,244,196,
    199,18,125,48,68,196,115,108,231,80,238,236,82,59,213,198,
    123,187,133,72,187,90,164,103,94,38,81,22,190,113,28,239,
    177,21,222,237,244,66,142,245,141,22,39,175,239,126,255,44,
    74,252,67,25,228,125,238,93,221,231,121,50,246,176,253,242,
    85,186,97,177,202,218,133,239,65,74,163,110,92,104,205,100,
    26,122,81,248,147,190,40,43,154,249,104,127,94,54,66,73,
    94,231,48,124,137,123,100,96,165,114,16,102,56,11,79,145,
    15,200,29,5,217,89,189,119,149,87,173,14,158,11,14,117,
    66,172,143,255,79,233,162,152,47,42,232,58,172,177,210,16,
    117,131,46,95,77,209,20,45,97,137,229,86,195,108,212,27,
    53,19,177,74,45,235,162,105,54,154,203,162,250,191,141,8,
    110,26,219,205,134,248,47,99,77,219,123,
};

EmbeddedPython embedded_m5_internal_param_NoncoherentBus(
    "m5/internal/param_NoncoherentBus.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_NoncoherentBus.py",
    "m5.internal.param_NoncoherentBus",
    data_m5_internal_param_NoncoherentBus,
    2123,
    6609);

} // anonymous namespace