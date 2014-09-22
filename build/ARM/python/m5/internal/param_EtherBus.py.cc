#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_EtherBus[] = {
    120,156,197,88,109,111,227,198,17,158,37,41,217,146,173,179,
    124,182,207,247,226,139,85,20,70,213,32,103,37,215,94,46,
    104,14,135,36,189,0,73,128,56,41,93,192,119,74,81,134,
    38,87,18,101,138,20,200,245,93,20,216,95,234,67,219,15,
    5,250,35,138,126,232,255,232,255,106,103,102,185,52,109,199,
    69,128,164,138,45,46,150,179,203,217,157,153,103,94,118,3,
    40,254,106,248,124,208,1,200,51,1,16,226,79,64,12,48,
    17,208,23,32,164,128,112,13,142,106,144,253,26,194,26,188,
    6,232,91,32,45,56,195,142,13,95,89,144,44,243,55,117,
    136,109,166,8,152,53,65,58,208,175,193,65,178,10,142,172,
    195,81,19,178,175,65,8,145,8,120,30,46,64,184,8,175,
    145,59,118,26,204,112,17,194,38,119,26,16,46,113,167,9,
    179,54,200,37,232,35,243,5,232,183,144,213,155,200,234,6,
    179,250,55,177,10,113,100,29,194,22,77,199,189,188,160,153,
    14,205,228,53,110,48,151,21,179,179,54,244,87,77,255,102,
    165,191,86,233,175,87,250,27,220,95,1,185,10,227,91,48,
    222,132,241,109,24,160,34,218,229,106,119,64,218,48,190,11,
    253,187,32,241,119,7,206,80,87,225,106,229,139,123,252,197,
    205,242,139,45,254,226,62,244,239,131,196,223,150,254,162,14,
    251,221,13,212,127,244,31,252,235,162,254,65,45,99,243,82,
    102,121,148,38,94,148,12,210,200,162,241,58,53,100,173,128,
    154,133,194,108,191,37,179,253,11,216,102,161,85,152,237,20,
    144,177,32,89,98,11,78,185,115,106,193,172,11,39,2,198,
    14,132,54,156,224,50,53,218,192,80,192,153,5,127,176,105,
    194,41,182,14,42,250,13,112,148,182,217,152,21,173,57,45,
    192,105,13,78,106,176,255,252,196,34,194,81,3,178,127,194,
    183,91,204,116,145,153,90,112,130,173,3,103,14,156,214,225,
    0,39,33,105,220,32,241,197,243,19,148,20,41,251,93,7,
    119,187,87,17,151,68,9,163,44,241,39,82,173,96,223,155,
    250,153,63,241,62,86,35,153,125,116,156,119,155,102,82,154,
    239,78,125,53,114,249,43,155,212,49,153,42,230,150,38,82,
    45,97,103,16,37,161,55,73,195,227,88,170,69,98,229,13,
    162,88,122,30,15,126,58,153,166,153,250,56,203,210,204,37,
    141,50,49,78,253,242,11,210,103,16,167,185,236,210,106,188,
    140,75,236,21,205,30,76,153,35,109,128,119,74,31,135,50,
    15,178,104,170,208,80,154,35,205,38,110,93,50,17,55,249,
    87,216,244,70,233,68,246,80,168,222,179,244,85,66,75,230,
    189,161,156,60,122,144,43,255,48,150,15,124,127,240,246,59,
    143,165,239,135,143,195,222,225,113,20,135,189,15,221,207,123,
    211,153,26,165,73,111,242,168,23,37,74,162,130,226,222,69,
    213,236,226,140,155,180,200,171,104,232,69,44,158,55,146,241,
    84,102,45,162,222,165,13,136,182,88,22,117,97,139,174,104,
    97,175,134,143,45,182,172,37,177,23,145,128,1,9,77,208,
    178,13,152,254,1,108,54,180,250,145,5,217,22,65,101,140,
    63,65,182,69,192,236,211,152,197,99,191,35,205,104,234,216,
    38,0,104,226,9,195,11,113,134,51,159,144,197,19,96,140,
    212,96,92,7,141,29,132,156,6,83,54,163,22,167,19,27,
    11,153,59,144,255,253,34,135,164,13,168,121,116,109,36,221,
    194,165,254,196,112,220,239,210,198,247,24,23,106,20,229,168,
    87,214,62,245,217,129,246,81,39,95,206,190,56,28,203,64,
    229,219,72,120,145,30,119,2,63,73,82,213,241,195,176,227,
    43,149,69,135,199,74,230,29,149,118,118,242,110,131,204,189,
    106,160,85,242,155,77,13,148,200,236,8,37,253,18,70,129,
    194,151,53,126,97,253,231,82,33,44,70,105,152,35,157,88,
    12,165,114,105,147,234,6,54,31,154,229,24,127,221,186,65,
    75,46,227,129,106,50,240,252,60,247,120,57,162,51,198,232,
    235,151,126,124,44,21,205,71,176,40,92,149,186,122,161,57,
    162,236,54,201,105,196,36,213,121,73,154,132,51,220,101,20,
    236,208,6,110,51,214,150,129,208,182,129,72,91,192,182,14,
    45,68,94,219,10,72,32,167,192,25,99,236,22,137,15,108,
    119,81,132,13,196,219,25,6,151,174,197,209,129,37,99,7,
    236,80,143,62,118,9,206,238,61,106,182,168,185,111,132,159,
    143,6,90,151,53,240,144,86,181,88,236,192,46,4,44,157,
    104,239,130,19,221,57,119,34,12,133,251,228,12,22,185,204,
    185,51,216,164,130,236,105,129,124,114,51,52,62,14,87,240,
    206,138,113,219,36,112,221,64,213,37,252,85,65,56,172,128,
    208,37,155,48,2,221,59,215,41,113,251,167,82,226,80,43,
    241,17,173,186,92,96,167,197,152,105,138,128,12,111,21,42,
    101,117,62,195,206,108,147,212,89,85,228,38,166,182,131,164,
    197,57,138,243,28,103,126,29,59,180,126,117,199,33,148,13,
    108,184,85,228,158,156,92,125,154,165,223,204,58,233,160,163,
    192,236,225,201,78,190,187,147,191,143,193,160,243,148,195,139,
    14,7,218,225,51,57,205,208,177,27,252,162,157,213,99,199,
    245,138,244,129,10,167,60,206,118,98,53,115,108,202,85,70,
    33,105,142,58,110,150,58,166,45,191,79,75,54,89,193,54,
    108,226,211,20,188,47,47,229,200,200,181,4,143,226,243,17,
    169,154,164,149,64,149,159,187,175,119,205,2,145,104,238,47,
    46,160,101,46,226,184,111,34,255,103,198,213,234,80,98,131,
    30,155,54,76,232,255,11,112,173,37,224,207,64,56,64,115,
    23,254,194,158,73,15,153,115,141,166,255,17,56,232,124,71,
    122,179,180,223,89,69,88,66,183,204,31,243,84,157,237,62,
    131,191,86,34,214,153,13,130,50,147,93,84,83,213,204,228,
    148,158,202,0,250,94,217,199,185,232,210,100,162,145,159,211,
    52,237,188,118,233,188,231,193,175,172,131,48,34,205,7,91,
    139,122,49,143,246,245,233,57,178,40,236,223,19,107,86,5,
    47,111,81,243,160,132,138,48,180,255,251,22,183,47,199,233,
    74,166,242,116,108,252,132,246,225,240,206,87,234,44,147,97,
    80,186,67,205,184,195,195,210,29,36,7,235,215,92,105,83,
    107,145,229,207,44,129,71,30,44,78,232,180,225,128,172,65,
    191,78,142,195,165,164,40,252,74,152,48,70,225,239,66,38,
    96,189,236,105,141,149,198,215,118,165,230,155,57,134,13,82,
    195,147,216,159,28,134,254,211,231,180,32,173,26,24,79,179,
    140,8,237,170,8,228,37,226,58,41,248,245,109,35,202,203,
    57,134,140,119,144,127,41,2,59,72,152,6,28,39,126,63,
    146,157,137,156,28,226,225,106,20,77,59,131,216,31,178,149,
    236,66,196,47,140,136,138,205,124,57,251,230,20,140,246,210,
    78,144,38,24,215,143,3,149,102,157,80,226,161,67,134,157,
    7,29,78,10,157,40,239,248,135,56,234,7,74,163,254,162,
    251,114,113,231,103,195,156,235,184,163,87,212,157,179,149,61,
    60,84,70,88,188,190,48,42,210,199,157,50,194,115,189,170,
    93,8,147,36,30,38,212,76,7,179,119,169,249,37,53,59,
    48,255,68,208,67,254,7,180,16,41,174,142,241,166,33,120,
    167,102,206,151,244,197,119,120,240,223,190,143,7,235,235,138,
    194,143,235,52,83,46,208,169,150,218,6,165,130,126,211,16,
    151,184,93,102,98,203,16,111,112,187,194,196,182,185,38,89,
    101,226,77,232,175,209,157,2,81,214,41,54,44,252,208,216,
    192,94,53,103,127,242,126,212,144,224,62,252,9,37,112,127,
    5,69,217,112,93,56,16,85,241,90,58,28,140,133,169,204,
    171,178,241,169,127,227,10,10,189,32,147,190,146,218,90,119,
    231,42,43,199,21,189,188,95,218,204,8,84,158,79,30,151,
    66,157,113,197,52,91,175,84,205,108,66,113,128,229,13,86,
    214,39,44,183,103,233,226,250,28,151,78,41,62,221,64,36,
    242,149,119,81,5,186,118,166,205,248,211,169,76,194,243,186,
    152,71,230,104,252,119,117,0,40,139,21,44,130,215,241,185,
    234,130,36,82,69,66,54,94,173,116,186,249,154,145,33,59,
    54,6,228,27,194,243,8,237,146,249,116,76,46,195,177,251,
    155,210,32,183,175,226,49,159,74,25,210,161,235,250,65,44,
    151,248,194,129,223,212,230,213,105,225,241,100,202,44,174,27,
    35,14,172,67,124,81,247,174,78,138,211,116,122,232,7,71,
    204,228,127,141,19,35,202,84,134,192,38,100,55,11,101,44,
    149,188,4,53,69,154,40,14,192,161,196,220,155,206,240,108,
    198,167,28,124,143,61,111,222,169,234,61,228,79,231,140,156,
    92,131,82,85,29,147,213,6,255,55,156,134,224,106,224,210,
    165,174,222,221,27,96,42,250,89,238,114,172,90,41,173,202,
    183,143,38,31,179,255,210,89,116,207,159,232,43,37,190,40,
    113,127,70,205,207,13,48,216,253,244,65,143,207,81,250,244,
    138,190,202,165,10,87,38,238,46,209,201,24,147,71,187,70,
    164,221,138,72,207,200,152,22,143,95,51,109,63,154,232,107,
    54,181,122,105,60,204,124,236,111,92,162,230,50,139,252,56,
    250,86,170,251,215,47,91,112,36,161,205,48,179,191,58,101,
    201,0,73,191,115,157,112,33,36,51,48,50,57,140,114,228,
    195,76,42,211,139,96,245,158,209,194,101,215,168,124,58,103,
    20,233,138,92,223,45,60,37,33,243,15,176,161,251,181,198,
    74,67,212,45,186,199,181,69,83,180,132,35,150,91,13,187,
    81,111,212,108,68,26,81,214,68,211,110,52,151,133,249,223,
    70,212,53,173,237,86,67,252,23,68,189,210,61,
};

EmbeddedPython embedded_m5_internal_param_EtherBus(
    "m5/internal/param_EtherBus.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_EtherBus.py",
    "m5.internal.param_EtherBus",
    data_m5_internal_param_EtherBus,
    2173,
    6644);

} // anonymous namespace