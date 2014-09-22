#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_HDLcd[] = {
    120,156,197,88,109,115,27,73,17,238,217,149,100,75,177,98,
    59,206,123,156,88,225,240,69,9,196,186,11,132,80,92,42,
    69,46,166,184,80,196,23,214,20,201,9,138,101,189,59,146,
    87,222,23,213,238,216,137,174,236,47,56,5,124,160,138,31,
    65,241,129,255,193,255,130,238,158,221,213,74,86,184,171,130,
    211,217,210,84,107,94,122,250,229,233,158,158,113,33,251,171,
    226,247,167,45,128,244,188,1,224,225,71,64,0,16,10,232,
    10,16,82,128,183,6,7,85,72,126,8,94,21,222,1,116,
    13,144,6,156,34,97,194,111,13,136,150,120,77,13,2,147,
    123,4,140,26,32,43,208,173,194,171,104,21,42,178,6,7,
    13,72,254,0,66,136,72,192,107,111,1,188,69,120,135,220,
    145,168,51,195,69,240,26,76,212,193,59,199,68,3,70,43,
    32,207,65,23,153,47,64,183,137,172,238,33,171,243,204,234,
    95,196,202,195,145,139,224,53,105,58,202,242,5,205,172,208,
    76,222,227,60,115,89,206,37,91,129,238,106,78,95,40,209,
    107,37,250,98,137,190,84,162,47,151,232,43,37,250,106,137,
    190,86,162,175,151,232,27,37,122,189,68,223,44,209,183,74,
    244,70,137,110,149,232,219,37,250,59,76,47,131,92,133,193,
    7,48,248,46,12,54,161,135,78,90,41,44,241,33,72,19,
    6,119,160,123,7,36,126,62,132,83,244,163,183,90,90,209,
    230,21,23,138,21,119,121,197,61,232,222,3,137,159,187,122,
    69,13,118,219,151,16,27,254,191,241,175,45,144,82,75,216,
    28,201,36,245,227,200,246,163,94,236,27,52,94,163,134,144,
    228,82,179,144,65,234,25,65,234,159,192,120,242,140,12,82,
    39,128,140,5,233,18,24,112,194,196,137,1,163,54,28,11,
    24,84,192,51,225,24,183,169,146,0,125,1,167,6,252,206,
    164,9,39,216,86,16,4,183,160,162,52,158,6,12,2,205,
    105,1,78,170,112,92,133,221,215,199,6,117,28,212,33,249,
    7,124,185,206,76,23,153,169,1,199,216,86,224,180,2,39,
    53,120,133,147,176,107,80,39,245,197,235,99,212,20,123,118,
    219,21,148,118,167,164,46,169,226,249,73,228,132,146,85,183,
    135,78,226,132,246,103,219,191,116,189,118,35,159,17,167,91,
    67,71,237,91,188,196,36,91,132,67,197,172,226,72,170,115,
    72,244,252,200,179,195,216,59,12,164,90,36,62,118,207,15,
    164,109,243,224,243,112,24,39,234,103,73,18,39,22,153,147,
    59,131,216,41,86,144,49,221,32,78,101,155,118,227,109,44,
    98,175,104,118,111,200,28,73,0,22,147,22,123,50,117,19,
    127,168,208,75,154,35,205,38,110,109,242,15,55,233,107,108,
    58,251,113,40,59,168,81,103,59,126,19,209,150,105,167,47,
    195,135,247,83,229,236,5,242,190,227,244,62,250,248,145,116,
    28,239,145,215,217,59,244,3,175,243,212,122,209,25,142,212,
    126,28,117,194,135,29,63,82,18,173,19,116,74,118,217,194,
    225,11,180,195,27,191,111,251,172,155,189,47,131,161,76,154,
    212,123,157,118,23,43,98,73,212,132,41,218,162,137,84,21,
    191,166,88,55,206,137,29,159,180,115,73,99,2,149,153,195,
    232,239,192,14,67,127,31,24,144,172,19,72,6,248,17,228,
    85,132,202,46,141,25,60,246,43,50,139,238,29,152,228,122,
    221,121,204,192,66,132,225,204,199,228,235,8,24,29,85,24,
    212,64,163,6,193,166,97,148,140,168,197,233,196,198,64,230,
    21,72,255,54,201,33,90,1,52,59,38,28,236,186,140,91,
    253,145,129,184,219,38,193,119,24,20,106,223,79,209,168,108,
    122,162,25,63,187,104,147,151,163,207,247,6,210,85,233,6,
    118,124,17,31,182,92,39,138,98,213,114,60,175,229,40,149,
    248,123,135,74,166,45,21,183,54,211,118,157,124,189,154,227,
    170,224,55,26,230,56,34,159,35,142,244,15,207,119,21,254,
    88,227,31,108,255,84,42,196,196,126,236,165,216,79,44,250,
    82,89,36,164,58,143,205,211,124,59,6,95,187,150,67,37,
    149,65,79,53,24,117,78,154,218,188,29,245,51,192,104,245,
    145,19,28,74,69,243,17,41,10,119,37,82,111,52,47,136,
    93,37,37,115,29,201,110,118,20,71,222,8,69,244,221,77,
    218,253,42,3,109,9,8,106,151,16,102,11,216,214,160,137,
    176,91,49,92,210,166,146,129,140,1,118,153,116,7,118,186,
    200,178,5,130,237,20,115,74,219,224,164,192,106,113,232,181,
    136,162,197,22,97,217,186,65,205,58,53,55,115,205,231,160,
    126,115,90,253,7,180,165,193,58,187,102,166,93,17,62,59,
    19,225,115,109,28,62,152,254,118,41,12,12,10,150,113,24,
    152,164,127,242,36,195,60,5,24,186,29,135,75,72,103,171,
    88,43,164,109,45,7,169,69,200,43,195,175,95,130,159,69,
    14,97,236,89,215,222,103,193,141,111,197,130,125,109,193,135,
    180,229,82,134,154,38,163,165,33,92,114,185,145,217,147,109,
    185,141,196,232,10,217,178,108,197,43,120,150,189,138,154,124,
    40,241,193,198,101,136,78,25,218,184,154,168,16,190,122,38,
    92,206,14,155,148,34,124,152,196,111,71,173,184,215,82,144,
    203,240,120,51,221,218,76,63,193,28,208,122,194,89,69,103,
    1,29,231,137,28,38,24,207,117,254,161,99,212,230,120,181,
    179,35,3,173,77,7,55,59,137,109,204,41,41,85,9,101,
    162,121,25,184,81,24,152,228,253,132,246,107,176,117,77,184,
    130,223,134,96,161,236,152,179,33,87,14,60,138,223,79,201,
    206,164,170,4,170,65,173,93,45,50,107,67,122,89,119,38,
    112,242,205,235,98,221,67,230,219,121,132,213,160,64,5,125,
    77,146,150,64,255,103,224,178,74,192,159,128,16,128,142,206,
    194,132,3,146,190,228,200,53,154,254,123,224,68,51,227,60,
    51,116,184,25,89,42,194,104,76,31,241,84,125,188,253,2,
    254,82,202,82,167,38,8,58,138,204,172,112,42,31,69,149,
    34,64,25,58,95,235,184,169,76,70,50,249,103,223,73,105,
    154,142,89,179,136,217,113,194,43,170,30,76,68,115,64,213,
    162,222,201,38,161,158,143,49,69,121,254,134,88,51,74,72,
    249,62,53,247,11,144,136,188,239,155,149,111,99,58,49,151,
    206,37,91,39,195,207,72,136,10,139,189,92,227,243,148,87,
    23,33,80,205,67,224,65,17,2,146,83,243,59,174,165,169,
    53,200,225,167,134,192,11,23,22,33,116,215,169,128,172,66,
    183,70,193,194,245,162,200,98,73,228,121,139,242,221,68,222,
    103,139,236,104,91,21,62,215,238,164,230,237,188,242,4,121,
    244,113,224,132,123,158,243,68,210,110,180,165,155,71,151,145,
    203,191,82,150,159,34,67,188,79,5,254,249,81,174,199,209,
    188,114,196,199,218,91,90,126,142,8,47,118,57,49,252,122,
    95,182,66,25,238,225,197,105,223,31,182,122,129,211,103,255,
    152,153,126,159,231,250,41,118,240,244,41,155,82,246,217,137,
    91,110,28,97,10,63,116,85,156,180,60,137,119,10,233,181,
    238,183,56,255,183,252,180,229,236,225,168,227,42,141,244,201,
    120,229,242,205,73,250,41,87,106,7,111,136,156,167,127,109,
    188,45,250,88,155,246,114,251,232,171,76,145,204,185,28,213,
    97,131,135,33,222,21,212,72,167,174,31,81,115,151,154,77,
    152,115,206,239,0,191,134,64,74,38,171,97,118,169,11,190,
    93,241,132,151,52,55,61,27,178,127,253,58,33,171,95,71,
    178,192,173,209,76,185,64,23,85,106,235,148,242,187,141,188,
    243,28,183,75,220,217,204,59,207,115,187,204,157,43,249,171,
    204,42,119,94,128,238,26,61,19,80,207,69,74,6,11,255,
    107,50,224,72,154,103,12,5,255,215,28,96,61,248,182,196,
    183,126,0,89,97,240,190,248,23,101,221,154,58,254,7,34,
    47,185,203,138,241,45,254,194,36,248,108,55,145,142,146,218,
    73,215,231,167,37,167,16,189,119,84,184,42,87,165,184,114,
    60,42,212,57,229,106,104,116,177,84,11,179,231,196,43,44,
    93,176,94,62,102,141,109,67,151,204,99,44,86,10,197,151,
    177,137,228,27,187,164,188,46,135,73,18,103,56,148,145,55,
    46,117,121,100,94,14,167,244,148,192,184,10,193,186,246,34,
    126,207,6,92,69,39,147,92,55,246,89,181,8,177,57,122,
    143,49,250,38,247,27,191,239,141,211,176,69,94,211,137,183,
    200,185,214,79,10,63,220,154,2,160,140,72,20,219,117,134,
    234,48,145,116,143,250,170,41,88,19,113,170,159,236,86,55,
    166,86,13,253,183,50,192,155,77,236,30,48,215,255,54,78,
    44,41,45,151,250,248,10,90,158,127,20,185,204,103,86,63,
    173,39,212,34,205,238,228,48,243,100,32,149,44,3,78,145,
    85,178,107,173,39,241,164,141,71,120,233,226,27,12,254,14,
    108,123,174,103,211,143,145,249,1,237,66,194,210,217,84,195,
    211,233,18,255,215,43,117,193,7,255,212,219,172,22,237,123,
    144,87,235,163,212,226,44,181,92,184,151,223,17,243,211,151,
    227,151,110,152,59,78,168,223,135,248,225,195,186,77,205,7,
    57,66,56,2,245,13,142,239,72,250,78,138,177,202,85,9,
    23,33,214,22,245,19,194,195,135,91,185,62,91,90,159,223,
    68,238,243,104,120,168,248,37,51,124,200,126,62,59,107,215,
    15,245,147,153,90,157,26,247,18,7,233,75,83,189,169,76,
    124,39,240,191,148,92,140,159,229,247,52,220,115,182,67,103,
    91,30,249,174,84,215,102,206,249,212,73,229,207,125,247,61,
    114,191,12,28,213,139,147,80,221,156,57,252,60,82,201,179,
    24,155,56,224,119,170,25,42,141,82,37,195,51,250,202,232,
    48,180,95,200,48,78,70,47,98,79,170,219,83,227,79,179,
    18,79,79,177,143,36,213,130,170,53,91,203,137,185,239,49,
    45,14,102,166,157,109,170,103,20,78,210,203,230,204,214,150,
    231,108,199,33,185,98,125,90,96,207,75,44,39,234,203,92,
    214,217,98,188,244,227,204,27,179,199,199,222,34,140,230,131,
    28,206,179,92,74,71,233,68,15,215,114,227,243,147,163,56,
    145,125,31,157,144,48,151,137,217,217,9,67,49,198,238,43,
    103,140,242,202,121,70,188,190,37,233,215,157,39,148,238,210,
    46,54,244,182,89,95,174,139,154,65,15,232,166,104,136,166,
    168,136,165,102,221,172,215,234,85,19,179,2,245,172,137,134,
    89,111,44,137,175,254,223,192,220,209,48,54,154,117,241,31,
    251,40,91,101,
};

EmbeddedPython embedded_m5_internal_param_HDLcd(
    "m5/internal/param_HDLcd.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_HDLcd.py",
    "m5.internal.param_HDLcd",
    data_m5_internal_param_HDLcd,
    2356,
    7197);

} // anonymous namespace
