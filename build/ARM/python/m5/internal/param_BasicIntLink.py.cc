#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BasicIntLink[] = {
    120,156,197,88,91,111,219,200,21,62,67,82,178,37,75,177,
    28,199,185,217,137,89,20,70,213,69,99,237,166,117,83,116,
    131,160,187,155,2,155,162,245,110,169,2,201,106,139,114,105,
    114,36,83,166,72,129,28,199,171,133,253,82,7,109,209,151,
    254,136,162,15,253,31,253,95,237,57,103,72,154,190,117,23,
    104,33,219,226,96,56,156,57,115,46,223,185,204,248,144,255,
    213,240,249,133,13,144,253,85,0,4,248,19,16,1,76,4,
    12,4,8,41,32,88,133,131,26,164,63,129,160,6,239,0,
    6,6,72,3,78,177,99,194,151,6,196,45,94,83,135,200,
    228,17,1,179,38,72,11,6,53,120,29,175,128,37,235,112,
    208,132,244,43,16,66,196,2,222,4,11,16,44,194,59,164,
    142,157,6,19,92,132,160,201,157,6,4,75,220,105,194,172,
    3,114,9,6,72,124,1,6,109,36,245,30,146,186,197,164,
    254,69,164,2,252,114,7,130,54,77,71,94,190,160,153,22,
    205,228,61,110,49,149,229,130,179,14,12,86,138,254,237,74,
    127,181,210,191,83,233,175,85,250,119,43,253,123,220,95,6,
    185,2,227,251,48,126,0,227,135,48,68,5,117,74,46,214,
    65,154,48,222,128,193,6,72,252,173,195,41,234,48,88,169,
    172,120,196,43,110,151,43,30,243,138,77,24,108,130,196,223,
    99,189,162,14,253,238,26,218,37,252,55,254,117,209,46,160,
    90,216,188,149,105,22,38,177,27,198,195,36,52,232,123,157,
    26,178,162,79,205,66,110,206,79,200,156,255,4,182,101,96,
    228,230,60,1,36,44,72,150,200,128,19,238,156,24,48,235,
    194,177,128,177,5,129,9,199,184,77,141,24,24,9,56,53,
    224,247,38,77,56,193,214,66,3,60,6,75,105,91,142,217,
    0,154,210,2,156,212,224,184,6,253,55,199,6,13,28,52,
    32,253,7,124,179,193,68,23,153,168,1,199,216,90,112,106,
    193,73,29,94,227,36,28,26,55,72,124,241,230,24,37,197,
    145,126,215,66,110,119,43,226,146,40,65,152,198,222,68,170,
    85,236,187,83,47,245,38,238,199,94,22,250,175,98,245,235,
    48,62,232,54,139,137,73,182,61,245,212,190,195,43,77,82,
    201,100,170,152,98,18,75,181,132,157,97,24,7,238,36,9,
    14,35,169,22,137,156,59,12,35,233,186,252,241,213,100,154,
    164,234,151,105,154,164,14,105,149,7,163,196,43,87,144,78,
    253,40,201,100,151,118,227,109,28,34,175,104,246,112,202,20,
    137,1,230,150,22,7,50,243,211,112,170,208,88,154,34,205,
    38,106,93,50,19,55,153,139,77,111,63,153,200,30,10,214,
    123,153,28,197,180,101,214,27,201,201,206,147,76,121,123,145,
    124,226,121,195,247,63,120,38,61,47,120,22,244,246,14,195,
    40,232,125,228,252,166,55,157,169,253,36,238,77,118,122,97,
    172,36,42,41,234,93,86,207,54,206,186,77,27,29,133,35,
    55,100,17,221,125,25,77,101,218,166,209,135,196,132,232,136,
    150,168,11,83,116,69,27,123,53,124,76,177,97,44,137,221,
    144,132,244,73,112,130,152,89,128,234,239,192,230,67,235,31,
    24,144,110,16,100,198,248,19,100,99,4,78,159,190,25,252,
    237,183,164,29,61,58,54,9,8,122,240,152,97,134,120,195,
    153,207,201,242,49,48,86,106,48,174,131,198,16,66,79,131,
    42,157,81,139,211,137,140,129,196,45,200,254,118,158,66,220,
    1,212,62,186,62,14,221,197,173,254,200,176,236,119,137,241,
    93,198,134,218,15,51,212,45,91,128,250,236,72,125,212,201,
    231,179,207,246,198,210,87,217,38,14,124,145,28,218,190,23,
    199,137,178,189,32,176,61,165,210,112,239,80,201,204,86,137,
    189,149,117,27,100,242,149,2,94,37,189,217,180,128,19,153,
    30,225,164,95,130,208,87,248,194,184,117,89,255,153,84,8,
    141,253,36,200,112,156,72,140,164,114,136,73,117,11,155,143,
    138,237,24,131,221,122,129,152,76,70,67,213,100,240,121,89,
    230,242,118,52,206,56,163,213,111,189,232,80,42,154,143,128,
    81,184,43,117,245,70,115,70,218,125,146,181,16,149,212,231,
    198,73,28,204,144,211,208,223,34,38,238,51,222,90,64,136,
    91,67,180,45,96,91,135,54,162,175,99,248,36,148,149,99,
    141,113,118,151,84,0,108,123,145,135,16,196,220,41,6,154,
    174,193,145,130,165,99,71,180,169,71,139,29,130,180,179,78,
    205,6,53,143,10,5,204,79,11,237,139,90,120,74,59,27,
    44,186,111,230,66,150,206,180,123,206,153,30,156,57,19,134,
    198,62,57,133,65,174,115,230,20,38,169,33,125,145,123,0,
    185,27,130,0,63,87,112,207,202,113,58,36,116,189,128,172,
    67,56,172,130,113,84,1,163,67,118,97,36,58,15,174,83,
    228,230,77,42,114,164,21,185,67,59,183,114,12,181,25,59,
    77,225,19,0,140,92,173,172,210,151,216,153,221,35,149,86,
    149,121,15,211,221,235,184,205,121,139,115,31,87,9,58,142,
    104,29,235,142,69,104,27,154,112,55,207,71,25,185,253,52,
    77,190,158,217,201,208,86,80,240,240,124,43,219,222,202,62,
    196,192,96,191,224,80,163,67,131,118,254,84,78,83,116,242,
    6,191,104,199,117,217,137,221,60,157,160,210,41,183,179,173,
    88,213,28,167,50,149,82,120,154,179,158,155,165,158,137,237,
    15,105,219,38,43,217,132,123,248,52,5,243,230,38,28,41,
    185,198,224,175,248,124,76,234,38,137,37,80,165,232,244,53,
    231,44,20,137,231,252,224,28,106,230,38,146,243,30,238,241,
    178,112,187,58,148,24,161,199,36,166,201,19,254,12,92,135,
    9,248,19,16,30,208,236,185,239,176,151,210,67,102,93,165,
    233,127,0,14,66,87,164,60,67,251,160,145,135,41,116,209,
    236,25,79,213,25,240,87,240,151,74,4,59,53,65,80,182,
    50,243,74,171,154,173,172,210,107,25,72,223,41,35,89,231,
    221,155,204,180,239,101,52,77,59,178,89,58,242,89,48,44,
    235,35,140,78,243,195,216,162,222,208,37,222,94,157,33,140,
    82,193,186,88,53,42,184,249,17,53,79,74,200,136,98,108,
    46,108,110,94,140,221,149,12,230,234,120,249,41,241,98,49,
    247,203,117,46,36,170,68,74,247,168,21,238,241,180,116,15,
    201,65,252,29,87,228,212,26,132,130,83,67,224,145,9,139,
    23,58,173,88,32,107,48,168,147,35,113,185,41,114,63,19,
    69,104,163,144,120,46,67,176,126,118,181,230,74,32,104,27,
    83,243,245,156,67,9,153,249,121,228,77,246,2,239,197,128,
    54,165,157,253,194,243,140,66,140,78,85,12,242,26,113,157,
    36,252,250,126,33,206,219,57,135,145,15,112,143,82,12,118,
    154,32,241,57,118,252,110,95,218,19,57,217,195,195,216,126,
    56,181,135,145,55,98,107,153,185,152,159,21,98,42,54,247,
    197,236,156,81,128,218,77,108,63,137,49,230,31,250,42,73,
    237,64,226,1,69,6,246,19,155,19,134,29,102,182,183,135,
    95,61,95,105,47,56,239,210,92,4,122,233,40,227,122,239,
    224,136,186,55,96,109,23,15,162,33,22,186,95,22,106,210,
    199,163,50,250,115,109,171,93,10,147,40,30,60,212,76,7,
    185,159,82,243,67,106,182,224,102,146,68,143,34,44,109,70,
    10,172,99,28,106,8,62,44,85,231,125,78,43,179,203,94,
    125,244,93,188,90,95,129,228,190,93,167,153,114,129,78,196,
    212,54,40,85,12,154,197,224,18,183,45,30,108,23,183,44,
    183,120,112,25,6,29,186,122,160,145,21,10,13,11,255,107,
    104,96,135,186,1,87,242,254,175,17,193,121,122,195,82,56,
    63,134,188,146,184,46,26,136,170,136,109,29,13,198,162,40,
    220,171,242,241,5,193,131,43,193,231,250,169,244,148,212,150,
    123,56,119,153,57,188,104,22,252,210,126,133,96,229,49,230,
    89,41,220,41,23,83,179,59,149,194,154,205,41,94,99,229,
    131,197,247,49,203,239,26,186,254,62,195,169,85,170,129,206,
    125,177,60,114,47,171,66,151,216,196,144,55,157,202,56,56,
    43,159,249,203,156,193,64,17,108,8,103,181,12,214,202,119,
    240,185,236,154,36,90,69,82,54,100,173,116,198,249,155,148,
    97,124,80,24,179,75,49,239,44,96,59,100,74,29,162,203,
    232,236,252,188,52,206,163,171,49,26,39,129,116,247,232,160,
    246,45,51,176,174,98,3,234,215,255,58,217,251,86,114,222,
    121,114,30,155,132,221,40,144,145,84,242,10,8,41,146,44,
    63,3,7,18,211,107,50,195,163,25,31,112,240,61,114,221,
    155,200,68,63,195,61,190,130,60,12,82,38,170,99,46,90,
    195,255,134,213,16,156,242,47,220,244,106,254,108,40,74,249,
    89,230,112,68,90,46,237,196,215,145,69,194,101,239,164,195,
    232,174,55,209,247,75,124,99,226,124,143,154,239,23,166,102,
    69,234,83,30,31,160,244,241,21,61,145,235,17,46,63,156,
    237,2,3,147,157,237,66,168,237,138,80,78,130,199,147,148,
    239,68,39,59,92,78,95,158,248,73,148,248,7,50,208,55,
    111,215,16,227,57,47,147,137,135,227,235,87,206,232,135,147,
    156,194,202,133,239,65,74,171,214,46,140,102,50,13,189,40,
    252,70,94,67,143,217,39,131,176,234,138,143,138,52,122,113,
    66,163,128,36,189,113,25,113,41,116,51,192,82,57,10,51,
    82,199,114,117,65,30,202,200,230,172,160,43,192,93,93,124,
    3,120,212,101,188,190,168,120,65,87,217,124,165,66,151,118,
    141,229,134,168,27,116,65,108,138,166,104,11,75,180,218,13,
    179,81,111,212,76,196,44,141,172,138,166,217,104,182,68,245,
    127,19,81,220,52,54,91,13,241,31,231,149,254,206,
};

EmbeddedPython embedded_m5_internal_param_BasicIntLink(
    "m5/internal/param_BasicIntLink.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_BasicIntLink.py",
    "m5.internal.param_BasicIntLink",
    data_m5_internal_param_BasicIntLink,
    2174,
    6769);

} // anonymous namespace