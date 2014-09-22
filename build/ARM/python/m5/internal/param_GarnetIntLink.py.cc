#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_GarnetIntLink[] = {
    120,156,197,88,91,111,220,198,21,62,67,114,41,237,74,107,
    73,150,100,249,46,58,129,208,109,80,107,19,183,170,139,198,
    48,154,196,69,235,160,85,82,170,128,157,109,26,134,34,103,
    181,92,113,201,5,57,178,178,129,244,82,25,109,223,250,35,
    138,62,244,165,191,162,255,171,61,231,12,73,81,55,192,64,
    139,149,180,28,12,135,51,103,206,229,59,151,153,0,138,191,
    6,62,191,112,0,242,127,9,128,16,127,2,98,128,145,128,
    158,0,33,5,132,203,176,223,128,236,39,16,54,224,45,64,
    207,0,105,192,9,118,76,248,131,1,201,60,175,177,33,54,
    121,68,192,164,5,210,130,94,3,94,37,75,96,73,27,246,
    91,144,125,11,66,136,68,192,235,112,6,194,89,120,139,212,
    177,211,100,130,179,16,182,184,211,132,112,142,59,45,152,44,
    130,156,131,30,18,159,129,94,27,73,125,128,164,110,48,169,
    127,19,169,16,191,172,64,216,166,233,200,203,87,52,211,162,
    153,188,199,13,166,178,80,114,182,8,189,165,178,127,179,214,
    95,174,245,87,106,253,213,90,255,86,173,191,86,235,223,174,
    245,239,212,250,119,185,191,0,114,9,134,247,96,120,31,134,
    15,160,143,74,92,172,56,125,8,210,132,225,58,244,214,65,
    226,239,33,156,160,158,195,165,218,10,135,87,220,172,86,60,
    226,21,239,65,239,61,144,248,123,164,87,216,176,211,89,69,
    219,69,255,193,191,14,218,14,212,60,54,111,100,150,71,105,
    226,69,73,63,141,12,250,110,83,67,150,14,168,153,41,76,
    254,25,153,252,159,192,246,14,141,194,228,199,128,132,5,201,
    18,27,112,204,157,99,3,38,29,56,18,48,180,32,52,225,
    8,183,105,16,3,123,2,78,12,248,218,164,9,199,216,90,
    104,164,135,96,41,109,239,33,27,73,83,154,129,227,6,28,
    53,96,231,245,145,65,3,251,77,200,254,1,223,223,103,162,
    179,76,212,128,35,108,45,56,177,224,216,134,87,56,9,135,
    134,77,18,95,188,62,66,73,113,100,167,99,33,183,219,53,
    113,73,148,48,202,18,127,36,213,10,246,189,177,159,249,35,
    239,87,126,150,72,245,50,81,191,137,146,253,78,171,156,153,
    230,155,99,95,13,92,94,106,146,78,70,99,197,36,83,156,
    62,135,157,126,148,132,222,40,13,15,98,169,102,137,158,215,
    143,98,233,121,252,241,229,104,156,102,234,151,89,150,102,46,
    169,149,7,227,212,175,86,144,82,131,56,205,101,135,118,227,
    109,92,34,175,104,118,127,204,20,137,1,102,151,22,135,50,
    15,178,104,172,208,90,154,34,205,38,106,29,178,19,55,249,
    183,216,116,7,233,72,118,81,178,238,139,244,48,161,45,243,
    238,158,28,109,61,206,149,191,27,203,199,190,223,255,240,163,
    167,210,247,195,167,97,119,247,32,138,195,238,39,238,111,187,
    227,137,26,164,73,119,180,213,141,18,37,81,75,113,247,18,
    253,108,226,180,155,180,211,97,180,231,69,44,163,55,144,241,
    88,102,109,26,189,75,92,136,69,49,47,108,97,138,142,104,
    99,175,129,143,41,238,27,115,98,59,34,41,3,146,156,64,
    102,150,176,250,59,176,1,209,254,251,6,100,247,9,52,67,
    252,9,178,50,66,103,135,190,25,252,237,119,164,30,61,58,
    52,9,10,122,240,136,129,134,136,195,153,207,200,246,9,48,
    90,26,48,180,65,163,8,193,167,97,149,77,168,197,233,68,
    198,64,226,22,228,127,59,75,33,89,4,84,63,6,8,28,
    186,133,91,253,137,129,185,211,33,198,183,25,28,106,16,229,
    168,92,54,1,245,217,149,118,80,39,95,78,190,216,29,202,
    64,229,235,56,240,85,122,224,4,126,146,164,202,241,195,208,
    241,149,202,162,221,3,37,115,71,165,206,70,222,105,146,205,
    151,74,124,85,244,38,227,18,79,100,123,196,147,126,9,163,
    64,225,203,50,191,176,254,115,169,16,27,131,52,204,113,156,
    72,236,73,229,18,147,234,6,54,159,148,219,49,8,59,118,
    9,153,92,198,125,213,98,244,249,121,238,241,118,52,206,64,
    163,213,111,252,248,64,42,154,143,136,81,184,43,117,245,70,
    211,134,218,109,18,182,148,149,244,231,37,105,18,78,144,213,
    40,216,32,46,110,51,224,230,129,32,183,138,112,155,193,214,
    134,54,194,111,209,8,72,42,171,0,27,3,237,22,233,0,
    216,248,162,136,34,8,186,19,140,53,29,131,131,5,139,199,
    174,232,80,143,22,187,132,105,247,30,53,247,169,121,80,106,
    96,138,106,104,159,87,195,19,218,218,96,217,3,179,144,178,
    114,167,237,51,238,116,231,212,157,48,60,238,144,91,24,228,
    60,167,110,97,146,30,178,231,133,15,144,195,33,12,240,115,
    13,249,172,29,119,145,164,182,75,208,186,132,196,58,28,247,
    106,112,116,201,48,140,69,247,206,85,154,92,191,86,77,238,
    105,77,110,209,214,243,5,138,218,140,158,150,8,8,2,70,
    161,87,214,233,11,236,76,214,72,167,117,109,174,97,206,123,
    149,180,57,121,113,2,228,114,66,135,18,173,100,221,177,8,
    111,125,19,110,21,73,41,39,207,31,103,233,119,19,39,237,
    59,10,74,30,158,109,228,155,27,249,199,24,27,156,231,28,
    109,116,116,208,254,159,201,113,134,126,222,228,23,237,187,30,
    251,177,87,164,20,212,58,37,120,54,22,235,154,67,85,174,
    50,138,80,211,86,116,171,82,52,241,253,49,237,219,98,45,
    155,176,134,79,75,48,115,94,202,209,146,43,13,254,138,207,
    167,164,111,18,89,2,213,148,238,142,102,157,165,34,249,220,
    31,156,193,205,244,100,114,63,192,77,94,148,158,103,67,133,
    18,122,76,226,154,156,225,47,192,229,152,128,63,3,33,2,
    13,95,184,15,59,42,61,100,216,101,154,254,13,112,32,186,
    36,239,25,218,13,141,34,84,161,151,230,79,121,170,78,131,
    159,195,95,107,81,236,196,4,65,41,203,44,10,174,122,202,
    178,42,199,101,40,189,83,90,178,206,122,56,217,105,224,231,
    52,77,251,178,89,249,242,105,64,172,170,36,12,80,83,68,
    217,172,222,209,35,230,94,158,98,140,242,193,61,177,108,212,
    144,243,35,106,30,87,160,17,229,216,116,248,92,135,171,243,
    152,167,131,230,175,137,25,139,217,95,176,57,78,157,161,82,
    185,72,163,116,145,39,149,139,72,14,229,111,185,54,167,214,
    32,32,156,24,2,15,88,88,196,208,217,198,2,217,128,158,
    77,206,196,117,167,40,124,77,148,241,141,246,59,147,39,88,
    67,219,90,119,21,22,180,153,169,249,110,218,241,132,44,253,
    44,246,71,187,161,255,252,143,180,43,109,29,148,222,103,148,
    114,44,214,229,32,207,17,87,137,194,175,31,150,242,188,153,
    118,44,249,8,55,169,228,96,207,9,211,128,3,200,239,7,
    210,25,201,209,46,30,204,6,209,216,233,199,254,30,219,203,
    44,228,252,162,148,83,177,193,207,103,233,156,162,212,118,234,
    4,105,130,161,255,32,80,105,230,132,18,207,42,50,116,30,
    59,156,55,156,40,119,252,93,252,234,7,74,123,194,89,191,
    230,114,208,207,246,114,174,252,246,15,169,123,29,246,246,240,
    84,26,97,205,251,77,169,39,125,84,170,146,0,151,185,218,
    173,48,153,226,25,68,77,116,168,251,41,53,63,164,102,3,
    174,41,87,116,113,147,175,105,55,82,161,141,209,168,41,184,
    86,58,51,241,75,90,155,95,116,237,221,119,113,109,125,107,
    82,56,184,77,51,229,12,29,144,169,109,82,202,232,181,202,
    59,152,57,30,156,167,11,143,208,230,145,27,20,10,102,254,
    215,80,192,254,115,29,158,19,254,95,35,128,251,228,186,197,
    112,127,12,69,249,112,149,247,139,186,140,109,237,253,67,81,
    22,236,117,1,249,110,224,238,229,80,243,130,76,250,74,106,
    227,221,157,190,212,28,80,52,15,253,202,132,165,104,213,1,
    230,105,37,222,9,215,80,147,149,90,69,205,22,21,175,176,
    224,193,170,251,136,53,224,25,186,240,62,197,170,85,41,98,
    13,155,68,30,122,151,40,67,23,215,196,145,63,30,203,36,
    60,45,156,249,203,180,1,65,65,107,0,167,53,12,86,201,
    43,248,92,116,80,75,35,167,148,149,109,217,168,92,242,26,
    172,202,88,30,149,246,236,44,64,61,72,187,100,77,29,150,
    171,136,236,254,188,178,207,198,21,64,197,129,195,52,219,247,
    98,28,202,233,176,246,142,51,177,190,226,226,233,204,40,235,
    150,125,34,148,177,84,242,50,52,40,98,177,56,201,134,18,
    147,99,58,193,243,21,31,82,240,61,246,188,107,73,35,63,
    3,125,15,155,83,170,163,52,98,99,34,89,21,77,171,41,
    56,95,159,187,178,213,236,189,15,101,49,62,201,93,14,47,
    11,149,190,249,90,177,76,150,236,104,116,160,220,246,71,250,
    154,136,239,61,220,71,212,188,95,154,140,93,68,31,212,248,
    8,164,143,160,232,84,92,76,112,237,224,110,210,248,67,130,
    193,214,102,41,211,230,182,182,1,201,226,189,145,84,137,240,
    229,230,104,75,61,56,55,83,75,95,155,207,21,243,197,25,
    159,197,105,176,47,67,125,199,118,5,21,158,243,34,29,249,
    56,126,239,210,25,59,209,168,160,176,116,238,123,152,209,170,
    213,115,163,185,204,34,63,142,190,151,23,36,212,244,62,245,
    243,40,40,140,118,5,83,60,197,77,241,148,149,93,193,20,
    207,96,10,100,162,242,35,95,182,94,178,11,221,22,212,7,
    184,222,184,24,245,25,206,153,220,139,114,218,248,230,185,69,
    69,24,36,140,41,231,10,239,170,19,184,14,15,208,133,191,
    190,224,120,78,215,224,249,231,216,208,117,95,115,161,41,108,
    131,238,150,77,209,18,109,97,137,249,118,211,108,218,205,134,
    137,94,66,35,203,162,101,54,91,243,226,226,255,58,250,79,
    203,88,159,107,138,255,2,162,98,38,59,
};

EmbeddedPython embedded_m5_internal_param_GarnetIntLink(
    "m5/internal/param_GarnetIntLink.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_GarnetIntLink.py",
    "m5.internal.param_GarnetIntLink",
    data_m5_internal_param_GarnetIntLink,
    2203,
    6872);

} // anonymous namespace