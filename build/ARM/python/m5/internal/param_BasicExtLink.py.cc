#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BasicExtLink[] = {
    120,156,197,88,109,111,219,200,17,158,37,41,217,146,165,216,
    142,227,188,58,49,211,194,168,122,104,172,187,180,105,14,189,
    32,232,93,114,64,175,104,125,87,170,64,114,186,107,89,154,
    92,73,148,41,82,32,215,113,116,176,191,212,65,219,111,253,
    15,45,250,161,255,163,255,171,157,153,37,105,250,173,119,64,
    11,217,22,23,203,229,236,236,206,204,51,47,187,62,228,127,
    53,124,126,110,3,100,127,19,0,1,254,4,68,0,19,1,
    125,1,66,10,8,214,96,175,6,233,79,32,168,193,59,128,
    190,1,210,128,99,236,152,240,149,1,113,139,231,212,33,50,
    121,68,192,172,9,210,130,126,13,94,197,171,96,201,58,236,
    53,33,253,3,8,33,98,1,175,131,5,8,22,225,29,114,
    199,78,131,25,46,66,208,228,78,3,130,37,238,52,97,182,
    2,114,9,250,200,124,1,250,109,100,245,30,178,186,198,172,
    254,69,172,2,252,114,3,130,54,145,227,94,190,36,74,139,
    40,121,141,107,204,101,185,216,217,10,244,87,139,254,245,74,
    127,173,210,191,81,233,175,87,250,55,43,253,91,149,254,237,
    74,255,14,247,151,65,174,194,248,46,140,239,193,120,3,6,
    168,184,149,114,119,247,65,154,48,126,0,253,7,32,241,119,
    31,142,81,183,193,106,101,198,38,207,184,94,206,176,121,198,
    67,232,63,4,137,63,91,207,168,67,175,179,142,246,10,255,
    141,127,29,180,23,168,22,54,111,100,154,133,73,236,134,241,
    32,9,13,250,94,167,134,172,235,83,179,144,155,249,5,153,
    249,159,192,54,14,140,220,204,71,128,140,5,201,18,25,112,
    196,157,35,3,102,29,56,20,48,182,32,48,225,16,151,169,
    209,6,134,2,142,13,248,218,36,130,35,108,45,52,204,3,
    176,148,182,241,152,13,163,57,45,192,81,13,14,107,208,123,
    125,104,208,192,94,3,210,127,192,55,27,204,116,145,153,26,
    112,136,173,5,199,22,28,213,225,21,18,225,208,184,65,226,
    139,215,135,40,41,142,244,58,22,238,118,167,34,46,137,18,
    132,105,236,77,164,90,195,190,59,245,82,111,226,126,226,101,
    161,255,233,91,245,171,48,222,235,52,11,194,36,219,158,122,
    106,228,240,76,147,84,50,153,42,230,152,196,82,45,97,103,
    16,198,129,59,73,130,253,72,170,69,98,231,14,194,72,186,
    46,127,252,108,50,77,82,245,105,154,38,169,67,90,229,193,
    40,241,202,25,164,83,63,74,50,217,161,213,120,25,135,216,
    43,162,30,76,153,35,109,128,119,75,147,3,153,249,105,56,
    85,104,44,205,145,168,137,91,135,204,196,77,230,98,211,29,
    37,19,217,69,193,186,47,147,131,152,150,204,186,67,57,121,
    242,40,83,222,110,36,31,121,222,224,253,15,158,74,207,11,
    158,6,221,221,253,48,10,186,31,59,191,238,78,103,106,148,
    196,221,201,147,110,24,43,137,74,138,186,231,213,179,141,84,
    215,105,161,131,112,232,134,44,162,59,146,209,84,166,109,26,
    189,75,155,16,43,162,37,234,194,20,29,209,198,94,13,31,
    83,108,24,75,98,39,36,33,125,18,156,32,102,22,160,250,
    59,176,249,208,250,123,6,164,27,4,153,49,254,4,217,24,
    129,211,163,111,6,127,251,13,105,71,143,142,77,2,130,30,
    60,100,152,33,222,144,242,25,89,62,6,198,74,13,198,117,
    208,24,66,232,105,80,165,51,106,145,156,216,24,200,220,130,
    236,175,167,57,196,43,128,218,199,144,128,67,55,113,169,63,
    50,44,123,29,218,248,14,99,67,141,194,12,117,203,22,160,
    62,59,82,15,117,242,197,236,243,221,177,244,85,182,137,3,
    95,38,251,182,239,197,113,162,108,47,8,108,79,169,52,220,
    221,87,50,179,85,98,111,101,157,6,153,124,181,128,87,201,
    111,54,45,224,68,166,71,56,233,151,32,244,21,190,48,110,
    93,214,127,38,21,66,99,148,4,25,142,19,139,161,84,14,
    109,82,93,195,230,227,98,57,198,96,167,94,32,38,147,209,
    64,53,25,124,94,150,185,188,28,141,51,206,104,246,27,47,
    218,151,138,232,17,48,10,87,165,174,94,104,206,72,187,77,
    178,22,162,146,250,220,56,137,131,25,238,52,244,183,104,19,
    183,25,111,45,32,196,173,35,218,22,176,173,67,27,209,183,
    98,248,36,148,149,99,141,113,118,147,84,0,108,123,145,135,
    16,196,220,49,6,154,142,193,145,130,165,99,71,180,169,71,
    147,29,130,180,115,143,154,13,106,238,23,10,152,159,22,218,
    103,181,240,152,86,54,88,116,223,204,133,44,157,105,231,148,
    51,221,57,113,38,12,141,61,114,10,131,92,231,196,41,76,
    82,67,250,60,247,0,114,55,4,1,126,174,224,158,149,227,
    172,144,208,245,2,178,14,225,176,10,198,97,5,140,14,217,
    133,145,232,220,185,76,145,155,87,169,200,161,86,228,19,90,
    185,149,99,168,205,216,105,10,159,0,96,228,106,101,149,190,
    196,206,236,22,169,180,170,204,91,152,238,94,197,109,206,91,
    156,251,184,122,208,113,68,235,88,119,44,66,219,192,132,155,
    121,62,202,200,237,167,105,242,118,102,39,3,91,65,177,135,
    103,91,217,246,86,246,17,6,6,251,57,135,26,29,26,180,
    243,167,114,154,162,147,55,248,69,59,174,203,78,236,230,233,
    4,149,78,185,157,109,197,170,230,56,149,169,148,194,211,156,
    245,220,44,245,76,219,254,136,150,109,178,146,77,184,133,79,
    83,240,222,220,132,35,37,215,24,252,21,159,79,72,221,36,
    177,4,170,32,157,158,222,57,11,69,226,57,63,56,133,154,
    185,137,228,188,135,107,188,44,220,174,14,37,70,232,49,105,
    211,228,9,127,6,174,195,4,252,9,8,15,104,246,220,119,
    216,75,233,33,179,174,17,249,239,129,131,208,5,41,207,208,
    62,104,228,97,10,93,52,123,202,164,58,3,254,18,254,82,
    137,96,199,38,8,202,86,102,94,105,85,179,149,85,122,45,
    3,233,59,101,36,235,180,123,147,153,70,94,70,100,218,145,
    205,210,145,79,130,97,89,31,97,116,154,31,198,22,245,130,
    46,237,237,179,19,132,81,42,184,39,214,140,10,110,126,68,
    205,163,18,50,162,24,155,203,54,55,207,198,238,74,6,115,
    117,188,252,5,237,197,226,221,47,215,185,144,168,50,41,221,
    163,86,184,199,227,210,61,36,7,241,119,92,145,83,107,16,
    10,142,13,129,71,41,44,94,232,20,99,129,172,65,191,78,
    142,196,229,166,200,253,76,20,161,141,66,226,169,12,193,250,
    217,209,154,43,129,160,109,76,205,219,57,135,18,50,243,179,
    200,155,236,6,222,243,175,105,81,90,217,47,60,207,40,196,
    88,169,138,65,94,35,46,147,132,95,223,47,196,121,51,231,
    48,242,1,174,81,138,193,78,19,36,62,199,142,223,142,164,
    61,145,147,93,60,140,141,194,169,61,136,188,33,91,203,204,
    197,252,188,16,83,177,185,207,102,231,140,2,212,78,98,251,
    73,140,49,127,223,87,73,106,7,18,15,40,50,176,31,217,
    156,48,236,48,179,189,93,252,234,249,74,123,193,105,151,230,
    34,208,75,135,25,215,123,123,7,212,189,2,107,187,120,16,
    13,177,208,253,93,161,38,125,60,42,163,63,215,182,218,165,
    48,137,226,193,67,205,116,144,251,41,53,63,164,102,11,174,
    38,73,116,113,141,175,104,49,82,96,29,227,80,67,240,97,
    169,74,247,5,205,204,206,123,245,193,119,241,106,125,53,146,
    251,118,157,40,229,2,157,136,169,109,80,170,232,55,139,193,
    37,110,91,60,216,46,110,95,174,241,224,50,244,87,232,234,
    129,70,86,41,52,44,252,175,161,129,29,234,10,92,201,255,
    191,70,4,231,241,21,75,225,252,24,242,74,226,178,104,32,
    170,34,182,117,52,24,139,162,112,175,202,199,23,4,119,46,
    4,159,235,167,210,83,82,91,238,238,220,101,230,240,162,183,
    32,75,251,21,130,149,199,152,167,165,112,199,92,76,205,110,
    84,10,107,54,167,120,133,149,15,22,223,135,44,191,107,232,
    250,251,4,167,86,169,6,58,247,197,242,192,61,175,10,93,
    98,211,134,188,233,84,198,193,73,249,204,95,230,12,6,138,
    96,35,56,169,101,176,86,190,129,207,121,215,36,209,42,146,
    178,33,107,165,51,206,223,164,12,227,73,97,204,14,197,188,
    147,128,237,144,41,117,136,46,163,179,243,179,210,56,155,23,
    99,84,190,85,88,42,5,146,142,106,223,74,131,181,21,231,
    142,98,224,178,9,40,197,183,50,45,105,10,166,197,0,155,
    135,93,42,144,145,84,242,2,56,41,146,50,63,15,7,18,
    83,109,50,195,99,26,31,118,240,61,114,221,171,200,74,31,
    226,26,187,144,135,68,202,74,117,204,75,235,248,223,176,26,
    130,211,255,153,91,95,189,191,239,65,81,214,207,50,135,163,
    211,114,105,51,190,154,44,146,47,123,42,29,76,119,188,137,
    190,107,226,219,19,231,33,53,223,47,204,206,78,166,79,124,
    124,152,210,71,89,244,74,174,77,184,20,113,182,105,156,230,
    78,158,108,23,66,109,107,161,156,253,221,217,139,36,70,141,
    70,17,174,106,48,145,218,184,148,182,55,203,148,156,176,141,
    207,19,188,136,18,127,79,6,250,166,78,221,191,156,230,101,
    50,241,112,252,222,133,20,189,112,146,115,88,61,243,61,72,
    105,214,250,153,209,76,166,161,23,133,223,200,75,86,100,187,
    57,9,158,198,210,75,86,100,10,50,107,129,75,254,168,200,
    46,103,9,26,5,184,233,141,11,147,115,201,128,97,154,202,
    97,152,209,138,203,213,9,121,112,252,240,191,248,103,117,242,
    21,160,90,31,12,244,213,199,115,186,28,207,232,24,69,215,
    128,141,229,134,168,27,116,229,108,138,166,104,11,75,180,218,
    13,179,81,111,212,76,68,62,141,172,137,166,217,104,182,196,
    217,255,77,244,135,166,177,217,106,136,255,0,124,67,33,188,
};

EmbeddedPython embedded_m5_internal_param_BasicExtLink(
    "m5/internal/param_BasicExtLink.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_BasicExtLink.py",
    "m5.internal.param_BasicExtLink",
    data_m5_internal_param_BasicExtLink,
    2208,
    6879);

} // anonymous namespace