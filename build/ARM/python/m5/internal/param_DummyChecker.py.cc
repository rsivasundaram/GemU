#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_DummyChecker[] = {
    120,156,197,88,109,111,219,200,17,158,37,37,217,146,173,216,
    142,243,30,231,172,220,213,61,93,238,108,221,165,245,165,232,
    5,65,157,184,64,83,92,124,41,149,194,57,181,40,75,139,
    43,137,178,72,10,228,42,137,14,246,151,58,104,251,173,63,
    162,232,135,254,143,254,175,235,204,44,73,211,150,5,4,104,
    33,219,226,98,184,47,179,59,51,207,188,44,219,144,252,21,
    241,249,85,13,32,126,98,0,184,248,19,48,0,240,5,180,
    4,8,41,192,93,133,195,34,68,63,7,183,8,239,1,90,
    6,72,3,78,144,48,225,15,6,4,139,188,166,4,3,147,
    123,4,140,43,32,11,208,42,194,126,176,2,5,89,130,195,
    10,68,127,6,33,68,32,224,181,59,7,238,60,188,71,238,
    72,148,153,225,60,184,21,38,202,224,46,48,81,129,241,50,
    200,5,104,33,243,57,104,85,145,213,3,100,117,133,89,253,
    135,88,185,56,114,13,220,42,77,199,179,124,79,51,11,52,
    147,247,184,194,92,150,210,147,45,67,107,37,165,175,230,232,
    213,28,125,45,71,95,207,209,55,114,244,205,28,125,43,71,
    223,206,209,119,114,244,221,28,189,150,163,239,229,232,143,114,
    244,122,142,174,229,232,251,57,250,227,28,253,73,142,254,73,
    142,222,200,209,63,101,122,9,228,10,244,63,133,126,29,250,
    159,65,7,141,186,156,105,238,1,72,19,250,159,67,235,115,
    144,248,123,0,39,104,119,119,37,183,226,11,94,113,53,91,
    177,201,43,182,160,181,5,18,127,155,122,69,9,154,245,235,
    136,37,239,71,252,171,11,164,212,34,54,111,100,20,123,97,
    96,123,65,39,244,12,26,47,81,67,200,107,83,51,151,64,
    240,25,65,240,223,192,248,115,141,4,130,199,128,140,5,201,
    50,48,224,152,137,99,3,198,117,56,18,208,47,128,107,194,
    17,110,83,164,3,116,5,156,24,240,71,147,38,28,99,91,
    64,208,124,4,5,165,241,215,103,208,104,78,115,112,92,132,
    163,34,52,95,31,25,212,113,88,134,232,95,240,195,26,51,
    157,103,166,6,28,97,91,128,147,2,28,151,96,31,39,97,
    87,191,76,226,139,215,71,40,41,246,52,235,5,60,237,94,
    78,92,18,197,245,162,192,241,165,90,69,218,30,58,145,227,
    219,187,35,223,31,63,235,201,246,161,140,234,149,116,98,24,
    111,13,29,213,179,120,165,73,42,241,135,138,57,134,129,84,
    11,72,116,188,192,181,253,208,29,13,164,154,39,118,118,199,
    27,72,219,230,193,231,254,48,140,212,175,163,40,140,44,210,
    42,119,14,66,39,91,65,58,109,15,194,88,214,105,55,222,
    198,34,246,138,102,119,134,204,145,14,192,167,165,197,174,140,
    219,145,55,84,104,44,205,145,102,19,183,58,153,137,155,216,
    198,166,209,11,125,217,64,193,26,187,225,219,128,182,140,27,
    93,233,111,111,198,202,57,24,200,77,199,233,124,249,213,35,
    233,56,238,35,183,113,48,242,6,110,99,199,122,209,24,142,
    85,47,12,26,254,118,195,11,148,68,37,13,26,147,234,217,
    194,89,87,105,163,183,94,215,246,88,68,187,39,7,67,25,
    85,169,247,14,29,66,44,139,69,81,18,166,168,139,42,82,
    69,124,76,177,102,44,136,61,143,132,108,147,224,4,49,51,
    5,213,63,129,205,135,214,63,52,32,90,35,200,244,241,39,
    200,198,8,156,38,141,25,60,246,59,210,142,238,237,155,4,
    4,221,121,196,48,67,188,225,204,199,100,249,0,24,43,69,
    232,151,64,99,8,161,167,65,21,141,169,197,233,196,198,64,
    230,5,136,255,113,150,67,176,12,168,125,12,87,216,117,3,
    183,250,11,195,178,89,167,131,239,49,54,84,207,139,81,183,
    108,1,162,217,145,154,168,147,151,227,239,14,250,178,173,226,
    117,236,248,62,28,213,218,78,16,132,170,230,184,110,205,81,
    42,242,14,70,74,198,53,21,214,54,226,122,153,76,190,146,
    194,43,227,55,30,166,112,34,211,35,156,244,139,235,181,21,
    190,48,110,109,214,127,44,21,66,163,23,186,49,246,19,139,
    174,84,22,29,82,93,193,102,39,221,142,49,88,47,165,136,
    137,229,160,163,42,12,62,39,142,109,222,142,250,25,103,180,
    250,141,51,24,73,69,243,17,48,10,119,37,82,111,52,99,
    164,221,34,89,83,81,73,125,118,16,6,238,24,79,234,181,
    55,232,16,183,24,111,139,64,136,187,142,104,155,195,182,4,
    85,68,223,178,209,38,161,10,9,214,24,103,55,72,5,192,
    182,23,73,8,65,204,157,96,160,169,27,28,41,88,58,118,
    196,26,81,180,216,34,72,91,119,169,89,163,230,94,170,128,
    217,105,161,122,94,11,15,105,103,131,69,111,155,137,144,153,
    51,237,157,113,166,219,167,206,132,161,177,73,78,97,144,235,
    156,58,133,73,106,136,158,36,30,64,238,134,32,192,225,28,
    238,89,57,214,50,9,93,74,33,107,17,14,243,96,236,230,
    192,104,145,93,24,137,214,237,105,138,92,191,76,69,118,181,
    34,183,105,231,197,4,67,85,198,78,69,180,9,0,70,162,
    86,86,233,46,18,227,155,164,210,188,50,111,98,186,219,15,
    170,156,183,56,247,113,101,163,227,136,214,177,38,10,132,182,
    142,9,55,146,124,20,147,219,15,163,240,221,184,22,118,106,
    10,210,51,60,222,136,183,54,226,111,48,48,212,158,112,168,
    209,161,65,59,127,36,135,17,58,121,153,95,180,227,218,236,
    196,118,146,78,80,233,148,219,217,86,172,106,142,83,177,138,
    40,60,205,88,207,149,76,207,116,236,111,104,219,10,43,217,
    132,155,248,84,4,159,205,14,57,82,114,141,193,163,248,60,
    37,117,147,196,18,168,186,181,154,250,228,44,20,137,103,125,
    122,6,53,51,19,201,122,128,123,236,166,110,87,130,12,35,
    244,152,116,104,242,132,191,1,215,97,2,254,10,132,7,52,
    123,226,59,236,165,244,144,89,87,105,250,159,128,131,208,5,
    41,207,208,62,104,36,97,10,93,52,126,196,83,117,6,252,
    45,252,61,23,193,78,76,16,148,173,204,164,210,202,103,171,
    66,230,181,12,164,15,202,72,133,179,238,77,102,234,57,49,
    77,211,142,108,102,142,124,26,12,179,250,8,163,211,236,48,
    54,175,55,180,233,108,207,79,17,70,169,224,174,88,53,114,
    184,249,130,154,205,12,50,34,237,155,201,49,215,207,199,238,
    92,6,179,117,188,252,13,157,165,192,167,95,42,113,33,145,
    103,146,185,71,49,117,143,135,153,123,72,14,226,239,185,34,
    167,214,32,20,156,24,2,175,121,88,188,208,13,171,0,178,
    8,173,18,57,18,151,155,34,241,51,145,134,54,10,137,103,
    50,4,235,103,79,107,46,3,130,182,49,53,239,102,28,74,
    200,204,143,7,142,127,224,58,79,40,61,199,180,115,59,245,
    60,35,21,99,57,47,6,121,141,152,38,9,191,126,153,138,
    243,102,198,97,228,43,208,55,43,22,131,157,198,13,219,28,
    59,94,245,100,205,151,254,1,94,198,122,222,176,214,25,56,
    93,182,150,153,136,249,93,42,166,98,115,159,207,206,49,5,
    168,189,176,214,14,3,140,249,163,182,10,163,154,43,241,130,
    34,221,218,102,141,19,70,205,139,107,206,1,142,58,109,165,
    189,224,172,75,115,17,232,68,221,152,235,189,195,183,68,94,
    130,181,109,188,136,122,88,232,246,83,53,233,235,81,22,253,
    185,182,213,46,133,73,20,47,30,106,172,131,220,215,212,124,
    70,205,6,92,78,146,104,80,184,164,205,72,129,37,140,67,
    101,193,151,165,252,188,151,180,50,158,244,234,23,31,226,213,
    250,179,77,226,219,165,244,123,207,28,200,121,186,251,182,42,
    244,65,129,122,22,200,225,231,254,87,135,103,55,185,4,7,
    25,254,95,253,220,122,120,201,82,88,63,131,164,62,152,230,
    227,34,47,98,85,251,120,95,164,229,120,94,62,190,246,223,
    190,16,82,118,59,146,142,146,218,114,119,102,46,51,7,13,
    125,132,40,179,95,42,88,118,57,121,148,9,119,194,37,210,
    248,90,174,92,102,115,138,125,172,103,176,164,62,98,249,109,
    67,87,213,167,56,45,100,106,160,219,92,32,223,218,147,170,
    208,133,51,29,200,25,14,101,224,158,22,197,60,50,99,48,
    124,157,168,35,173,80,176,2,190,134,207,164,107,146,104,57,
    73,217,144,197,204,25,103,111,82,134,241,187,212,152,117,170,
    76,78,195,176,69,166,212,129,55,139,185,214,47,33,141,195,
    25,80,93,57,144,74,94,96,36,69,107,147,187,163,43,49,
    45,133,99,188,210,240,197,0,223,7,182,125,25,17,252,23,
    184,71,8,201,133,140,34,120,9,99,120,185,80,22,156,38,
    207,125,29,213,103,163,160,175,203,223,113,108,177,191,47,101,
    10,224,79,120,105,146,98,236,211,5,110,207,241,245,55,25,
    254,202,96,221,167,230,147,84,145,12,91,125,51,226,75,135,
    190,242,33,206,57,135,115,202,182,182,168,159,10,112,127,123,
    43,21,104,75,11,148,200,242,236,229,239,249,51,162,191,205,
    70,152,156,247,212,137,37,77,186,117,225,232,78,228,191,250,
    246,169,170,77,29,36,101,239,59,3,220,105,10,135,230,56,
    86,210,87,119,207,13,202,96,228,219,47,164,31,70,227,23,
    161,43,213,253,115,227,59,73,125,162,167,216,111,36,21,50,
    211,206,113,102,238,196,86,122,18,14,234,15,114,92,137,95,
    160,175,65,136,250,114,147,57,247,166,207,217,13,125,7,251,
    47,222,165,233,165,187,172,156,27,119,35,90,117,253,92,111,
    44,35,207,25,120,63,200,9,43,238,184,110,100,57,65,87,
    166,178,95,124,108,180,193,115,234,137,70,67,21,171,59,231,
    230,188,140,194,182,140,227,148,197,197,8,72,38,77,44,126,
    22,142,136,156,182,152,118,110,238,164,163,83,225,131,115,166,
    32,244,57,22,168,175,208,110,8,29,114,143,116,148,195,193,
    4,134,41,100,158,190,114,129,53,145,254,56,132,68,178,235,
    33,226,34,102,115,186,34,201,7,228,214,172,201,11,210,102,
    126,245,37,132,28,125,195,209,223,112,158,208,87,254,248,0,
    27,250,158,89,94,42,139,146,65,223,206,77,81,17,85,81,
    16,139,213,178,89,46,149,139,38,134,37,234,89,21,21,179,
    92,89,20,31,254,191,142,161,172,98,172,87,202,226,191,183,
    112,136,139,
};

EmbeddedPython embedded_m5_internal_param_DummyChecker(
    "m5/internal/param_DummyChecker.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_DummyChecker.py",
    "m5.internal.param_DummyChecker",
    data_m5_internal_param_DummyChecker,
    2307,
    7262);

} // anonymous namespace
