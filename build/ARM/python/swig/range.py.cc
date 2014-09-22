#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_range[] = {
    120,156,205,90,123,111,27,89,21,63,51,99,59,177,27,55,
    206,187,143,180,49,66,21,102,69,155,221,133,221,174,104,41,
    180,205,178,100,161,161,140,11,105,13,194,76,60,55,246,164,
    246,140,53,51,73,235,146,8,137,84,176,127,32,45,223,1,
    33,196,247,64,124,45,56,191,115,103,198,118,31,18,108,106,
    67,18,95,93,95,95,159,231,239,60,238,157,180,40,249,201,
    243,235,7,85,162,232,159,6,145,203,127,6,117,137,122,6,
    53,12,50,148,65,238,69,122,154,167,240,59,228,230,233,37,
    81,195,36,101,210,41,79,44,250,165,73,254,156,124,167,64,
    93,75,86,12,26,148,72,229,168,145,167,93,127,129,114,170,
    64,79,75,20,254,134,12,195,240,13,122,236,206,144,59,75,
    47,153,58,79,138,66,112,150,220,146,76,138,228,158,147,73,
    137,6,21,82,231,168,193,196,103,168,81,102,82,239,49,169,
    243,66,234,31,32,229,242,39,151,200,45,99,59,203,242,4,
    59,115,216,41,60,206,11,149,121,114,231,65,101,159,117,168,
    100,27,153,176,69,7,11,212,88,32,197,127,21,58,133,154,
    188,180,72,141,69,82,139,116,176,68,141,37,114,23,132,198,
    178,236,94,198,196,93,148,149,21,89,89,193,196,93,146,149,
    85,89,89,197,196,93,78,25,174,100,12,215,228,227,11,212,
    184,64,138,255,214,52,67,230,115,145,26,23,133,225,37,106,
    92,146,201,101,106,92,150,201,58,53,214,201,93,21,242,87,
    228,251,87,48,113,215,100,229,170,172,92,197,196,189,32,43,
    27,178,178,129,9,187,162,94,187,204,62,245,254,197,63,53,
    246,41,197,115,60,28,169,48,242,2,191,233,249,251,129,103,
    226,243,2,6,32,160,133,97,38,129,194,125,64,225,239,36,
    56,112,205,4,10,39,68,6,222,19,117,77,58,145,201,137,
    73,131,26,29,27,116,144,35,215,162,99,102,147,135,106,109,
    131,78,77,250,149,133,13,39,60,230,216,121,87,41,23,107,
    28,28,136,243,52,165,25,58,201,211,113,158,234,143,143,77,
    44,60,45,82,248,87,122,177,46,68,103,133,168,73,199,60,
    230,232,52,71,39,5,218,229,77,188,116,80,132,97,141,199,
    199,172,41,175,212,107,57,150,118,103,68,93,168,226,122,161,
    239,244,84,12,37,155,161,227,183,85,173,148,126,22,68,55,
    250,78,220,177,101,179,5,43,244,250,177,16,9,124,21,159,
    227,201,190,231,187,205,94,224,30,118,85,60,11,10,205,125,
    175,171,154,77,249,112,187,215,15,194,248,211,48,12,66,27,
    134,148,197,110,224,100,223,128,25,91,221,32,82,53,112,19,
    54,54,200,199,216,189,223,23,138,16,64,4,196,151,93,21,
    181,66,175,31,179,127,52,69,236,6,181,26,60,35,67,244,
    99,30,54,59,65,79,109,134,78,111,115,43,120,230,131,101,
    180,217,86,189,143,174,71,177,179,215,85,215,29,103,255,253,
    15,110,42,199,113,111,186,155,123,135,94,215,221,188,107,63,
    216,236,15,226,78,224,111,70,207,188,246,166,152,226,6,175,
    44,130,40,175,52,61,81,167,217,81,221,190,10,203,88,189,
    4,134,70,197,152,51,10,134,101,212,140,50,207,242,252,178,
    140,117,243,156,177,227,65,161,22,148,132,113,173,20,51,127,
    33,241,14,59,247,169,73,225,58,16,113,192,127,6,92,200,
    184,168,227,51,83,62,251,25,44,161,87,15,44,248,89,47,
    30,11,138,24,78,188,243,54,28,235,147,64,33,79,7,5,
    210,16,97,100,105,204,132,3,140,188,29,100,76,38,158,163,
    232,203,113,10,126,133,216,210,156,21,120,105,149,89,253,94,
    80,87,175,65,240,29,193,65,220,241,34,182,163,88,27,115,
    137,147,58,219,228,225,224,167,123,7,170,21,71,27,188,240,
    36,56,172,182,28,223,15,226,170,227,186,85,39,142,67,111,
    239,48,86,81,53,14,170,215,162,90,17,238,93,72,161,148,
    209,27,244,83,232,192,205,12,29,253,198,245,90,49,191,89,
    146,55,98,255,72,197,12,131,78,224,70,188,14,18,109,21,
    219,16,50,62,207,195,221,148,157,224,173,86,72,209,17,169,
    238,126,92,18,160,57,81,212,20,118,88,23,76,225,219,71,
    78,247,80,195,159,193,17,51,87,76,53,163,9,162,234,2,
    244,74,213,130,169,154,126,224,187,3,150,202,107,93,3,195,
    11,130,173,57,2,186,86,24,89,51,60,22,168,204,72,171,
    152,45,40,144,75,112,37,152,90,133,186,36,126,54,146,108,
    192,248,58,229,156,81,51,37,232,69,19,9,176,42,102,248,
    178,13,248,218,200,128,246,58,134,43,169,178,147,209,184,252,
    170,198,31,130,139,41,106,182,172,68,161,44,72,118,198,130,
    228,226,48,72,56,163,213,1,118,19,33,49,4,187,5,149,
    195,59,9,178,17,70,236,92,254,120,4,207,98,8,187,2,
    5,11,41,20,109,224,107,20,100,237,17,144,217,240,129,32,
    204,190,248,54,163,109,76,203,104,109,109,180,143,192,101,46,
    193,70,89,48,81,50,90,112,172,153,152,80,204,183,197,147,
    193,26,204,55,106,184,53,174,72,187,126,89,74,139,148,39,
    105,2,116,46,208,246,212,147,28,80,180,111,209,106,82,50,
    34,132,110,63,12,158,15,170,193,126,53,166,84,134,219,215,
    162,27,215,162,91,28,220,213,59,146,46,116,120,235,0,14,
    85,63,228,64,45,202,27,29,124,77,9,196,102,146,254,217,
    192,43,48,156,153,154,85,114,77,20,135,72,49,19,180,105,
    41,179,41,68,188,5,22,37,49,168,69,107,252,42,25,34,
    71,51,144,204,38,37,95,62,229,215,61,152,22,218,41,66,
    211,103,215,181,148,162,0,84,177,191,49,134,134,137,136,111,
    191,199,244,182,210,208,41,80,230,123,188,44,8,8,52,255,
    145,164,185,50,232,15,4,63,179,59,19,252,75,164,225,5,
    119,45,97,251,175,73,146,198,27,202,145,169,227,200,76,210,
    10,135,89,116,83,182,234,234,244,57,125,49,146,113,78,45,
    50,80,73,172,164,201,25,173,36,185,44,242,4,32,255,81,
    181,200,141,135,40,92,210,113,34,108,211,193,104,101,193,56,
    76,94,89,159,194,25,102,50,216,153,213,196,155,144,99,123,
    136,28,164,233,203,198,146,57,130,135,111,97,184,158,65,193,
    72,215,222,185,72,27,175,230,213,145,74,210,212,185,236,71,
    224,155,19,73,231,11,82,223,238,15,90,93,238,165,82,112,
    231,83,112,111,101,224,86,146,90,95,74,123,139,209,132,95,
    79,77,131,207,46,250,212,194,13,3,58,234,2,169,25,116,
    161,56,148,228,147,67,9,7,6,152,8,52,211,151,164,37,
    164,179,177,76,46,118,217,209,22,203,156,173,253,136,225,249,
    4,211,0,92,121,187,235,244,246,92,231,78,29,12,192,165,
    149,70,146,153,138,92,25,21,25,81,96,188,77,106,121,251,
    126,42,250,209,4,83,192,7,76,47,19,89,0,239,6,45,
    137,251,71,29,85,237,169,222,30,159,97,58,94,191,186,223,
    117,218,226,133,52,234,63,131,74,223,37,105,22,180,62,127,
    210,39,148,229,145,34,32,113,111,236,114,52,115,161,56,150,
    240,111,154,186,86,12,93,38,241,41,61,56,66,192,87,207,
    154,26,83,186,4,192,253,78,191,175,124,119,24,168,210,118,
    57,97,91,111,153,176,103,155,124,126,243,184,129,252,249,48,
    72,231,57,185,47,243,235,117,88,66,176,17,213,36,86,243,
    89,212,94,154,168,180,226,204,70,234,204,26,78,57,195,90,
    34,157,173,14,110,46,191,124,236,136,7,58,141,126,19,3,
    250,68,251,102,230,7,4,151,171,248,88,165,18,87,196,21,
    202,250,26,62,57,197,97,48,224,18,44,197,141,223,119,155,
    205,73,151,170,77,200,150,58,160,192,25,178,96,20,115,69,
    67,210,78,90,182,62,75,243,66,130,72,2,34,199,34,44,
    3,89,24,244,30,120,173,48,120,120,95,59,230,227,201,74,
    127,27,112,165,97,143,250,21,164,198,14,63,8,123,78,119,
    154,130,127,143,36,137,159,69,112,116,119,94,100,79,215,224,
    119,72,223,179,36,114,75,235,120,215,117,67,27,27,94,175,
    85,127,163,255,190,86,37,37,106,6,183,63,201,149,218,76,
    122,165,54,155,84,175,228,182,237,92,122,219,54,151,222,182,
    149,201,157,147,201,249,228,66,13,55,103,250,10,173,130,43,
    52,76,22,210,155,177,229,228,46,173,177,130,114,136,140,119,
    166,114,40,133,101,194,89,168,71,239,178,10,218,31,78,81,
    98,251,219,148,116,190,255,139,10,40,97,206,21,48,3,171,
    182,7,206,106,35,125,234,199,169,161,38,104,20,84,131,128,
    222,94,242,50,135,142,165,128,87,189,41,74,173,140,70,31,
    87,211,88,133,93,229,28,41,119,26,85,81,110,251,70,120,
    70,25,52,207,172,73,155,121,28,118,157,208,139,7,83,211,
    100,132,103,124,22,77,22,198,52,225,138,238,245,85,52,21,
    45,146,163,57,248,29,158,69,131,243,227,26,120,47,212,84,
    196,151,155,64,102,118,116,22,217,231,199,100,63,114,186,222,
    116,98,33,185,165,244,220,103,239,78,122,22,35,140,167,38,
    189,112,123,126,22,233,151,198,164,143,3,1,191,223,158,138,
    6,232,67,50,142,131,49,45,228,210,133,146,170,82,25,213,
    2,133,229,77,138,44,143,41,210,83,97,91,69,187,94,220,
    25,94,131,77,184,205,146,86,122,200,247,197,59,213,71,82,
    118,132,39,2,211,213,103,200,247,183,103,212,103,113,92,159,
    168,126,184,135,219,164,169,105,131,195,107,202,245,248,157,234,
    210,10,252,216,241,252,41,122,6,186,164,92,79,94,139,254,
    255,159,227,247,239,82,217,106,120,142,50,60,126,219,159,96,
    120,211,129,251,30,6,92,150,217,63,196,128,11,54,251,115,
    12,63,193,128,135,23,246,67,12,56,253,218,143,48,252,34,
    243,10,232,37,231,244,97,195,136,142,212,190,53,166,234,100,
    244,253,62,211,235,130,48,91,126,228,76,254,230,223,175,114,
    116,148,71,185,224,246,233,243,169,156,26,31,51,189,47,232,
    76,167,221,76,228,109,127,42,34,63,33,17,228,44,34,23,
    83,145,235,89,11,53,97,161,113,71,245,231,161,208,114,176,
    121,229,255,35,52,119,0,75,223,194,15,34,224,159,236,249,
    76,108,121,162,159,222,100,125,146,42,178,227,244,244,99,91,
    121,56,105,127,13,195,215,211,168,147,187,68,253,48,70,158,
    125,232,39,74,124,220,146,103,74,242,48,192,190,129,1,183,
    77,210,39,232,219,47,185,247,10,85,219,139,184,52,72,3,
    216,58,226,46,4,190,126,224,60,127,228,181,158,202,13,143,
    220,150,200,213,131,132,70,140,12,48,210,38,141,210,128,87,
    146,251,16,59,232,221,243,226,148,24,246,75,127,181,237,75,
    159,246,168,19,42,199,221,222,146,111,36,107,15,131,48,222,
    222,18,180,138,255,197,158,19,14,118,125,199,173,159,182,221,
    193,57,36,2,83,60,81,46,206,23,141,130,137,255,84,176,
    140,146,81,54,114,198,92,185,104,21,11,197,188,197,9,1,
    43,75,70,201,42,150,86,139,69,126,199,169,192,42,154,217,
    175,177,186,156,172,190,242,201,191,1,72,46,208,49,
};

EmbeddedPython embedded_m5_internal_range(
    "m5/internal/range.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/swig/range.py",
    "m5.internal.range",
    data_m5_internal_range,
    2510,
    9515);

} // anonymous namespace