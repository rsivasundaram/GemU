#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_PL031[] = {
    120,156,213,89,123,115,219,198,17,95,0,36,37,82,162,37,
    89,47,63,100,139,142,171,134,118,108,209,78,170,248,143,120,
    60,117,226,78,235,78,173,56,80,167,118,212,7,2,1,71,
    9,20,1,112,128,147,101,122,164,105,39,242,244,241,87,63,
    68,39,127,244,123,244,227,244,59,180,187,123,0,116,164,232,
    113,102,90,51,99,137,188,57,236,237,61,118,247,183,143,3,
    61,200,254,202,248,253,105,3,32,189,103,2,248,248,49,160,
    11,16,26,176,109,128,33,12,240,23,97,191,12,201,79,192,
    47,195,107,128,109,19,132,9,39,216,177,224,183,38,68,211,
    60,167,2,93,139,41,6,244,107,32,74,176,93,134,103,209,
    28,148,68,5,246,107,144,124,3,134,97,68,6,60,247,39,
    192,159,132,215,184,58,118,170,188,224,36,248,53,238,84,193,
    159,226,78,13,250,179,32,166,96,27,23,159,128,237,58,46,
    117,19,151,58,199,75,253,139,150,242,113,100,9,252,58,177,
    227,89,190,38,206,18,113,242,30,231,120,149,25,240,103,104,
    149,54,202,48,91,48,226,194,22,116,230,96,123,14,4,126,
    102,225,4,197,204,68,56,15,219,243,185,56,11,90,127,81,
    235,47,105,253,101,173,127,65,235,95,212,250,151,180,254,101,
    173,191,162,245,175,104,253,171,90,127,85,235,55,180,254,53,
    173,255,129,214,191,174,245,127,196,125,148,112,30,58,107,208,
    249,49,116,62,100,77,156,47,52,209,100,77,220,128,237,27,
    32,240,211,84,154,152,215,102,220,228,25,11,197,140,143,120,
    198,45,216,190,5,2,63,31,169,25,21,216,106,46,35,136,
    130,255,224,95,211,192,158,156,198,230,133,72,210,32,142,156,
    32,106,199,129,73,227,21,106,8,114,30,53,19,25,246,190,
    32,236,253,19,24,120,190,153,97,239,24,112,97,131,100,233,
    154,112,204,157,99,19,250,77,56,50,160,83,2,223,130,35,
    220,166,76,7,216,53,224,196,132,223,89,196,112,140,109,9,
    209,114,21,74,82,1,175,195,104,81,43,77,192,113,25,142,
    202,176,245,252,200,36,194,126,21,146,239,224,213,10,47,58,
    201,139,154,112,132,109,9,78,74,112,92,129,103,200,132,164,
    78,149,196,55,158,31,161,164,72,217,106,150,240,180,155,154,
    184,36,138,31,36,145,27,10,22,221,233,185,137,27,58,79,
    127,117,231,147,187,205,90,206,17,167,235,61,87,238,217,60,
    197,34,93,132,61,201,75,197,145,144,83,216,105,7,145,239,
    132,177,127,208,21,114,146,214,113,218,65,87,56,14,15,62,
    14,123,113,34,127,150,36,113,98,147,58,153,216,141,221,98,
    6,41,211,235,198,169,104,210,110,188,141,77,203,75,226,110,
    247,120,69,58,0,31,147,38,251,34,245,146,160,39,209,74,
    106,69,226,166,213,154,100,31,110,210,231,216,180,246,226,80,
    180,80,162,214,163,248,48,162,45,211,214,174,8,55,110,167,
    210,221,233,138,219,174,219,190,115,247,158,112,93,255,158,223,
    218,57,8,186,126,235,161,253,164,213,235,203,189,56,106,133,
    27,173,32,146,2,181,211,109,105,122,89,199,225,243,180,195,
    97,176,235,4,44,155,179,39,186,61,145,212,137,122,137,118,
    55,102,141,105,163,98,88,70,211,168,99,175,140,95,203,88,
    49,167,140,205,128,164,243,72,98,2,149,149,195,232,31,192,
    6,67,123,239,155,144,172,16,72,58,248,49,200,170,8,149,
    45,26,51,121,236,43,82,139,162,118,44,50,189,34,30,49,
    176,16,97,200,121,159,108,29,1,163,163,12,157,10,40,212,
    32,216,20,140,146,62,181,200,78,203,152,184,120,9,210,191,
    15,174,16,205,2,170,29,35,19,146,150,112,171,111,25,136,
    91,77,58,248,38,131,66,238,5,41,42,149,85,79,125,198,
    207,22,234,228,105,255,203,157,142,240,100,186,138,132,175,227,
    131,134,231,70,81,44,27,174,239,55,92,41,147,96,231,64,
    138,180,33,227,198,90,218,172,146,173,231,114,92,21,235,245,
    123,57,142,200,230,136,35,245,224,7,158,196,135,121,126,96,
    253,167,66,34,38,246,98,63,69,58,45,177,43,164,77,135,
    148,231,176,121,152,111,199,224,107,86,114,168,164,162,219,150,
    53,70,157,155,166,14,111,71,116,6,24,205,126,225,118,15,
    132,36,126,68,138,196,93,169,171,54,26,23,196,46,144,144,
    185,140,164,55,39,138,35,191,143,71,12,188,53,218,253,2,
    3,109,26,8,106,139,8,179,9,108,43,80,71,216,205,154,
    30,73,83,202,64,198,0,91,34,217,129,141,110,100,209,2,
    193,118,130,49,165,105,114,80,96,177,216,245,26,212,163,201,
    54,97,217,190,76,205,10,53,87,114,201,199,32,126,125,88,
    252,143,105,75,147,101,246,172,76,186,194,125,54,7,220,231,
    226,169,251,96,248,219,34,55,48,201,89,78,221,192,34,249,
    147,7,25,230,201,193,208,236,56,172,33,157,181,98,207,146,
    180,149,28,164,54,33,79,135,223,174,6,63,155,12,194,216,
    179,47,190,73,131,171,63,136,6,119,149,6,55,104,203,233,
    12,53,117,70,75,205,240,200,228,102,166,79,214,229,35,236,
    244,151,73,151,186,22,151,49,151,61,139,234,156,148,56,177,
    113,189,162,66,134,82,174,234,148,8,95,109,11,150,178,100,
    147,146,135,247,146,248,101,191,17,183,27,18,242,51,220,95,
    75,215,215,210,207,48,6,52,30,112,84,81,81,64,249,121,
    34,122,9,250,115,149,31,148,143,58,236,175,78,150,50,80,
    219,139,164,69,51,215,49,135,164,84,38,20,137,198,165,224,
    90,161,96,58,239,103,180,95,141,181,107,193,50,126,107,6,
    31,202,137,57,26,114,229,192,163,248,253,156,244,76,162,10,
    160,98,213,222,82,71,102,105,72,46,251,195,1,156,188,123,
    89,236,155,184,248,163,220,195,42,80,160,130,190,22,157,150,
    64,255,23,224,178,202,128,63,3,33,0,13,157,185,9,59,
    36,125,201,144,243,196,254,7,224,64,51,34,159,153,202,221,
    204,44,20,161,55,166,247,152,85,165,183,95,194,95,181,40,
    117,98,129,65,169,200,202,10,39,61,21,149,10,7,101,232,
    124,175,116,83,26,244,100,178,207,158,155,18,155,242,89,171,
    240,217,211,128,87,84,61,24,136,198,128,170,73,181,147,67,
    135,122,124,138,41,138,243,151,141,121,83,67,202,45,106,110,
    23,32,49,114,218,187,61,223,234,112,96,214,242,146,163,130,
    225,47,232,16,37,62,246,76,133,139,56,25,22,248,47,231,
    248,255,119,129,127,193,113,249,53,23,210,212,154,100,237,19,
    211,192,107,153,186,144,33,3,94,199,58,19,220,78,18,102,
    240,202,149,17,107,220,78,49,113,58,39,214,185,61,199,196,
    153,156,56,203,237,28,19,207,231,196,121,110,23,152,184,152,
    19,151,184,93,102,226,133,156,120,145,219,75,76,188,156,19,
    87,184,189,194,196,171,84,47,209,253,109,149,137,120,239,185,
    150,95,61,63,32,31,167,52,194,142,149,127,57,220,82,152,
    30,72,87,108,200,77,101,226,2,170,10,133,212,188,28,87,
    120,35,32,222,239,186,225,142,239,62,216,162,221,104,75,47,
    15,10,102,126,254,89,253,252,228,208,198,155,68,224,199,59,
    185,28,47,198,21,218,238,226,226,197,249,217,145,253,216,227,
    120,246,235,61,209,8,69,184,131,247,189,189,160,215,104,119,
    221,221,34,240,21,165,197,61,224,194,73,25,7,163,94,127,
    65,203,121,44,174,241,12,67,20,230,197,35,206,152,142,169,
    82,227,169,37,57,232,176,23,16,2,34,113,232,200,80,101,
    59,122,118,123,61,17,249,167,153,140,71,198,105,99,7,47,
    186,1,150,213,191,215,130,13,166,175,5,252,158,69,43,201,
    162,137,198,49,167,92,68,159,75,227,59,58,155,117,39,55,
    107,243,58,232,169,147,139,127,21,164,176,244,192,155,153,236,
    171,68,113,131,154,181,194,30,117,14,77,104,14,140,101,30,
    149,73,67,20,140,112,108,35,245,168,13,134,65,52,196,78,
    20,141,29,31,249,16,106,112,47,62,72,152,127,136,68,19,
    38,20,137,158,181,225,208,119,251,67,51,152,164,205,160,103,
    253,0,241,153,35,197,131,71,138,245,35,245,133,59,124,36,
    38,105,27,208,179,54,124,120,246,72,135,67,71,162,103,125,
    193,179,51,250,67,51,232,217,254,148,250,84,236,249,162,43,
    164,64,62,57,11,69,141,141,151,125,153,196,125,172,0,185,
    156,194,231,174,227,140,181,56,106,17,96,32,187,219,87,48,
    13,87,176,120,30,252,175,150,170,6,223,28,121,210,217,100,
    71,183,151,183,38,187,44,125,148,64,148,97,187,66,41,131,
    223,140,252,47,41,131,227,237,56,93,178,15,255,207,76,97,
    127,252,67,29,223,254,4,178,170,247,77,89,194,202,100,251,
    50,151,77,178,77,135,175,144,41,149,214,155,113,195,139,35,
    196,241,129,39,227,164,225,139,118,16,9,191,113,187,193,151,
    155,70,144,54,220,29,28,117,61,169,172,54,88,140,242,187,
    9,55,217,77,217,149,247,15,169,59,46,165,144,111,190,42,
    194,236,196,64,152,181,41,55,170,192,250,233,64,116,29,171,
    119,82,110,127,73,187,84,50,239,68,79,156,202,61,241,41,
    241,166,103,253,113,231,251,248,163,42,62,51,175,172,112,201,
    55,65,245,39,181,85,46,249,106,121,129,55,197,196,105,122,
    145,239,87,152,114,142,252,119,226,189,243,223,63,189,223,254,
    91,28,255,237,254,107,232,178,213,33,175,242,178,247,65,186,
    96,92,43,156,31,132,148,227,37,194,149,98,236,117,15,135,
    0,181,247,183,133,169,222,105,209,58,3,170,104,213,132,87,
    166,222,40,76,61,238,186,149,195,205,107,120,239,234,213,191,
    229,22,107,146,82,71,6,210,211,10,245,121,97,129,133,33,
    232,201,32,20,92,87,141,28,160,234,138,95,162,227,131,42,
    173,8,186,89,105,165,27,241,43,218,192,134,49,135,235,223,
    224,226,127,164,93,168,40,84,197,84,53,43,160,56,227,13,
    253,226,166,142,69,111,57,212,59,152,126,106,179,123,207,20,
    218,225,95,135,242,42,159,129,79,165,228,166,27,170,183,254,
    252,58,219,190,70,205,245,92,193,236,68,234,189,28,191,249,
    82,111,26,17,228,156,142,57,251,218,235,212,180,114,7,160,
    27,0,22,163,137,216,13,82,148,139,223,130,132,27,235,185,
    144,235,74,200,135,225,142,251,56,146,143,196,139,192,19,252,
    226,35,220,144,23,71,178,126,238,166,226,231,129,39,47,141,
    28,125,218,117,101,59,78,66,121,101,228,48,238,145,124,129,
    229,124,18,119,249,119,130,179,28,91,125,60,102,40,47,15,
    13,138,232,32,116,158,136,48,78,250,79,98,95,200,107,67,
    227,15,179,42,68,177,56,47,4,149,43,178,49,90,216,1,
    222,51,91,41,38,28,84,191,6,189,65,99,95,116,99,111,
    95,248,25,207,104,105,153,231,81,28,186,72,31,189,203,86,
    144,239,50,55,52,238,39,52,107,113,136,154,138,36,112,187,
    193,43,33,87,134,21,224,251,137,237,70,187,34,151,125,244,
    134,79,131,56,51,242,155,129,112,202,51,90,127,136,128,192,
    59,101,34,24,231,28,252,19,195,40,72,81,154,26,160,112,
    245,115,154,155,6,65,58,63,204,157,57,62,185,32,227,70,
    143,29,250,204,113,6,4,245,158,73,189,214,127,64,117,91,
    250,13,54,244,163,86,117,166,106,84,76,250,229,212,50,106,
    70,221,40,25,211,245,170,85,173,84,203,22,6,13,162,204,
    27,53,171,90,91,154,171,26,53,228,123,251,255,106,137,56,
    87,167,170,198,127,1,153,11,160,86,
};

EmbeddedPython embedded_m5_internal_param_PL031(
    "m5/internal/param_PL031.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_PL031.py",
    "m5.internal.param_PL031",
    data_m5_internal_param_PL031,
    2602,
    8773);

} // anonymous namespace