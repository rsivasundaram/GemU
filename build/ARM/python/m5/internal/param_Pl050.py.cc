#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_Pl050[] = {
    120,156,197,88,109,111,27,199,17,158,189,35,41,145,18,45,
    201,178,252,42,71,116,82,213,140,83,139,142,91,213,65,99,
    24,117,172,162,117,1,43,234,169,168,29,54,232,245,116,183,
    164,142,186,23,226,110,37,133,129,244,165,50,218,126,235,143,
    40,250,161,255,163,255,171,157,153,189,59,158,100,10,48,144,
    134,145,200,197,112,119,110,118,94,158,153,157,61,23,178,191,
    42,126,127,217,2,72,111,27,0,30,126,4,4,0,161,128,
    174,0,33,5,120,203,112,80,133,228,103,224,85,225,45,64,
    215,0,105,192,25,18,38,252,209,128,104,158,159,169,65,96,
    242,140,128,81,3,100,5,186,85,120,29,45,65,69,214,224,
    160,1,201,159,65,8,17,9,120,227,205,128,55,11,111,81,
    58,18,117,22,56,11,94,131,137,58,120,115,76,52,96,180,
    8,114,14,186,40,124,6,186,77,20,245,0,69,93,97,81,
    255,33,81,30,174,92,3,175,73,236,168,203,87,196,89,33,
    78,222,227,10,75,89,200,53,91,132,238,82,78,95,45,209,
    203,37,250,90,137,94,41,209,215,75,244,141,18,125,179,68,
    223,42,209,183,75,244,157,18,189,90,162,239,150,232,15,74,
    244,90,137,110,149,232,123,37,250,195,18,253,17,211,11,32,
    151,96,240,35,24,172,195,224,199,208,195,128,45,22,94,185,
    15,210,132,65,27,186,109,144,248,185,15,103,24,83,111,169,
    244,196,199,252,196,213,226,137,7,252,196,39,208,253,4,36,
    126,30,232,39,106,176,219,94,65,156,248,255,197,191,182,64,
    74,205,227,112,36,147,212,143,35,219,143,122,177,111,208,122,
    141,6,66,149,75,195,76,6,175,23,4,175,127,3,99,203,
    51,50,120,157,2,10,22,100,75,96,192,41,19,167,6,140,
    218,112,34,96,80,1,207,132,19,220,166,74,10,244,5,156,
    25,240,181,73,12,167,56,86,16,16,31,64,69,105,108,13,
    24,16,90,210,12,156,86,225,164,10,187,111,78,12,154,56,
    168,67,242,47,248,118,149,133,206,178,80,3,78,112,172,192,
    89,5,78,107,240,26,153,112,106,80,39,243,197,155,19,180,
    20,103,118,219,21,212,118,187,100,46,153,226,249,73,228,132,
    146,77,183,135,78,226,132,246,78,240,104,243,81,187,145,115,
    196,233,198,208,81,251,22,63,98,146,47,194,161,98,81,113,
    36,213,28,18,61,63,242,236,48,246,14,3,169,102,73,142,
    221,243,3,105,219,188,248,50,28,198,137,250,85,146,196,137,
    69,238,228,201,32,118,138,39,200,153,110,16,167,178,77,187,
    241,54,22,137,87,196,221,27,178,68,82,128,213,164,135,61,
    153,186,137,63,84,24,37,45,145,184,73,90,155,226,195,67,
    250,6,135,206,126,28,202,14,90,212,217,138,143,35,218,50,
    237,244,101,184,249,48,85,206,94,32,31,58,78,239,209,167,
    79,164,227,120,79,188,206,222,161,31,120,157,231,214,171,206,
    112,164,246,227,168,19,110,118,252,72,73,244,78,208,41,249,
    101,3,151,175,210,14,199,126,223,246,217,54,123,95,6,67,
    153,52,105,246,54,237,46,22,197,188,168,9,83,180,69,19,
    169,42,126,77,177,106,204,137,109,159,172,115,201,98,2,149,
    153,195,232,159,192,1,195,120,31,24,144,172,18,72,6,248,
    17,20,85,132,202,46,173,25,188,246,59,114,139,158,29,152,
    20,122,61,121,194,192,66,132,33,231,83,138,117,4,140,142,
    42,12,106,160,81,131,96,211,48,74,70,52,34,59,137,49,
    80,120,5,210,127,156,151,16,45,2,186,29,139,15,78,93,
    199,173,254,194,64,220,109,147,226,219,12,10,181,239,167,232,
    84,118,61,209,140,159,93,244,201,206,232,203,189,129,116,85,
    186,134,19,95,197,135,45,215,137,162,88,181,28,207,107,57,
    74,37,254,222,161,146,105,75,197,173,245,180,93,167,88,47,
    229,184,42,228,141,134,57,142,40,230,136,35,253,195,243,93,
    133,63,150,249,7,251,63,149,10,49,177,31,123,41,206,147,
    136,190,84,22,41,169,174,224,240,60,223,142,193,215,174,229,
    80,73,101,208,83,13,70,157,147,166,54,111,71,243,12,48,
    122,250,200,9,14,165,34,126,68,138,194,93,137,212,27,77,
    11,98,55,201,200,220,70,242,155,29,197,145,55,66,21,125,
    119,157,118,191,201,64,155,7,130,218,10,194,108,6,199,26,
    52,17,118,139,134,75,214,84,50,144,49,192,174,147,237,192,
    65,23,89,181,64,176,157,97,77,105,27,92,20,216,44,78,
    189,22,81,244,176,69,88,182,238,208,176,74,195,221,220,242,
    41,152,223,188,104,254,99,218,210,96,155,93,51,179,174,72,
    159,237,115,233,115,107,156,62,88,254,118,41,13,12,74,150,
    113,26,152,100,127,242,44,195,60,37,24,134,29,151,75,72,
    103,175,88,139,100,109,45,7,169,69,200,43,195,175,95,130,
    159,69,1,97,236,89,183,46,243,224,218,15,226,193,190,246,
    224,38,109,57,159,161,166,201,104,105,8,151,66,110,100,254,
    100,95,110,33,49,186,65,190,44,123,241,6,158,101,175,163,
    38,31,74,124,176,113,75,162,75,134,118,174,38,42,132,175,
    158,9,215,179,195,38,165,12,31,38,241,55,163,86,220,107,
    41,200,117,120,186,158,110,172,167,159,99,13,104,61,227,170,
    162,171,128,206,243,68,14,19,204,231,58,255,208,57,106,115,
    190,218,217,145,129,222,166,131,155,131,196,62,230,146,148,170,
    132,42,209,180,28,220,40,28,76,250,126,78,251,53,216,187,
    38,220,192,111,67,176,82,118,204,213,144,59,7,94,197,239,
    23,228,103,50,85,2,245,163,214,174,86,153,173,33,187,172,
    251,231,112,242,253,219,98,61,64,225,91,121,134,213,160,64,
    5,125,77,210,150,64,255,55,224,182,74,192,95,129,16,128,
    129,206,210,132,19,146,190,20,200,101,98,255,19,112,161,153,
    112,158,25,58,221,140,172,20,97,54,166,79,152,85,31,111,
    191,133,191,151,170,212,153,9,130,142,34,51,107,156,202,71,
    81,165,72,80,134,206,123,29,55,149,243,153,76,241,217,119,
    82,98,211,57,107,22,57,59,46,120,69,215,131,133,104,10,
    168,154,213,59,217,164,212,203,49,166,168,206,223,17,203,70,
    9,41,63,161,225,97,1,18,145,207,125,191,250,173,93,44,
    204,165,115,201,214,197,240,55,164,68,133,213,94,168,241,121,
    202,79,23,41,80,205,83,224,113,145,2,146,75,243,91,238,
    165,105,52,40,224,103,134,192,203,23,54,33,116,239,169,128,
    172,66,183,70,201,194,253,162,200,114,73,228,117,139,234,221,
    185,186,207,30,217,214,190,42,98,174,195,73,195,55,211,170,
    19,20,209,167,129,19,238,121,206,179,30,237,70,91,186,121,
    118,25,185,254,139,101,253,41,51,196,101,38,240,207,71,185,
    29,71,211,170,17,159,162,240,66,127,206,8,47,118,185,48,
    252,126,95,182,66,25,238,225,197,105,223,31,182,122,129,211,
    231,248,152,153,125,95,230,246,41,14,240,197,83,54,165,234,
    179,29,183,220,56,194,18,126,232,170,56,105,121,18,239,20,
    210,107,61,108,113,253,111,249,105,203,217,195,85,199,85,26,
    233,231,243,149,219,55,39,233,167,220,169,29,28,19,57,205,
    248,218,120,91,244,177,55,237,231,254,209,87,153,162,152,115,
    59,170,211,6,15,67,188,43,168,145,46,93,63,167,225,99,
    26,214,97,202,53,191,163,179,15,82,114,89,13,171,75,93,
    240,237,138,25,118,136,55,125,55,101,143,223,39,101,245,155,
    146,44,113,107,196,41,103,232,162,74,99,157,74,126,183,145,
    79,206,241,56,207,147,205,252,101,204,21,158,92,128,238,34,
    189,17,160,153,37,202,251,153,239,154,247,156,52,211,76,151,
    240,255,154,238,214,227,31,74,125,235,167,144,245,0,151,165,
    186,40,219,214,212,169,62,16,121,119,93,54,140,47,236,87,
    207,227,204,118,19,233,40,169,131,116,123,122,86,114,181,208,
    123,199,69,168,114,83,138,219,197,147,194,156,51,110,124,70,
    215,74,109,47,71,78,188,198,46,5,91,227,19,182,216,54,
    116,119,60,198,98,165,48,124,1,135,72,30,219,37,227,117,
    231,75,154,56,195,161,140,188,113,87,203,43,211,10,56,85,
    162,4,198,13,7,182,176,215,240,251,110,194,145,49,37,219,
    56,102,213,34,197,166,24,61,198,232,113,30,183,54,97,106,
    92,113,45,138,154,174,177,69,121,181,126,81,196,225,230,5,
    0,250,41,94,45,14,83,73,151,165,203,23,177,229,225,114,
    159,79,240,93,176,204,121,20,185,44,97,210,60,61,76,152,
    66,154,157,205,73,224,201,64,42,89,134,131,34,157,179,251,
    165,39,241,200,139,71,120,251,225,171,4,254,14,108,123,170,
    135,196,103,40,60,128,172,94,209,33,81,195,99,98,5,255,
    235,149,186,224,243,247,194,43,82,173,24,245,161,186,105,30,
    165,22,87,144,133,194,245,252,58,47,63,4,57,183,232,162,
    183,237,132,250,53,13,191,127,176,238,209,240,81,30,61,206,
    14,125,145,226,171,138,190,26,98,30,113,115,192,189,128,181,
    65,243,132,190,112,115,35,183,102,67,91,243,135,200,125,25,
    13,15,21,191,80,12,55,213,157,137,92,187,126,168,223,92,
    169,165,11,235,94,226,32,189,114,97,54,149,137,239,4,254,
    183,146,123,226,119,229,61,15,247,156,151,145,218,146,71,190,
    43,213,173,137,60,95,56,169,252,181,239,94,162,247,78,224,
    168,94,156,132,234,238,196,101,20,158,188,136,113,136,3,134,
    236,4,147,70,169,146,225,59,246,202,232,48,180,95,201,48,
    78,70,175,98,79,170,123,23,214,159,103,157,150,102,177,143,
    36,181,100,170,53,217,202,115,188,151,184,22,23,51,215,78,
    118,213,139,32,118,15,164,151,241,76,182,150,121,182,226,144,
    66,177,122,81,97,207,75,44,39,234,203,92,215,201,106,236,
    248,113,22,141,203,35,54,230,153,108,47,70,204,119,199,76,
    92,13,50,14,206,250,73,177,167,243,240,220,12,247,94,227,
    67,144,147,61,145,125,31,163,149,176,148,115,220,217,49,241,
    217,164,186,85,126,114,154,133,65,223,106,244,219,152,103,212,
    53,166,95,227,64,239,34,235,11,117,81,51,232,133,183,41,
    26,162,41,42,98,190,89,55,235,181,122,213,196,226,65,51,
    203,162,97,214,27,243,226,125,254,215,176,204,52,140,181,249,
    186,248,31,206,133,60,213,
};

EmbeddedPython embedded_m5_internal_param_Pl050(
    "m5/internal/param_Pl050.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_Pl050.py",
    "m5.internal.param_Pl050",
    data_m5_internal_param_Pl050,
    2327,
    7131);

} // anonymous namespace