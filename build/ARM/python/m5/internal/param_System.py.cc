#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_System[] = {
    120,156,197,26,107,115,219,198,113,1,62,36,210,162,37,89,
    47,63,36,139,182,163,152,113,99,209,118,44,59,141,93,53,
    142,149,105,147,25,43,41,212,25,219,106,26,4,2,78,18,
    40,18,224,0,39,219,204,72,31,90,101,218,126,232,76,127,
    68,167,31,250,165,191,162,63,163,255,165,221,221,195,129,32,
    41,201,106,39,165,45,98,231,176,183,183,119,251,190,59,216,
    133,228,95,1,159,79,171,0,241,63,12,0,15,127,6,52,
    1,90,6,108,26,96,8,3,188,41,216,43,64,116,31,188,
    2,252,0,176,105,130,48,225,8,27,57,248,141,9,193,24,
    143,41,66,51,199,24,3,58,101,16,121,216,44,192,243,96,
    18,242,162,8,123,101,136,190,3,195,48,2,3,94,120,35,
    224,141,194,15,200,29,27,37,102,56,10,94,153,27,37,240,
    206,113,163,12,157,9,16,231,96,19,153,143,192,102,5,89,
    221,66,86,231,153,213,63,137,149,135,61,211,224,85,136,28,
    215,242,146,40,243,68,201,115,156,103,46,227,122,101,19,176,
    57,169,219,23,50,237,169,76,123,58,211,158,201,180,103,51,
    237,185,76,251,98,166,125,41,211,190,204,237,113,16,147,208,
    184,2,141,121,104,44,192,54,42,113,34,93,233,85,16,57,
    104,44,194,230,34,8,252,93,133,35,212,179,55,153,25,81,
    229,17,23,210,17,215,120,196,117,216,188,14,2,127,215,212,
    136,34,108,212,102,208,118,254,191,241,95,13,109,7,114,12,
    193,43,17,197,126,24,216,126,176,29,250,38,245,23,9,144,
    165,93,2,35,137,201,159,146,201,255,14,108,111,207,76,76,
    126,8,200,216,32,89,154,38,28,114,227,208,132,78,13,14,
    12,104,228,193,203,193,1,78,83,160,5,236,24,112,100,194,
    55,57,34,56,68,152,71,35,93,133,188,84,246,110,176,145,
    20,167,17,56,44,192,65,1,54,94,28,152,132,216,43,65,
    244,55,248,126,158,153,142,50,83,19,14,16,230,225,40,15,
    135,69,120,142,68,136,106,148,72,124,227,197,1,74,138,152,
    141,90,30,87,187,158,17,151,68,241,252,40,112,90,66,86,
    176,109,183,157,200,105,217,27,157,88,138,86,173,172,73,194,
    120,185,237,200,93,139,199,228,72,25,173,182,100,94,97,32,
    228,57,108,108,251,129,103,183,66,111,191,41,228,40,49,178,
    183,253,166,176,109,238,252,162,213,14,35,249,121,20,133,145,
    69,250,100,100,51,116,210,17,164,77,183,25,198,162,70,179,
    241,52,22,177,151,68,189,221,102,142,180,0,94,39,13,246,
    68,236,70,126,91,162,153,20,71,162,38,110,53,50,16,131,
    248,37,130,250,110,216,18,117,20,169,190,22,190,14,104,202,
    184,190,35,90,43,183,99,233,108,53,197,109,199,217,190,115,
    247,161,112,28,239,161,87,223,218,247,155,94,253,137,245,172,
    222,238,200,221,48,168,183,86,234,126,32,5,170,167,89,207,
    42,102,25,251,47,208,20,175,253,29,219,103,225,236,93,209,
    108,139,136,116,24,95,166,233,141,9,99,204,40,26,57,163,
    102,84,176,85,192,39,103,204,155,231,140,117,159,196,115,73,
    100,114,171,156,118,164,191,2,155,12,45,190,103,66,52,79,
    110,210,192,159,65,118,69,103,217,160,62,147,251,126,69,122,
    81,216,70,142,140,175,144,7,236,90,232,99,72,249,152,172,
    29,0,251,71,1,26,69,80,126,131,238,166,28,41,234,16,
    68,114,98,99,34,243,60,196,127,233,229,16,76,0,234,29,
    83,2,162,102,113,170,223,179,43,110,212,104,225,235,236,21,
    114,215,143,81,171,172,123,106,115,240,108,160,78,190,238,124,
    181,213,16,174,140,23,17,241,50,220,175,186,78,16,132,178,
    234,120,94,213,145,50,242,183,246,165,136,171,50,172,46,197,
    181,18,25,123,82,59,86,202,175,211,214,142,68,70,71,71,
    82,47,158,239,74,124,153,226,23,214,127,44,36,58,197,110,
    232,197,136,39,22,59,66,90,180,72,121,30,193,19,61,29,
    123,95,173,168,125,37,22,205,109,89,102,183,115,226,216,230,
    233,8,207,30,70,163,95,57,205,125,33,137,30,93,69,226,
    172,212,84,19,13,205,199,46,146,148,90,72,82,156,29,132,
    129,215,193,53,250,238,18,77,127,145,61,109,12,200,215,102,
    208,207,70,16,22,161,130,126,55,97,186,36,78,62,241,50,
    246,176,89,18,30,216,234,70,146,48,208,219,142,48,173,212,
    76,206,11,44,23,7,95,149,90,52,216,34,103,182,174,16,
    152,39,176,160,69,31,134,252,149,126,249,239,209,156,38,11,
    237,230,18,241,210,0,90,239,9,160,75,221,0,194,20,184,
    65,129,96,82,184,116,3,33,71,10,136,86,19,175,167,16,
    67,195,99,119,198,215,89,45,214,4,137,91,212,110,106,145,
    239,101,29,112,39,227,128,22,89,132,189,207,186,116,146,10,
    23,223,141,10,119,148,10,87,104,206,177,196,111,42,236,47,
    101,195,37,163,155,137,66,89,153,107,216,232,204,145,50,179,
    106,156,195,130,246,60,168,112,101,226,234,198,123,5,149,53,
    148,118,85,35,79,30,182,157,131,217,164,226,196,20,228,237,
    40,124,211,169,134,219,85,9,122,13,143,151,226,229,165,248,
    17,166,129,234,42,39,22,149,8,84,168,71,162,29,97,72,
    151,248,69,133,169,205,33,107,39,101,3,213,77,213,155,173,
    196,74,230,172,20,203,136,146,209,208,52,92,78,53,76,11,
    126,68,19,150,89,189,57,152,195,167,108,240,170,236,144,51,
    34,239,31,184,23,159,207,72,209,36,171,0,218,41,90,27,
    106,205,44,14,9,102,221,236,241,148,33,8,99,221,66,238,
    107,58,200,138,144,250,5,61,57,90,46,249,253,31,129,119,
    87,6,252,1,200,7,208,212,73,164,112,76,210,67,166,156,
    34,242,111,129,147,205,49,69,205,84,17,103,38,233,8,3,
    50,126,200,164,170,198,125,9,127,202,100,170,163,28,24,84,
    143,114,201,254,41,91,143,242,105,140,178,243,156,169,230,228,
    123,131,153,12,180,235,196,68,166,194,54,151,134,109,55,233,
    165,123,31,204,69,195,240,171,81,53,149,77,171,250,162,235,
    85,148,236,175,24,83,102,198,87,62,36,112,59,117,19,67,
    227,254,207,11,92,236,207,206,153,234,100,171,140,248,75,90,
    69,158,215,61,94,228,10,170,134,167,97,80,208,97,176,150,
    134,129,224,4,253,3,239,170,9,154,100,243,35,211,192,163,
    17,110,70,232,84,146,7,81,128,205,162,62,77,141,208,169,
    65,157,130,48,132,200,150,236,182,250,225,116,70,105,176,167,
    30,176,150,214,149,254,82,71,80,54,38,240,102,104,233,131,
    204,252,184,233,180,182,60,103,245,183,52,29,205,233,234,152,
    51,181,0,19,89,1,40,94,140,147,100,224,215,59,90,144,
    87,67,75,29,119,145,123,42,0,7,138,23,186,156,47,126,
    189,43,170,45,209,218,194,99,213,174,223,174,110,55,157,29,
    182,80,46,17,240,43,45,160,100,163,247,215,223,152,146,210,
    122,88,117,195,0,115,251,190,43,195,168,234,9,60,112,8,
    175,122,187,202,133,161,234,199,85,103,11,123,29,87,42,255,
    239,13,99,222,218,57,209,78,204,62,184,247,154,154,67,181,
    176,141,135,73,31,55,174,223,166,22,214,238,153,90,184,162,
    20,208,48,244,110,36,107,94,62,227,76,167,241,67,197,252,
    153,104,133,81,231,89,232,9,37,241,101,24,242,182,162,103,
    13,223,245,8,102,38,214,253,69,198,178,74,48,116,221,63,
    191,69,182,184,71,182,180,184,91,247,135,45,96,207,66,28,
    45,160,58,176,164,245,153,79,25,42,13,226,6,7,143,128,
    178,163,138,209,3,2,31,16,160,173,186,245,49,129,79,96,
    216,5,189,142,220,191,129,164,122,20,177,114,148,240,112,80,
    50,212,81,141,169,190,166,17,241,96,70,254,151,113,134,140,
    172,174,169,146,188,92,36,74,49,66,55,18,4,75,84,212,
    55,203,26,121,142,225,24,35,43,26,121,158,225,56,35,39,
    52,114,146,225,5,70,78,105,228,52,195,25,70,206,106,228,
    28,195,139,140,188,164,145,151,25,94,97,228,188,70,46,48,
    188,202,200,69,141,172,50,188,198,200,235,26,121,131,225,123,
    140,92,210,200,247,25,222,100,100,77,35,63,96,120,139,145,
    63,209,200,15,25,222,102,228,178,70,214,25,222,97,228,93,
    141,188,199,240,35,70,222,215,200,21,134,15,24,249,80,35,
    63,102,248,83,70,126,162,145,143,24,62,102,228,207,52,114,
    149,225,207,25,249,169,46,144,79,24,249,25,108,62,213,149,
    114,237,71,169,148,92,101,134,90,95,182,225,199,44,144,214,
    189,119,182,126,235,35,72,54,211,39,21,199,255,178,54,76,
    245,197,179,237,70,194,145,239,162,52,80,133,85,147,239,30,
    91,19,88,160,135,169,64,71,124,134,232,76,103,206,144,108,
    60,227,57,110,248,241,156,121,192,50,219,166,58,106,118,253,
    49,159,138,78,150,15,196,107,59,43,190,58,71,210,82,156,
    118,91,4,94,166,140,76,14,211,232,15,148,87,166,155,119,
    60,16,78,227,51,24,118,36,78,70,58,54,91,33,13,180,
    97,26,144,29,245,119,218,116,181,207,33,91,237,44,58,223,
    170,250,214,45,109,171,169,41,22,251,189,144,47,121,241,8,
    22,217,45,39,222,163,109,203,91,105,176,234,114,77,237,69,
    203,185,254,97,123,184,122,209,100,150,39,245,17,43,222,240,
    241,171,172,14,196,136,227,238,10,187,137,59,73,59,246,191,
    23,204,235,173,68,196,116,156,156,186,23,47,47,245,15,196,
    168,166,139,10,197,246,228,94,226,71,91,68,141,144,239,245,
    147,190,14,163,61,27,157,216,118,247,218,210,118,195,253,64,
    50,207,51,17,18,123,74,14,199,244,201,133,126,6,91,97,
    40,237,48,166,36,20,243,20,167,83,16,111,218,72,100,145,
    199,74,26,70,190,136,79,212,131,234,205,232,129,17,242,253,
    99,197,219,18,59,126,208,175,137,51,146,210,20,51,90,23,
    253,189,242,74,63,147,184,211,218,10,155,252,29,131,38,57,
    173,159,56,83,136,119,81,131,228,124,2,80,193,118,44,187,
    76,191,102,215,69,13,42,14,83,172,151,174,237,228,94,173,
    86,141,24,52,105,176,223,82,250,242,189,19,140,222,67,161,
    141,158,69,202,165,99,13,224,182,247,227,126,83,157,141,146,
    38,153,214,150,234,235,28,212,28,133,78,228,4,59,137,143,
    157,214,175,53,219,69,201,155,167,57,79,123,31,229,179,197,
    27,95,173,254,172,180,52,205,108,159,167,117,187,79,137,92,
    30,125,150,16,207,16,14,132,120,183,111,208,150,202,100,116,
    206,194,197,28,107,237,30,10,109,237,44,242,212,112,235,19,
    224,140,164,199,68,102,70,136,123,253,76,248,139,91,172,142,
    139,220,118,195,32,16,46,125,14,204,204,253,63,12,163,117,
    92,195,97,111,165,228,202,206,90,247,4,6,149,232,217,125,
    240,150,36,249,58,224,161,135,69,97,199,182,213,53,48,190,
    55,109,123,184,103,192,199,192,39,57,136,73,33,116,6,164,
    19,224,204,153,254,74,249,146,193,215,40,125,223,193,213,218,
    111,128,190,18,237,196,22,97,172,241,116,43,192,159,108,245,
    129,152,118,13,124,145,191,238,180,212,151,56,254,194,100,145,
    170,173,27,122,55,193,181,90,221,147,243,69,180,186,250,199,
    173,29,223,241,240,149,142,181,76,120,10,240,214,202,178,22,
    120,89,80,46,234,30,213,249,171,113,107,133,13,153,37,123,
    146,220,16,41,74,251,149,160,171,36,46,248,89,42,165,189,
    94,218,129,25,21,17,118,170,15,157,188,169,25,236,127,218,
    12,221,61,225,37,52,11,39,211,172,133,45,7,241,199,207,
    178,225,235,89,38,251,250,189,136,70,205,244,97,99,17,249,
    78,147,182,36,243,253,10,192,221,148,69,73,79,203,62,202,
    53,70,245,242,166,166,95,46,178,64,250,198,151,9,153,147,
    6,59,120,132,193,138,47,17,15,79,73,147,93,56,121,222,
    96,109,202,14,27,106,36,168,203,86,245,241,104,149,254,27,
    65,252,37,2,250,120,90,26,47,25,69,147,62,209,231,140,
    178,81,49,242,198,88,165,148,43,21,75,133,28,70,11,97,
    166,140,114,174,84,30,51,6,255,22,139,37,163,108,46,46,
    148,140,255,0,79,211,237,124,
};

EmbeddedPython embedded_m5_internal_param_System(
    "m5/internal/param_System.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_System.py",
    "m5.internal.param_System",
    data_m5_internal_param_System,
    2760,
    9233);

} // anonymous namespace