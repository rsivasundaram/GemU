#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_ArmISA_vector[] = {
    120,156,205,92,121,115,219,198,21,127,0,15,153,180,100,73,
    214,233,35,49,115,40,97,210,218,74,210,184,153,38,30,79,
    156,186,135,51,173,146,66,153,218,97,210,32,16,177,146,32,
    147,0,3,64,178,233,74,109,26,185,73,122,77,122,31,211,
    63,59,253,163,223,163,223,164,31,164,125,191,183,0,69,138,
    116,39,30,74,132,69,114,103,9,128,251,222,239,93,187,239,
    97,161,58,37,127,5,254,188,94,33,138,222,49,136,92,126,
    27,212,32,106,26,84,51,200,80,6,185,51,116,187,64,225,
    203,228,22,232,62,81,205,36,101,210,62,119,114,244,158,73,
    254,184,252,166,72,141,156,28,49,168,93,38,149,167,90,129,
    110,250,211,148,87,69,186,93,166,240,67,50,12,195,55,232,
    150,59,70,238,9,186,207,163,115,167,36,3,158,32,183,44,
    157,18,185,39,165,83,166,246,20,169,147,84,227,193,199,168,
    54,193,67,61,207,67,157,146,161,254,141,161,92,62,51,75,
    238,4,46,103,94,222,197,149,121,92,41,52,78,201,40,147,
    228,78,98,148,117,198,48,213,185,144,7,206,209,214,52,213,
    166,73,241,123,138,246,25,102,2,225,52,213,102,82,56,179,
    93,253,185,174,254,188,244,167,211,129,79,119,6,94,144,129,
    23,169,182,72,138,223,11,122,224,34,173,86,231,88,194,222,
    127,249,175,202,18,166,120,156,155,29,21,70,94,224,219,158,
    191,30,120,38,206,23,209,64,31,117,52,99,137,98,190,9,
    197,252,139,68,43,174,153,40,102,143,120,96,3,12,53,76,
    218,147,206,158,73,237,42,237,26,180,149,39,55,71,187,76,
    166,0,6,54,12,218,55,233,253,28,46,216,227,54,207,162,
    124,156,242,177,214,202,150,136,82,143,52,70,123,5,218,45,
    208,234,173,93,19,7,110,151,40,252,39,221,59,47,131,158,
    144,65,77,218,229,54,79,251,121,218,43,210,77,190,136,15,
    109,149,0,223,184,181,203,72,249,200,106,53,207,220,174,116,
    193,5,20,215,11,125,167,169,226,83,220,183,175,133,205,27,
    171,215,236,29,85,143,131,176,90,78,175,9,162,75,45,39,
    222,180,228,71,57,72,163,217,138,101,176,192,87,241,73,238,
    172,123,190,107,55,3,119,187,161,226,19,24,201,94,247,26,
    202,182,229,228,141,102,43,8,227,111,133,97,16,90,16,168,
    28,108,4,78,231,23,16,103,189,17,68,170,10,106,66,198,
    194,240,49,174,94,111,201,136,96,64,24,197,143,93,21,213,
    67,175,21,179,158,244,136,184,26,163,85,161,33,105,162,26,
    55,203,155,65,83,45,135,78,115,249,122,112,199,7,201,104,
    121,67,53,47,95,140,98,103,173,161,46,58,206,250,11,47,
    190,162,28,199,125,197,93,94,219,246,26,238,242,53,235,251,
    203,173,118,188,25,248,203,205,203,203,158,31,43,150,79,99,
    185,71,50,151,248,130,211,160,113,199,219,176,61,65,103,111,
    170,70,75,133,19,56,122,22,244,141,41,99,220,40,26,57,
    163,106,76,112,175,192,159,156,113,222,60,105,172,120,192,87,
    7,102,24,86,46,53,165,127,144,40,141,117,126,219,164,240,
    60,12,101,139,223,6,52,203,230,178,138,115,166,156,251,1,
    4,163,143,110,229,160,126,125,112,87,140,139,173,140,175,188,
    2,125,251,36,22,82,160,173,34,105,203,97,131,211,166,20,
    182,209,242,229,24,198,228,193,243,20,125,209,59,130,63,69,
    44,120,118,93,62,52,207,164,62,17,99,92,173,130,241,21,
    49,139,120,211,139,88,172,34,124,244,197,125,86,89,38,111,
    183,223,90,219,98,73,69,23,248,192,187,193,118,165,238,248,
    126,16,87,28,215,173,56,113,28,122,107,219,177,138,42,113,
    80,89,138,170,37,104,123,58,181,172,206,120,237,86,106,73,
    208,58,91,146,254,226,122,245,152,191,204,200,23,145,127,164,
    98,182,138,205,192,141,248,56,134,216,80,177,5,38,197,164,
    175,165,228,196,252,170,197,212,88,34,213,88,143,203,98,119,
    78,20,217,66,14,199,197,196,240,235,29,167,177,173,98,92,
    207,182,18,51,85,116,53,161,209,25,217,34,96,166,40,33,
    57,219,15,124,183,205,76,122,245,37,208,95,20,83,27,39,
    24,219,28,27,218,24,183,69,154,96,195,155,50,235,192,147,
    79,204,76,76,108,30,232,73,212,110,36,49,131,205,109,159,
    35,75,213,148,208,32,192,196,253,42,232,225,199,22,172,217,
    58,135,230,60,154,199,82,236,35,17,192,196,97,1,188,4,
    162,166,160,174,231,18,124,29,23,90,233,113,161,51,7,46,
    196,97,112,21,174,96,194,97,14,92,33,7,9,132,87,19,
    187,135,147,177,234,249,116,151,181,139,92,172,41,224,45,166,
    134,106,193,250,186,77,112,163,203,4,45,168,68,236,207,58,
    243,32,25,94,200,72,134,27,90,134,151,65,116,60,177,156,
    9,177,152,178,81,135,218,205,68,162,34,205,235,220,105,47,
    64,154,221,114,92,224,89,237,166,63,33,211,147,76,113,50,
    173,235,192,161,197,171,59,121,216,216,122,142,230,147,105,39,
    130,159,183,194,224,110,187,18,172,87,98,74,121,184,178,20,
    93,90,138,94,227,72,80,185,42,177,69,199,2,237,237,161,
    106,133,236,213,37,249,162,61,213,22,175,181,147,169,131,229,
    141,41,92,212,36,82,150,192,20,197,33,226,209,232,68,92,
    238,136,24,28,191,6,138,101,145,111,142,22,248,83,54,132,
    45,59,144,168,40,171,8,57,203,159,55,32,105,128,85,132,
    85,157,181,170,153,22,60,64,102,61,219,99,43,163,64,99,
    61,207,195,95,79,253,172,72,29,203,192,39,7,126,97,250,
    159,145,44,178,12,250,148,96,5,172,236,196,89,196,45,241,
    129,50,103,112,249,7,36,1,103,192,204,102,106,167,51,147,
    144,196,62,25,189,34,151,234,137,238,77,250,188,43,90,237,
    231,200,192,164,148,75,150,81,221,147,82,190,227,166,98,62,
    95,106,226,201,247,250,51,52,180,233,68,184,76,123,110,174,
    227,185,7,129,175,179,2,226,112,52,18,203,58,161,105,217,
    96,235,198,129,93,33,226,159,51,102,204,46,107,249,42,154,
    139,29,67,49,210,99,199,205,225,133,195,33,186,107,142,178,
    117,88,252,46,216,200,11,227,147,69,153,150,245,42,225,6,
    143,233,240,48,29,143,40,164,30,241,159,142,71,40,137,214,
    247,101,153,141,214,132,246,247,77,131,51,26,94,155,32,153,
    200,147,42,80,173,72,106,12,171,97,164,42,133,36,85,225,
    37,62,58,39,233,32,181,145,180,5,217,141,129,148,36,201,
    110,78,165,217,13,231,37,227,210,153,74,18,24,78,69,146,
    148,229,52,82,22,116,102,146,148,165,54,139,92,3,157,185,
    36,215,168,205,35,35,67,103,1,41,16,58,139,228,206,73,
    231,12,185,243,210,57,75,238,130,116,206,193,217,49,89,136,
    127,165,31,137,188,136,216,61,115,151,104,115,69,235,185,99,
    177,218,24,209,220,29,93,164,131,61,94,105,56,205,53,215,
    185,186,10,122,32,90,79,163,131,153,34,152,234,70,0,207,
    54,30,4,66,190,190,144,34,217,25,93,148,123,17,102,152,
    34,16,159,118,131,186,132,182,119,54,85,165,169,154,107,156,
    8,110,122,173,202,122,195,217,16,29,229,18,132,111,165,8,
    99,177,202,195,171,133,8,241,115,37,168,212,3,159,39,162,
    109,208,171,184,138,51,36,229,86,46,86,100,22,171,120,81,
    197,89,227,179,78,61,214,158,218,27,113,100,41,234,132,27,
    145,172,58,111,223,65,119,180,58,182,57,255,245,120,165,253,
    78,71,199,125,86,10,30,221,3,227,20,24,133,78,232,57,
    75,163,156,178,160,204,155,125,172,118,204,113,66,43,107,203,
    72,215,121,221,182,40,9,228,108,95,80,178,37,3,200,2,
    12,102,154,91,41,24,157,45,118,166,223,110,7,235,32,210,
    83,103,63,168,153,126,80,158,95,15,187,86,75,248,133,63,
    58,203,130,201,128,131,119,251,34,199,208,192,92,213,13,204,
    250,198,40,53,38,184,192,64,173,199,4,83,92,223,25,132,
    235,215,131,172,112,97,0,46,206,171,29,191,174,186,176,189,
    60,82,108,136,6,41,19,239,13,137,111,128,151,169,143,182,
    157,70,102,224,16,175,132,131,247,135,9,30,3,204,177,30,
    180,218,25,196,14,177,68,208,254,209,17,227,241,213,221,56,
    43,60,160,253,193,48,120,6,184,149,45,136,108,59,11,76,
    73,89,75,232,219,71,140,171,21,170,29,47,216,142,178,194,
    149,210,255,112,200,72,49,223,15,205,113,119,50,13,132,72,
    16,19,30,156,33,209,205,13,50,72,245,17,155,99,86,224,
    138,98,147,96,97,237,56,176,249,42,115,108,96,161,62,36,
    182,129,129,196,115,92,55,67,116,201,106,93,152,112,143,5,
    95,180,189,150,61,62,97,66,29,125,84,177,237,108,213,39,
    133,65,205,195,250,113,160,203,86,121,26,157,240,176,65,135,
    103,186,158,76,114,247,32,147,20,238,50,155,151,61,62,109,
    219,155,41,183,85,41,254,117,74,177,250,62,169,148,187,90,
    97,208,82,97,220,214,101,199,175,163,121,14,205,82,79,40,
    116,85,67,197,202,238,213,75,60,69,157,155,7,174,138,226,
    48,104,219,118,34,44,254,129,109,75,14,104,189,138,230,10,
    154,171,104,94,71,243,6,26,84,103,173,111,163,65,105,205,
    122,19,205,247,208,224,14,136,245,54,26,11,13,178,119,235,
    135,104,110,246,72,114,36,105,236,50,232,130,14,170,144,69,
    227,156,81,50,138,252,193,107,156,95,165,7,190,228,142,133,
    30,39,185,35,221,95,32,116,141,47,81,32,212,155,29,146,
    50,97,49,173,11,142,165,117,193,19,168,5,38,187,27,78,
    164,37,195,82,90,50,212,165,193,241,180,52,56,145,150,6,
    79,165,165,193,201,180,52,56,149,150,6,167,211,210,224,233,
    180,52,56,147,150,6,103,211,210,224,92,90,26,156,79,75,
    131,11,105,105,112,49,45,13,158,33,119,49,45,22,158,73,
    138,133,238,89,233,156,39,247,156,116,30,35,247,188,116,30,
    39,247,49,233,92,32,247,113,233,84,200,189,32,157,39,200,
    173,72,231,73,114,159,144,206,83,228,62,41,157,167,201,125,
    74,58,75,228,62,45,157,103,72,61,75,91,85,170,61,71,
    238,146,28,121,30,21,202,103,6,122,236,67,84,40,165,184,
    55,218,74,208,71,157,80,115,20,133,73,235,165,236,0,88,
    95,163,228,126,203,131,138,146,15,153,50,204,31,118,45,137,
    119,136,73,89,69,220,148,126,68,15,156,31,202,29,165,237,
    31,170,55,126,64,153,76,21,18,78,227,62,134,31,66,15,
    103,250,244,96,227,190,201,61,21,6,217,36,165,216,86,211,
    197,194,246,48,224,250,141,140,211,138,32,104,100,153,110,107,
    250,59,195,192,154,27,0,171,161,252,108,80,233,197,130,144,
    191,51,12,168,233,62,80,173,160,149,5,32,172,117,153,244,
    221,30,48,15,191,8,62,59,64,73,27,42,142,26,94,61,
    203,28,116,92,212,117,192,72,251,24,96,70,143,10,204,3,
    70,238,29,3,76,94,32,63,26,48,15,24,249,241,144,48,
    7,205,6,60,58,79,141,205,12,81,158,76,81,106,62,118,
    143,1,36,59,196,35,1,178,195,199,222,49,128,140,30,17,
    144,29,62,126,50,36,200,217,62,144,78,171,165,124,55,211,
    10,159,102,225,167,52,196,92,56,211,135,75,53,91,113,38,
    247,113,228,198,20,136,127,60,12,160,211,125,128,34,239,94,
    22,247,180,245,158,91,166,253,179,163,213,79,189,161,156,76,
    50,23,189,101,157,137,127,50,164,47,13,80,209,29,167,149,
    153,39,137,158,152,129,253,97,244,116,174,15,211,6,182,176,
    53,26,65,61,171,76,19,76,247,48,113,255,104,13,113,77,
    109,120,126,86,134,40,196,127,62,12,160,254,44,0,225,60,
    163,44,128,73,127,58,12,152,254,233,41,204,76,61,152,153,
    52,245,207,134,129,212,31,37,194,140,20,132,0,1,218,159,
    15,3,167,191,64,192,153,159,189,230,212,111,103,118,223,58,
    161,255,11,26,46,156,15,88,66,132,78,148,221,77,107,89,
    71,128,131,95,246,0,203,117,3,123,245,0,24,163,210,79,
    172,205,118,109,232,151,93,218,198,77,127,10,155,254,119,101,
    179,182,109,234,125,255,7,229,184,124,79,44,241,213,29,187,
    71,12,186,176,138,146,89,215,190,106,136,66,78,140,174,112,
    135,219,52,191,34,74,247,81,79,26,57,154,229,207,16,247,
    75,15,217,241,118,180,169,13,57,43,141,203,115,22,41,23,
    95,12,227,165,253,182,188,30,6,126,38,219,128,96,198,66,
    252,183,71,27,69,179,10,57,240,22,208,254,221,144,225,102,
    64,38,22,69,222,134,159,109,38,38,44,252,254,200,161,133,
    74,178,151,44,161,105,22,254,112,228,208,60,63,82,97,156,
    41,52,205,194,31,135,132,214,95,35,103,145,169,112,39,219,
    109,91,9,15,127,26,38,120,244,175,89,234,78,203,169,123,
    217,212,7,176,102,73,233,255,185,15,86,207,206,138,71,105,
    143,254,95,83,86,171,95,161,238,109,21,22,246,187,235,141,
    20,7,123,40,112,115,79,238,180,89,31,162,89,67,131,29,
    78,22,54,202,88,216,161,97,109,161,105,160,241,209,180,208,
    132,148,172,52,44,220,237,177,112,67,193,66,29,218,66,1,
    211,66,237,203,250,24,13,50,119,11,105,160,133,108,195,194,
    130,214,194,42,73,111,229,248,13,26,76,159,22,98,180,5,
    151,183,96,63,214,95,208,252,13,205,223,123,28,58,217,232,
    209,187,234,129,72,173,247,122,196,60,18,89,99,59,51,132,
    17,97,107,8,246,94,20,255,207,126,139,7,188,242,253,199,
    100,237,118,232,159,34,104,80,168,123,234,7,227,218,17,54,
    125,144,53,217,145,142,222,23,154,108,150,129,162,101,141,178,
    226,52,245,67,217,242,172,177,245,4,154,167,82,3,144,136,
    164,31,151,148,199,17,245,35,160,188,170,148,199,103,228,105,
    25,235,18,26,108,50,145,122,195,161,141,79,216,90,19,114,
    218,21,241,1,121,140,186,121,249,82,42,174,75,45,135,133,
    158,168,72,254,199,64,243,178,140,209,127,205,170,215,212,143,
    178,203,170,182,251,188,27,58,220,159,59,116,148,195,140,231,
    52,120,138,16,13,12,184,125,209,205,215,104,109,66,63,186,
    165,31,155,189,138,130,112,132,77,69,120,112,188,52,89,50,
    138,38,254,63,65,206,40,27,19,70,222,24,159,40,229,74,
    197,82,33,199,118,131,35,51,70,57,87,42,207,47,148,140,
    178,57,110,232,215,252,115,37,227,127,250,52,150,55,
};

EmbeddedPython embedded_m5_internal_ArmISA_vector(
    "m5/internal/ArmISA_vector.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/ArmISA_vector.py",
    "m5.internal.ArmISA_vector",
    data_m5_internal_ArmISA_vector,
    3278,
    17578);

} // anonymous namespace