#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_Uart8250[] = {
    120,156,197,88,109,111,27,199,17,158,189,35,41,145,18,45,
    201,178,252,42,71,116,83,37,116,80,139,142,91,213,65,99,
    24,117,172,0,117,1,43,238,49,133,29,165,232,245,116,183,
    164,142,186,23,226,110,37,135,129,244,165,50,218,126,235,143,
    40,250,161,255,163,255,171,157,153,189,59,30,41,9,13,208,
    132,145,200,197,112,95,102,119,102,158,121,217,117,33,251,171,
    226,247,215,45,128,212,48,0,60,252,8,8,0,66,1,123,
    2,132,20,224,173,194,97,21,146,95,128,87,133,119,0,123,
    6,72,3,206,144,48,225,107,3,162,69,94,83,131,192,228,
    30,1,163,6,200,10,236,85,225,117,180,2,21,89,131,195,
    6,36,127,2,33,68,36,224,141,55,7,222,60,188,67,238,
    72,212,153,225,60,120,13,38,234,224,45,48,209,128,209,50,
    200,5,216,67,230,115,176,215,68,86,31,33,171,43,204,234,
    223,196,202,195,145,107,224,53,105,58,158,229,43,154,89,161,
    153,188,199,21,230,178,148,159,108,25,246,86,114,250,106,137,
    94,45,209,215,74,244,90,137,190,94,162,111,148,232,155,37,
    250,86,137,190,93,162,239,148,232,245,18,125,183,68,191,87,
    162,55,74,116,171,68,223,99,122,9,228,10,12,126,2,131,
    247,97,240,83,232,161,97,150,11,233,55,65,154,48,248,0,
    246,62,0,137,159,77,56,67,219,121,43,165,21,31,242,138,
    171,197,138,54,175,184,15,123,247,65,226,167,173,87,212,160,
    219,94,67,60,248,255,193,191,182,64,74,45,98,115,44,147,
    212,143,35,219,143,122,177,111,208,120,141,26,66,143,75,205,
    92,6,163,231,4,163,127,1,99,200,51,50,24,157,2,50,
    22,36,75,96,192,41,19,167,6,140,218,112,34,96,80,1,
    207,132,19,220,166,74,7,232,11,56,51,224,15,38,77,56,
    197,182,130,134,127,15,42,74,99,104,192,134,215,156,230,224,
    180,10,39,85,232,190,57,49,168,227,176,14,201,63,225,219,
    117,102,58,207,76,13,56,193,182,2,103,21,56,173,193,107,
    156,132,93,131,58,137,47,222,156,160,164,216,211,109,87,240,
    180,187,37,113,73,20,207,79,34,39,148,106,9,105,123,232,
    36,78,104,255,222,73,212,39,143,182,31,182,27,249,164,56,
    221,26,58,234,192,226,85,38,169,35,28,42,230,22,71,82,
    45,32,209,243,35,207,14,99,239,40,144,106,158,88,217,61,
    63,144,182,205,131,47,194,97,156,168,207,147,36,78,44,210,
    40,119,6,177,83,172,32,125,186,65,156,202,54,237,198,219,
    88,196,94,209,236,222,144,57,210,1,248,164,180,216,147,169,
    155,248,67,133,134,210,28,105,54,113,107,147,137,184,73,191,
    198,166,115,16,135,178,131,66,117,118,226,183,17,109,153,118,
    250,50,220,126,144,42,103,63,144,15,28,167,247,240,227,199,
    210,113,188,199,94,103,255,200,15,188,206,51,235,101,103,56,
    82,7,113,212,9,183,59,126,164,36,42,40,232,76,170,102,
    11,103,92,165,77,222,250,125,219,103,241,236,3,25,12,101,
    210,164,222,219,116,0,177,44,22,69,77,152,162,45,154,72,
    85,241,107,138,117,99,65,236,250,36,160,75,66,19,180,204,
    28,76,255,0,54,27,90,253,208,128,100,157,160,50,192,143,
    32,219,34,96,186,52,102,240,216,239,72,51,186,119,96,18,
    0,116,231,9,195,11,113,134,51,159,144,197,35,96,140,84,
    97,80,3,141,29,132,156,6,83,50,162,22,167,19,27,3,
    153,87,32,253,251,36,135,104,25,80,243,24,106,176,235,58,
    110,245,103,134,99,183,77,7,223,101,92,168,3,63,69,189,
    178,246,137,102,7,234,162,78,94,141,190,216,31,72,87,165,
    27,216,241,85,124,212,114,157,40,138,85,203,241,188,150,163,
    84,226,239,31,41,153,182,84,220,218,76,219,117,50,247,74,
    14,173,130,223,104,152,67,137,204,142,80,210,63,60,223,85,
    248,99,149,127,176,254,83,169,16,22,7,177,151,98,63,177,
    232,75,101,209,33,213,21,108,158,229,219,49,254,218,181,28,
    45,169,12,122,170,193,192,115,210,212,230,237,168,159,49,70,
    171,143,157,224,72,42,154,143,96,81,184,43,145,122,163,25,
    162,236,38,201,153,139,73,170,179,163,56,242,70,120,74,223,
    221,164,3,220,100,172,45,2,161,109,13,145,54,135,109,13,
    154,136,188,101,195,37,129,42,25,206,24,99,215,73,124,96,
    187,139,44,108,32,222,206,48,184,180,13,142,14,44,25,59,
    96,139,40,90,108,17,156,173,59,212,172,83,115,55,23,126,
    54,26,104,78,107,224,17,237,106,176,216,174,153,9,88,56,
    209,238,132,19,221,26,59,17,134,194,46,57,131,65,46,51,
    118,6,147,84,144,60,205,144,79,110,134,198,199,225,18,222,
    89,49,214,50,9,92,203,161,106,17,254,202,32,236,151,64,
    104,145,77,24,129,214,173,203,148,184,241,99,41,177,175,149,
    184,77,187,46,102,216,105,50,102,26,194,37,195,27,153,74,
    89,157,59,72,140,110,144,58,203,138,188,129,169,237,117,212,
    228,28,197,121,142,43,17,29,59,180,126,53,81,33,148,245,
    76,184,158,229,158,148,92,125,152,196,223,140,90,113,175,165,
    32,63,195,147,205,116,107,51,253,20,131,65,235,41,135,23,
    29,14,180,195,39,114,152,160,99,215,249,135,118,86,155,29,
    215,206,210,7,42,156,242,56,219,137,213,204,177,41,85,9,
    133,164,25,234,184,81,232,152,142,252,41,109,217,96,5,155,
    112,3,191,13,193,231,178,99,142,140,92,75,240,40,126,63,
    35,85,147,180,18,168,18,181,186,250,212,44,16,137,102,125,
    56,129,150,153,136,99,125,132,252,119,114,87,171,65,129,13,
    250,154,116,96,66,255,95,129,107,45,1,127,1,194,1,154,
    59,243,23,246,76,250,146,57,87,105,250,31,129,131,206,5,
    233,205,208,126,103,100,97,9,221,50,125,204,83,117,182,251,
    45,252,173,20,177,206,76,16,148,153,204,172,154,42,103,166,
    74,225,169,12,160,239,148,125,42,147,46,77,38,58,112,82,
    154,166,157,215,44,156,119,28,252,138,58,8,35,210,108,176,
    53,175,55,179,233,92,47,198,200,162,176,127,71,172,26,37,
    188,252,140,154,7,5,84,68,222,247,131,31,113,99,58,78,
    151,50,149,173,99,227,111,232,28,21,62,249,82,141,101,202,
    25,20,238,80,205,221,225,81,225,14,146,131,245,59,174,180,
    169,53,200,242,103,134,192,43,24,22,39,116,251,169,128,172,
    194,94,141,28,135,75,73,145,249,149,200,195,24,133,191,137,
    76,192,122,217,213,26,43,140,175,237,74,205,55,51,12,27,
    164,134,39,129,19,238,123,206,83,143,54,164,93,221,220,211,
    140,92,132,229,178,8,228,37,226,50,41,248,231,195,92,148,
    227,25,134,140,143,129,111,213,90,4,118,16,47,118,57,78,
    124,121,32,91,161,12,247,241,114,117,224,15,91,189,192,233,
    179,149,204,76,196,47,114,17,21,155,121,58,251,166,20,140,
    118,227,150,27,71,24,215,143,92,21,39,45,79,226,165,67,
    122,173,7,45,78,10,45,63,109,57,251,56,234,184,74,163,
    126,210,125,185,184,115,146,126,202,117,220,225,91,34,103,108,
    101,27,47,149,62,22,175,50,87,145,190,238,20,17,158,235,
    85,237,66,152,36,241,50,161,70,58,152,253,146,154,251,212,
    108,194,236,19,65,7,244,213,55,37,197,213,48,222,212,5,
    159,52,159,243,138,86,164,231,61,248,229,119,241,96,253,124,
    146,249,113,45,127,119,153,3,57,79,247,215,189,6,61,10,
    80,207,2,57,247,220,255,235,220,236,22,51,118,136,195,239,
    213,167,173,71,63,162,4,214,207,33,203,251,151,249,179,40,
    139,215,212,254,60,16,121,105,93,150,141,175,237,107,231,96,
    100,187,137,116,148,212,214,186,61,83,89,57,48,232,237,195,
    194,102,185,64,197,5,227,113,33,212,25,151,60,163,107,165,
    178,151,77,40,94,99,125,130,165,241,9,203,109,27,186,58,
    30,227,178,82,136,79,79,8,145,124,107,79,170,64,23,191,
    116,24,103,56,148,145,55,46,108,121,100,134,198,167,184,19,
    193,184,218,192,42,246,26,126,207,187,32,137,84,146,144,141,
    87,45,156,110,182,102,100,200,166,185,1,219,116,191,24,135,
    88,139,204,167,131,106,17,79,173,95,65,30,99,11,80,122,
    50,144,74,78,25,70,209,186,236,190,231,73,76,53,241,8,
    175,34,92,212,227,239,192,182,103,29,153,63,1,14,30,250,
    18,69,145,185,134,177,185,94,169,11,78,123,83,175,151,250,
    92,148,72,117,233,58,74,45,246,233,165,66,112,126,102,203,
    19,15,227,156,46,93,187,78,168,223,78,248,69,192,186,71,
    205,251,185,2,25,166,250,70,195,23,6,125,77,67,76,115,
    78,230,20,108,109,81,63,61,60,132,219,91,185,48,91,99,
    97,248,145,47,220,86,183,47,156,241,165,76,66,31,127,170,
    187,23,14,191,136,84,242,60,198,38,14,248,169,228,252,140,
    238,40,69,127,86,119,166,6,101,116,20,218,47,101,24,39,
    163,151,177,39,213,189,169,241,103,89,41,161,167,216,199,146,
    106,14,213,186,112,139,201,185,231,182,210,147,112,80,191,137,
    113,161,124,126,252,121,16,187,135,210,203,230,92,44,45,207,
    217,137,67,7,251,47,222,165,235,231,187,172,76,141,123,9,
    173,90,155,234,77,101,226,59,129,255,173,84,235,211,10,240,
    188,196,114,162,190,204,101,191,216,62,175,2,71,245,226,36,
    188,68,53,159,57,169,239,190,242,227,29,121,236,187,242,146,
    67,143,199,9,129,249,32,223,167,75,48,169,100,185,130,235,
    145,137,204,193,30,153,200,190,143,150,78,120,25,141,100,193,
    148,124,132,247,157,202,51,229,53,51,246,91,93,242,235,199,
    139,167,244,156,157,190,193,134,30,240,234,75,117,81,51,232,
    161,216,20,13,209,20,21,177,216,172,155,245,90,189,106,162,
    111,83,207,170,104,152,245,198,162,248,95,255,27,24,5,26,
    198,70,163,46,254,11,92,132,4,190,
};

EmbeddedPython embedded_m5_internal_param_Uart8250(
    "m5/internal/param_Uart8250.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_Uart8250.py",
    "m5.internal.param_Uart8250",
    data_m5_internal_param_Uart8250,
    2250,
    6909);

} // anonymous namespace
