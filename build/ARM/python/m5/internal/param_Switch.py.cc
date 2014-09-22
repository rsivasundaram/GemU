#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_Switch[] = {
    120,156,197,88,109,111,219,200,17,158,37,41,218,146,173,216,
    142,227,188,216,78,204,67,97,84,61,52,214,93,90,55,69,
    47,8,122,185,20,232,21,168,239,74,21,72,162,22,229,209,
    228,74,162,76,145,2,185,142,79,7,187,31,234,160,237,183,
    254,136,162,31,250,63,250,191,218,153,89,146,166,149,75,113,
    192,181,58,91,92,44,151,187,51,59,51,207,188,236,6,80,
    252,53,240,249,185,3,144,255,81,0,132,248,19,16,3,76,
    4,244,5,8,41,32,220,132,147,6,100,63,134,176,1,111,
    0,250,6,72,3,46,177,99,194,239,12,72,86,121,141,13,
    177,201,35,2,102,45,144,22,244,27,240,34,217,0,75,218,
    112,210,130,236,11,16,66,36,2,94,134,75,16,46,195,27,
    164,142,157,38,19,92,134,176,197,157,38,132,43,220,105,193,
    108,29,228,10,244,145,248,18,244,219,72,234,125,36,117,131,
    73,253,139,72,133,248,229,22,132,109,154,142,123,121,69,51,
    45,154,201,60,110,48,149,181,114,103,235,208,223,40,251,55,
    107,253,205,90,255,86,173,191,85,235,223,230,254,26,200,13,
    24,223,129,241,93,24,223,131,1,42,101,189,226,188,13,210,
    132,241,14,244,119,64,226,111,27,46,81,111,225,70,109,197,
    46,175,184,89,173,184,207,43,30,64,255,1,72,252,221,215,
    43,108,232,117,182,208,22,209,191,241,175,131,182,0,181,138,
    205,107,153,229,81,154,120,81,50,72,35,131,190,219,212,144,
    229,2,106,150,10,19,126,66,38,252,39,176,253,66,163,48,
    225,5,32,97,65,178,196,6,92,112,231,194,128,89,7,206,
    5,140,45,8,77,56,71,54,13,218,192,80,192,165,1,191,
    55,105,194,5,182,22,42,253,1,88,74,219,111,204,74,215,
    148,150,224,162,1,231,13,232,189,60,55,104,224,164,9,217,
    63,224,171,93,38,186,204,68,13,56,199,214,130,75,11,46,
    108,120,129,147,112,104,220,36,241,197,203,115,148,20,71,122,
    29,11,119,123,84,19,151,68,9,163,44,241,39,82,181,177,
    239,77,253,204,159,120,189,179,72,5,163,78,171,156,146,230,
    7,83,95,141,92,94,99,146,50,38,83,197,180,210,68,170,
    21,236,12,162,36,244,38,105,120,26,75,181,76,132,188,65,
    20,75,207,227,143,159,78,166,105,166,126,145,101,105,230,146,
    62,121,48,78,253,106,5,105,51,136,211,92,118,136,27,179,
    113,137,188,162,217,131,41,83,164,13,240,62,105,113,40,243,
    32,139,166,10,205,164,41,210,108,162,214,33,3,113,147,191,
    194,166,59,74,39,178,139,34,117,159,167,103,9,177,204,187,
    67,57,57,124,152,43,255,56,150,15,125,127,240,193,135,143,
    165,239,135,143,195,238,241,105,20,135,221,143,221,95,119,167,
    51,53,74,147,238,228,176,27,37,74,162,122,226,110,93,49,
    7,248,253,38,177,56,139,134,94,196,194,121,35,25,79,101,
    70,58,204,183,137,189,88,23,171,194,22,166,232,136,54,246,
    26,248,152,98,215,88,17,71,17,137,23,144,200,4,43,179,
    4,210,223,129,77,134,22,63,49,32,219,37,152,140,241,39,
    200,174,8,150,30,125,51,248,219,111,72,47,122,116,108,146,
    241,245,224,57,67,11,49,134,51,159,144,181,19,96,124,52,
    96,108,131,198,13,194,77,3,41,155,81,139,211,137,140,129,
    196,45,200,255,118,157,66,178,14,168,119,116,113,28,186,141,
    172,254,196,80,236,117,104,227,71,140,10,53,138,114,212,42,
    235,158,250,236,60,168,161,225,231,179,207,142,199,50,80,249,
    30,14,188,74,79,157,192,79,146,84,57,126,24,58,190,82,
    89,116,124,170,100,238,168,212,217,207,59,77,50,246,70,9,
    172,138,222,108,90,2,137,140,142,64,210,47,97,20,40,124,
    217,228,23,214,127,46,21,130,98,148,134,57,142,19,137,161,
    84,46,109,82,221,192,230,227,146,29,163,175,99,151,88,201,
    101,60,80,45,134,157,159,231,30,179,163,113,70,24,173,126,
    237,199,167,82,209,124,132,138,66,174,212,213,140,22,134,177,
    187,36,101,41,36,41,206,75,210,36,156,225,30,163,96,159,
    216,223,101,164,173,2,97,109,11,113,182,132,173,13,109,196,
    221,186,17,144,56,86,129,50,70,216,109,18,30,216,234,162,
    8,24,136,182,75,12,43,29,131,227,2,203,197,206,231,80,
    143,22,187,4,102,119,135,154,93,106,238,151,162,47,66,254,
    246,188,252,143,136,167,193,66,7,102,33,94,229,64,71,215,
    28,232,222,149,3,97,8,236,145,35,24,228,46,87,142,96,
    146,2,178,167,5,234,201,197,208,240,248,185,134,117,86,139,
    187,78,226,218,37,76,93,194,94,29,128,195,26,0,93,178,
    8,163,207,189,247,46,21,238,125,55,42,28,106,21,30,18,
    207,213,2,55,109,198,75,75,4,100,116,163,80,40,43,243,
    57,118,102,119,72,153,117,53,222,193,132,246,34,105,115,102,
    226,236,198,185,95,71,13,173,93,221,177,8,97,3,19,110,
    23,25,39,39,39,159,102,233,151,51,39,29,56,10,202,61,
    60,217,207,15,246,243,143,48,12,56,79,57,176,232,64,160,
    93,61,147,211,12,93,186,201,47,218,77,61,118,89,175,72,
    27,168,110,202,222,108,37,86,50,71,165,92,101,20,140,22,
    166,225,86,165,97,218,240,71,196,176,197,234,53,225,14,62,
    45,193,187,242,82,142,136,92,63,240,87,124,158,145,162,73,
    86,9,84,249,185,61,189,103,22,135,4,115,191,127,13,41,
    11,16,198,125,31,169,63,47,157,204,134,10,23,244,152,180,
    93,194,253,95,128,171,43,1,127,6,194,0,154,186,240,20,
    246,73,122,200,148,155,52,253,15,192,193,230,107,146,154,161,
    61,206,40,194,17,58,100,254,152,167,234,28,247,43,248,107,
    45,82,93,154,32,40,31,153,69,253,84,207,71,86,229,163,
    12,158,111,148,115,172,235,206,76,6,26,249,57,77,211,110,
    107,86,110,123,21,244,170,218,7,99,209,34,112,181,172,89,
    121,180,171,79,175,80,69,193,126,71,108,26,53,172,252,144,
    154,135,21,76,68,57,246,127,222,224,222,124,116,174,101,39,
    79,71,196,95,210,46,44,222,247,154,205,25,84,47,175,220,
    160,81,186,193,163,202,13,36,7,232,55,92,85,83,107,144,
    205,47,13,129,71,29,44,70,232,148,97,129,108,64,223,38,
    135,225,194,81,20,254,36,202,224,69,65,239,90,244,103,157,
    28,105,109,85,102,215,22,165,230,203,133,5,11,50,234,147,
    216,159,28,135,254,83,226,148,19,207,160,244,48,163,20,96,
    189,46,0,121,135,120,151,12,252,250,65,41,200,235,133,5,
    138,15,201,203,74,1,216,45,194,52,224,232,240,219,145,116,
    38,114,114,140,135,168,81,52,117,6,177,63,100,11,153,133,
    128,159,149,2,42,54,241,124,182,205,41,4,29,165,78,144,
    38,24,201,79,3,149,102,78,40,241,120,33,67,231,161,195,
    105,192,137,114,199,63,198,175,126,160,52,218,175,59,45,23,
    114,126,54,204,25,113,39,103,212,93,168,133,61,60,58,70,
    88,166,246,75,5,233,99,77,21,211,185,50,213,174,131,73,
    17,143,13,106,166,3,216,79,168,249,1,53,251,176,232,208,
    223,69,234,47,137,13,41,205,198,24,211,20,101,65,143,51,
    62,167,217,249,219,126,123,252,77,252,86,95,78,20,222,107,
    211,76,185,68,231,86,106,155,20,250,251,173,242,170,99,133,
    7,87,233,94,1,143,231,52,114,131,252,124,233,219,250,57,
    251,200,66,189,227,139,255,169,123,187,143,190,179,253,187,63,
    130,34,241,191,203,181,69,93,184,182,118,237,177,40,171,234,
    186,100,124,86,223,156,67,149,23,100,210,87,82,219,105,123,
    129,114,114,124,208,204,143,43,107,149,194,84,231,138,199,149,
    64,151,92,239,204,110,213,234,93,54,158,120,129,197,9,214,
    196,231,44,179,103,232,178,248,10,143,86,37,58,89,62,145,
    103,94,93,124,93,243,210,86,252,233,84,38,225,85,61,203,
    95,22,102,116,10,62,33,92,21,26,88,188,222,194,231,109,
    183,179,244,196,82,58,54,91,163,114,180,69,26,144,129,26,
    149,166,235,172,65,61,202,186,100,56,29,87,171,144,234,254,
    172,50,197,246,60,10,95,71,153,242,18,169,114,58,42,253,
    183,207,88,238,240,169,164,26,97,213,49,172,67,25,75,37,
    175,153,151,109,94,28,21,67,137,89,43,157,225,57,134,207,
    4,248,30,123,222,98,195,252,79,137,49,177,161,28,68,97,
    222,198,64,191,37,154,86,83,112,6,157,187,240,212,251,162,
    82,79,215,190,179,220,229,152,176,86,233,145,239,230,202,44,
    198,190,66,186,57,242,39,250,202,133,175,18,220,247,168,249,
    94,105,10,6,187,62,16,241,137,67,159,241,208,47,56,189,
    115,54,119,15,104,156,10,239,201,225,65,41,204,129,22,230,
    153,159,71,129,155,98,61,159,241,5,225,228,144,107,209,183,
    39,126,18,167,193,137,12,245,101,212,59,136,241,156,231,233,
    196,199,241,157,175,157,209,139,38,5,133,141,185,239,97,70,
    171,182,230,70,115,153,69,126,28,125,165,239,175,202,97,94,
    252,246,254,233,50,179,246,206,89,184,22,32,25,54,153,28,
    70,57,77,222,184,62,185,8,31,100,81,117,111,30,174,245,
    133,11,69,152,174,113,245,9,253,41,137,151,63,195,134,110,
    168,154,107,77,97,27,116,15,106,138,150,104,11,75,172,182,
    155,102,211,110,54,76,68,33,141,108,138,150,217,108,173,138,
    171,255,61,196,101,203,216,91,105,138,255,0,23,148,155,227,
};

EmbeddedPython embedded_m5_internal_param_Switch(
    "m5/internal/param_Switch.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_Switch.py",
    "m5.internal.param_Switch",
    data_m5_internal_param_Switch,
    2144,
    6462);

} // anonymous namespace