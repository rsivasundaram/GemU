#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_RubyTester[] = {
    120,156,197,25,93,111,219,200,113,150,148,228,72,182,98,59,
    254,202,135,19,51,73,141,83,15,141,117,151,54,151,67,47,
    8,154,158,11,52,69,227,75,233,3,146,83,63,88,154,92,
    73,148,37,82,37,87,201,233,96,191,212,65,219,183,254,136,
    162,15,253,11,125,238,95,234,115,59,51,75,210,180,44,95,
    130,107,161,216,230,98,57,59,156,217,249,158,93,123,144,254,
    148,241,249,137,5,144,252,83,0,248,248,39,160,15,48,16,
    208,18,32,164,0,127,5,14,203,16,255,8,252,50,188,1,
    104,25,32,13,56,193,137,9,191,54,32,92,224,111,42,208,
    55,25,34,96,92,3,89,130,86,25,94,132,203,80,146,21,
    56,172,65,252,123,16,66,132,2,94,250,115,224,95,130,55,
    72,29,39,85,38,120,9,252,26,79,170,224,207,243,164,6,
    227,37,144,243,208,66,226,115,208,170,35,169,15,145,212,101,
    38,245,47,34,229,227,202,42,248,117,66,199,189,124,69,152,
    37,194,100,30,151,153,202,98,182,179,37,104,45,103,243,43,
    133,249,74,97,190,90,152,175,21,230,235,133,249,70,97,126,
    181,48,191,86,152,95,47,204,111,240,124,17,228,50,244,54,
    161,119,19,122,183,160,141,10,93,202,119,189,5,210,132,158,
    5,45,11,36,254,109,193,9,234,220,95,46,124,113,155,191,
    184,146,127,113,135,191,184,11,173,187,32,241,239,142,254,162,
    2,251,141,53,180,99,240,31,252,105,160,29,65,45,224,240,
    74,198,73,16,133,78,16,182,163,192,160,245,10,13,100,117,
    143,134,185,212,252,159,147,249,255,1,108,123,223,72,205,127,
    12,72,88,144,44,125,3,142,121,114,108,192,184,1,71,2,
    122,37,240,77,56,66,54,101,218,64,71,192,137,1,191,49,
    9,225,24,199,18,26,236,22,148,148,182,125,143,13,166,41,
    205,193,113,25,142,202,176,255,242,200,32,192,97,21,226,191,
    195,55,155,76,244,18,19,53,224,8,199,18,156,148,224,184,
    2,47,16,9,65,189,42,137,47,94,30,161,164,8,217,111,
    148,112,183,123,5,113,73,20,63,136,67,119,32,213,50,206,
    157,161,27,187,3,199,30,29,140,191,148,137,146,113,163,150,
    161,69,201,206,208,85,93,155,191,51,73,33,131,161,98,122,
    81,40,213,60,78,218,65,232,59,131,200,31,245,165,186,68,
    196,156,118,208,151,142,195,139,79,7,195,40,86,63,139,227,
    40,182,73,167,12,236,71,110,254,5,105,212,235,71,137,108,
    16,55,102,99,19,121,69,216,237,33,83,164,13,240,94,233,
    99,95,38,94,28,12,21,154,74,83,36,108,162,214,32,35,
    241,144,252,22,135,102,55,26,200,38,138,213,220,141,94,135,
    196,50,105,118,228,224,193,189,68,185,7,125,121,207,117,219,
    31,125,252,80,186,174,255,208,111,30,140,130,190,223,124,98,
    63,107,14,199,170,27,133,205,193,131,102,16,162,34,66,183,
    223,156,84,206,14,226,92,33,54,175,131,142,19,176,128,78,
    87,246,135,50,174,19,244,58,109,65,44,137,5,81,17,166,
    104,136,58,206,202,248,152,98,211,152,23,123,1,137,232,145,
    216,228,94,102,230,80,127,3,54,29,90,254,208,128,120,147,
    220,165,135,127,130,236,139,78,179,79,107,6,175,253,138,116,
    163,161,61,147,156,64,3,143,216,197,208,215,16,243,17,89,
    61,4,246,147,50,244,42,160,253,7,221,78,59,84,60,166,
    17,209,137,140,129,196,75,144,252,245,44,133,112,9,80,247,
    152,38,16,180,142,172,254,200,46,185,223,160,141,239,177,103,
    168,110,144,160,102,89,255,52,231,32,218,71,157,60,31,127,
    113,208,147,158,74,182,16,240,85,52,178,60,55,12,35,101,
    185,190,111,185,74,197,193,193,72,201,196,82,145,181,157,52,
    170,100,240,229,204,185,114,122,227,97,230,76,100,120,116,38,
    253,226,7,158,194,151,21,126,97,253,39,82,161,99,116,35,
    63,65,56,145,232,72,101,211,38,213,101,28,158,100,236,216,
    3,27,149,204,95,18,217,111,171,26,187,158,155,36,14,179,
    35,56,123,25,125,253,202,237,143,164,34,124,116,23,133,92,
    105,170,25,205,212,207,174,146,164,153,160,164,60,39,140,66,
    127,140,251,12,188,109,218,194,85,246,182,5,32,127,91,67,
    95,155,195,177,2,117,244,189,37,195,35,145,74,169,167,177,
    151,173,147,2,128,45,47,210,228,129,30,119,130,41,166,97,
    112,142,96,217,56,8,45,154,209,199,54,57,180,125,131,134,
    77,26,110,102,226,207,74,7,245,73,29,220,39,190,6,11,
    238,153,169,136,121,32,237,157,9,164,107,167,129,132,41,113,
    159,2,194,160,176,57,13,8,147,148,16,63,78,189,159,66,
    13,29,0,151,11,62,207,170,177,151,72,228,74,230,174,54,
    249,96,209,17,59,5,71,180,201,42,236,133,246,181,139,212,
    184,245,254,212,216,209,106,124,64,124,23,82,255,169,179,223,
    212,132,71,198,55,82,165,178,66,119,113,50,222,32,133,22,
    85,185,129,69,238,69,88,231,106,197,21,143,123,9,157,65,
    180,134,245,164,68,158,214,54,97,61,173,66,9,5,252,48,
    142,190,30,91,81,219,82,144,237,225,209,118,178,179,157,124,
    134,41,193,122,204,73,70,39,5,29,246,177,28,198,24,222,
    85,126,209,33,235,112,248,58,105,25,65,149,83,69,103,75,
    177,162,57,67,37,42,166,196,52,83,45,215,114,45,211,166,
    63,35,166,53,86,177,9,27,248,212,4,239,204,137,56,67,
    114,95,193,171,248,252,148,148,77,242,74,160,110,210,222,215,
    251,102,145,72,56,251,131,51,30,51,35,129,236,15,145,195,
    110,22,112,21,200,253,131,30,147,182,76,49,240,103,224,206,
    75,192,159,128,124,1,77,158,70,13,199,39,61,100,210,21,
    66,255,29,112,242,153,82,232,12,29,125,70,154,158,48,56,
    147,135,140,170,235,222,47,224,47,133,204,117,98,130,160,26,
    101,166,189,85,177,70,149,242,120,101,39,122,167,58,84,58,
    27,216,100,164,174,155,16,154,14,97,51,15,225,211,36,152,
    247,68,152,151,102,229,95,151,52,59,135,118,246,244,212,187,
    168,0,220,16,43,70,193,103,126,64,195,189,220,93,68,6,
    155,193,38,183,38,51,118,161,106,57,58,75,254,156,118,82,
    226,189,47,86,56,106,78,73,228,97,81,206,194,226,126,30,
    22,146,19,247,27,238,190,105,52,200,254,39,134,192,227,20,
    54,43,116,146,41,129,44,67,171,66,1,196,205,165,72,227,
    75,100,9,141,18,225,153,170,192,186,217,211,90,203,93,64,
    91,151,134,175,103,154,64,200,192,143,250,238,224,192,119,31,
    83,172,36,196,215,203,34,206,200,132,88,42,10,65,209,34,
    46,146,131,95,63,202,132,121,53,211,228,241,49,114,200,133,
    224,80,241,35,143,51,198,151,93,105,13,228,224,0,15,93,
    221,96,104,181,251,110,135,45,101,166,66,126,145,9,169,216,
    212,147,213,56,161,180,180,23,89,94,20,98,150,31,121,42,
    138,45,95,226,81,68,250,214,61,139,75,132,21,36,150,123,
    128,171,174,167,180,247,159,13,100,110,248,220,184,147,112,111,
    119,248,154,166,51,183,180,131,199,205,0,91,90,39,83,146,
    62,6,229,249,158,187,88,29,74,88,52,241,136,161,198,58,
    177,125,66,195,247,105,216,134,247,81,22,154,200,129,184,36,
    164,188,10,230,158,170,80,75,103,162,248,57,125,149,156,143,
    229,127,191,75,44,235,75,145,52,162,43,132,41,231,232,204,
    75,99,149,74,67,171,150,1,231,121,92,96,96,61,3,94,
    230,113,145,129,75,25,112,153,199,43,12,92,201,128,171,60,
    174,49,112,61,3,110,240,120,149,129,215,50,224,117,30,111,
    48,112,51,3,222,228,241,22,3,183,178,91,31,139,129,183,
    161,117,135,174,54,8,114,151,210,209,220,255,154,142,56,140,
    103,30,192,242,255,154,133,236,251,239,85,6,251,135,144,246,
    44,23,101,32,81,20,176,174,51,80,79,100,135,131,162,116,
    124,253,176,49,197,233,29,47,150,174,146,218,102,215,103,44,
    47,167,51,189,129,78,110,185,76,168,252,152,244,48,23,236,
    132,91,182,241,106,161,117,103,67,138,23,216,95,97,123,127,
    196,178,59,134,238,240,79,253,179,148,171,96,21,135,80,190,
    118,38,213,160,91,120,218,142,59,28,202,208,63,109,207,121,
    101,166,78,64,249,114,8,167,253,18,246,226,171,248,156,15,
    71,18,171,32,37,155,176,156,7,224,172,141,201,206,59,202,
    204,216,248,30,20,139,131,77,70,212,229,32,175,4,246,143,
    115,179,220,156,230,153,225,104,224,120,195,81,66,135,192,183,
    96,96,239,198,53,42,3,168,15,166,161,251,210,245,251,145,
    119,232,168,110,44,147,110,212,247,153,244,187,226,18,19,186,
    76,59,191,164,110,76,35,145,140,241,101,192,44,190,109,157,
    200,242,237,13,191,42,107,106,144,118,37,242,107,247,71,73,
    151,233,189,21,137,136,210,21,100,1,166,182,167,125,244,218,
    61,148,163,161,211,142,229,31,70,50,244,198,76,254,221,48,
    137,7,89,116,114,97,186,66,121,39,137,163,34,199,139,6,
    195,190,84,242,98,229,79,193,205,148,127,126,73,125,50,141,
    4,95,117,162,43,60,197,166,235,57,207,163,48,196,115,44,
    93,155,123,209,40,84,204,253,59,126,74,155,185,77,81,250,
    54,204,111,103,176,235,42,247,59,238,237,226,79,39,247,118,
    33,38,231,25,174,9,190,100,21,79,50,99,227,166,87,70,
    62,194,227,104,236,56,250,78,0,223,251,142,51,251,86,238,
    83,157,241,32,161,147,45,181,114,21,108,230,214,38,126,171,
    165,170,224,222,121,226,95,35,141,60,47,233,147,240,56,177,
    185,204,46,230,105,136,111,240,179,222,149,203,14,221,227,236,
    185,3,125,41,203,23,141,54,41,215,190,155,101,50,14,93,
    125,69,194,247,15,250,230,7,75,12,55,246,220,199,219,59,
    4,167,123,182,193,131,157,76,188,29,45,222,190,14,122,131,
    23,57,77,20,113,36,229,179,103,114,16,197,227,103,145,47,
    217,174,197,245,39,233,153,65,163,56,175,36,29,46,56,57,
    156,231,116,22,247,28,43,141,132,139,250,66,156,207,197,231,
    215,63,167,188,39,253,20,231,230,197,56,187,209,192,69,248,
    116,46,251,65,198,101,121,98,221,143,233,171,181,9,104,34,
    227,192,237,7,223,72,181,57,169,0,223,143,109,55,236,200,
    76,118,178,96,182,170,200,176,147,114,145,121,242,55,62,26,
    76,180,69,236,239,177,236,4,244,194,36,114,244,180,85,248,
    244,194,138,85,252,116,230,193,161,143,227,250,146,241,49,229,
    254,228,151,56,208,101,123,117,177,42,42,6,253,91,199,20,
    53,81,23,37,177,80,175,154,213,74,181,108,98,0,17,100,
    69,212,204,106,109,65,76,251,221,194,112,170,25,91,87,170,
    226,191,209,0,79,102,
};

EmbeddedPython embedded_m5_internal_param_RubyTester(
    "m5/internal/param_RubyTester.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_RubyTester.py",
    "m5.internal.param_RubyTester",
    data_m5_internal_param_RubyTester,
    2438,
    7771);

} // anonymous namespace