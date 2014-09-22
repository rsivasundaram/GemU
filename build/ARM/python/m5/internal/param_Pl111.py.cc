#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_Pl111[] = {
    120,156,197,88,235,115,27,73,17,239,217,149,100,75,182,252,
    118,158,78,172,112,248,162,4,98,93,2,33,20,151,74,145,
    139,41,8,85,241,133,53,69,114,130,98,89,239,142,236,149,
    247,161,218,29,219,209,149,253,5,167,128,15,84,241,71,80,
    124,224,255,224,255,130,238,158,221,245,202,145,185,171,130,211,
    217,210,84,107,30,61,253,248,117,79,207,184,144,253,85,241,
    251,211,22,64,58,103,0,120,248,17,16,0,132,2,186,2,
    132,20,224,45,195,65,21,146,31,130,87,133,247,0,93,3,
    164,1,103,72,152,240,91,3,162,89,94,83,131,192,228,30,
    1,195,6,200,10,116,171,240,38,90,132,138,172,193,65,3,
    146,63,128,16,34,18,240,214,155,2,111,26,222,35,119,36,
    234,204,112,26,188,6,19,117,240,102,152,104,192,112,1,228,
    12,116,145,249,20,116,155,200,234,62,178,154,99,86,255,34,
    86,30,142,172,128,215,164,233,40,203,23,52,179,66,51,121,
    143,57,230,50,159,75,182,0,221,197,156,94,42,209,203,37,
    122,165,68,175,150,232,43,37,250,106,137,190,86,162,175,151,
    232,27,37,250,102,137,94,43,209,183,74,244,237,18,189,94,
    162,91,37,250,78,137,254,14,211,243,32,23,161,255,17,244,
    191,11,253,13,232,161,147,22,10,75,124,12,210,132,254,93,
    232,222,5,137,159,143,225,12,253,232,45,150,86,180,121,197,
    82,177,226,30,175,184,15,221,251,32,241,115,79,175,168,193,
    78,123,21,177,225,255,27,255,218,2,41,53,139,205,145,76,
    82,63,142,108,63,234,197,190,65,227,53,106,8,73,46,53,
    83,25,164,94,16,164,254,9,140,39,207,200,32,117,10,200,
    88,144,46,129,1,167,76,156,26,48,108,195,137,128,126,5,
    60,19,78,112,155,42,9,176,39,224,204,128,223,153,52,225,
    20,219,10,130,224,54,84,148,198,83,159,65,160,57,77,193,
    105,21,78,170,176,243,246,196,160,142,131,58,36,255,128,47,
    215,152,233,52,51,53,224,4,219,10,156,85,224,180,6,111,
    112,18,118,245,235,164,190,120,123,130,154,98,207,78,187,130,
    210,110,151,212,37,85,60,63,137,156,80,178,234,246,192,73,
    156,208,126,29,60,124,248,176,221,200,103,196,233,230,192,81,
    251,22,47,49,201,22,225,64,49,171,56,146,106,6,137,158,
    31,121,118,24,123,135,129,84,211,196,199,238,249,129,180,109,
    30,124,25,14,226,68,253,44,73,226,196,34,115,114,103,16,
    59,197,10,50,166,27,196,169,108,211,110,188,141,69,236,21,
    205,238,13,152,35,9,192,98,210,98,79,166,110,226,15,20,
    122,73,115,164,217,196,173,77,254,225,38,125,139,77,103,63,
    14,101,7,53,234,108,197,199,17,109,153,118,246,100,248,248,
    65,170,156,221,64,62,112,156,222,39,15,159,72,199,241,158,
    120,157,221,67,63,240,58,207,173,87,157,193,80,237,199,81,
    39,124,220,241,35,37,209,58,65,167,100,151,77,28,94,162,
    29,142,253,61,219,103,221,236,125,25,12,100,210,164,222,27,
    180,187,88,16,179,162,38,76,209,22,77,164,170,248,53,197,
    154,49,35,182,125,210,206,37,141,9,84,102,14,163,191,3,
    59,12,253,125,96,64,178,70,32,233,227,71,144,87,17,42,
    59,52,102,240,216,175,200,44,186,183,111,146,235,117,231,9,
    3,11,17,134,51,159,146,175,35,96,116,84,161,95,3,141,
    26,4,155,134,81,50,164,22,167,19,27,3,153,87,32,253,
    219,40,135,104,1,208,236,152,112,176,235,10,110,245,71,6,
    226,78,155,4,223,102,80,168,125,63,69,163,178,233,137,102,
    252,236,160,77,94,15,63,223,237,75,87,165,235,216,241,69,
    124,216,114,157,40,138,85,203,241,188,150,163,84,226,239,30,
    42,153,182,84,220,218,72,219,117,242,245,98,142,171,130,223,
    112,144,227,136,124,142,56,210,63,60,223,85,248,99,153,127,
    176,253,83,169,16,19,251,177,151,98,63,177,216,147,202,34,
    33,213,28,54,207,243,237,24,124,237,90,14,149,84,6,61,
    213,96,212,57,105,106,243,118,212,207,0,163,213,71,78,112,
    40,21,205,71,164,40,220,149,72,189,209,164,32,118,141,148,
    204,117,36,187,217,81,28,121,67,20,209,119,55,104,247,107,
    12,180,89,32,168,173,34,204,166,176,173,65,19,97,183,96,
    184,164,77,37,3,25,3,236,10,233,14,236,116,145,101,11,
    4,219,25,230,148,182,193,73,129,213,226,208,107,17,69,139,
    45,194,178,117,147,154,53,106,110,229,154,79,64,253,230,69,
    245,31,209,150,6,235,236,154,153,118,69,248,108,143,132,207,
    245,243,240,193,244,183,67,97,96,80,176,156,135,129,73,250,
    39,207,50,204,83,128,161,219,113,184,132,116,182,138,181,64,
    218,214,114,144,90,132,188,50,252,246,74,240,179,200,33,140,
    61,235,250,101,22,92,255,86,44,184,167,45,248,152,182,156,
    205,80,211,100,180,52,132,75,46,55,50,123,178,45,183,144,
    24,94,37,91,150,173,120,21,207,178,55,81,147,15,37,62,
    216,184,12,209,41,67,27,87,19,21,194,87,207,132,43,217,
    97,147,82,132,15,146,248,221,176,21,247,90,10,114,25,158,
    110,164,155,27,233,167,152,3,90,207,56,171,232,44,160,227,
    60,145,131,4,227,185,206,63,116,140,218,28,175,118,118,100,
    160,181,233,224,102,39,177,141,57,37,165,42,161,76,52,41,
    3,55,10,3,147,188,159,210,126,13,182,174,9,87,241,219,
    16,44,148,29,115,54,228,202,129,71,241,251,25,217,153,84,
    149,64,53,168,181,163,69,102,109,72,47,235,238,8,78,190,
    121,93,172,251,200,124,43,143,176,26,20,168,160,175,73,210,
    18,232,255,12,92,86,9,248,19,16,2,208,209,89,152,112,
    64,210,151,28,185,76,211,127,15,156,104,198,156,103,134,14,
    55,35,75,69,24,141,233,19,158,170,143,183,95,194,95,74,
    89,234,204,4,65,71,145,153,21,78,229,163,168,82,4,40,
    67,231,107,29,55,149,209,72,38,255,236,59,41,77,211,49,
    107,22,49,123,158,240,138,170,7,19,209,4,80,53,173,119,
    178,73,168,151,231,152,162,60,127,83,44,27,37,164,124,159,
    154,7,5,72,68,222,247,205,202,183,126,49,49,151,206,37,
    91,39,195,95,144,16,21,22,123,190,198,231,41,175,46,66,
    160,154,135,192,163,34,4,36,167,230,247,92,75,83,107,144,
    195,207,12,129,23,46,44,66,232,174,83,1,89,133,110,141,
    130,133,235,69,145,197,146,200,243,22,229,187,145,188,207,22,
    217,214,182,42,124,174,221,73,205,187,73,229,9,242,232,211,
    192,9,119,61,231,153,164,221,104,75,55,143,46,35,151,127,
    161,44,63,69,134,184,76,5,254,249,73,174,199,209,164,114,
    196,67,237,45,45,63,71,132,23,187,156,24,126,189,47,91,
    161,12,119,241,226,180,239,15,90,189,192,217,99,255,152,153,
    126,159,231,250,41,118,240,197,83,54,165,236,179,29,183,220,
    56,194,20,126,232,170,56,105,121,18,239,20,210,107,61,104,
    113,254,111,249,105,203,217,197,81,199,85,26,233,163,241,202,
    229,155,147,236,165,92,169,29,28,19,57,73,255,218,120,91,
    244,177,54,237,229,246,209,87,153,34,153,115,57,170,195,6,
    15,67,188,43,168,161,78,93,63,162,230,30,53,27,48,225,
    156,223,1,126,13,129,148,76,86,195,236,82,23,124,187,226,
    9,175,105,110,250,97,200,254,245,235,132,172,126,29,201,2,
    183,70,51,229,20,93,84,169,173,83,202,239,54,242,206,25,
    110,103,185,179,153,119,206,113,59,207,157,11,249,171,204,34,
    119,46,65,119,153,158,9,168,103,133,146,193,212,255,154,12,
    56,146,38,25,67,193,255,53,7,88,143,190,45,241,173,31,
    64,86,24,92,22,255,162,172,91,83,199,127,95,228,37,119,
    89,49,190,197,47,141,130,207,118,19,233,40,169,157,116,99,
    114,90,114,10,209,123,71,133,171,114,85,138,43,199,147,66,
    157,51,174,134,134,43,165,90,152,61,39,222,96,233,130,245,
    242,9,107,108,27,186,100,62,199,98,165,80,124,30,155,72,
    30,219,37,229,117,57,76,146,56,131,129,140,188,243,82,151,
    71,38,229,112,74,79,9,156,87,33,88,215,174,224,247,195,
    128,171,232,100,146,235,198,62,171,22,33,54,65,239,49,70,
    143,115,191,241,251,222,121,26,182,200,107,58,241,22,57,215,
    250,73,225,135,219,23,0,40,35,18,197,118,157,129,58,76,
    36,221,163,190,106,10,214,68,156,234,71,187,213,205,11,171,
    6,254,59,25,224,205,38,118,15,152,235,127,27,39,150,148,
    150,75,125,124,5,45,207,63,138,92,230,51,174,159,214,19,
    106,145,102,119,114,152,121,50,144,74,150,1,167,200,42,217,
    181,214,147,120,210,198,67,188,116,241,13,6,127,7,182,61,
    209,179,233,199,200,252,128,118,33,97,233,108,170,225,233,180,
    202,255,245,74,93,240,193,127,225,109,86,139,246,61,200,171,
    245,97,106,113,150,154,47,220,203,239,136,249,233,203,241,75,
    55,204,109,39,212,239,67,252,240,97,221,161,230,163,28,33,
    28,129,250,6,199,119,36,125,39,197,88,229,170,132,139,16,
    107,147,250,9,225,225,227,205,92,159,77,173,207,111,34,247,
    101,52,56,84,252,146,25,62,102,63,127,56,107,199,15,245,
    147,153,90,188,48,238,37,14,210,171,23,122,83,153,248,78,
    224,127,41,185,24,255,144,223,243,112,215,217,10,157,45,121,
    228,187,82,93,31,59,231,51,39,149,63,247,221,75,228,126,
    29,56,170,23,39,161,186,53,118,248,101,164,146,23,49,54,
    113,192,239,84,99,84,26,166,74,134,31,232,43,163,195,208,
    126,37,195,56,25,190,138,61,169,238,92,24,127,158,149,120,
    122,138,125,36,169,22,84,173,241,90,142,204,189,196,180,56,
    152,153,118,188,169,94,80,56,73,47,155,51,94,91,158,179,
    21,135,228,138,181,139,2,123,94,98,57,209,158,204,101,29,
    47,198,107,63,206,188,49,126,252,220,91,132,209,124,144,195,
    121,156,75,233,40,29,233,225,90,238,252,252,228,40,78,228,
    158,143,78,72,152,203,200,236,236,132,161,24,99,247,149,51,
    70,121,229,36,35,94,223,146,244,235,206,51,74,119,105,23,
    27,122,219,172,207,215,69,205,160,7,116,83,52,68,83,84,
    196,108,179,110,214,107,245,170,137,89,129,122,150,69,195,172,
    55,102,197,87,255,175,99,238,104,24,235,205,186,248,15,231,
    213,81,21,
};

EmbeddedPython embedded_m5_internal_param_Pl111(
    "m5/internal/param_Pl111.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_Pl111.py",
    "m5.internal.param_Pl111",
    data_m5_internal_param_Pl111,
    2355,
    7197);

} // anonymous namespace