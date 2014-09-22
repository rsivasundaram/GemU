#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_O3CPU[] = {
    120,156,181,152,89,115,27,185,17,128,65,74,34,117,217,162,
    101,203,247,49,190,233,147,62,214,199,122,189,94,75,148,100,
    115,45,75,242,144,46,87,244,194,26,14,64,17,210,112,134,
    30,204,232,72,109,158,156,183,60,100,171,242,15,242,75,242,
    152,199,252,163,164,187,49,32,65,217,235,173,84,121,151,22,
    119,240,161,209,232,110,0,141,30,250,44,251,111,4,254,94,
    58,140,169,61,120,224,240,47,199,2,198,186,57,182,145,99,
    57,108,231,89,144,103,141,236,105,68,63,141,176,96,148,117,
    71,217,198,104,95,102,76,247,140,178,160,192,186,5,182,81,
    208,61,99,44,40,178,110,145,109,20,161,93,96,98,148,181,
    115,140,23,217,95,25,251,196,216,159,54,198,25,31,103,245,
    242,4,76,46,255,11,255,149,115,240,148,140,195,87,43,149,
    1,95,10,119,52,193,175,155,250,177,8,95,11,158,18,213,
    245,247,26,224,224,181,135,213,142,240,183,69,172,209,12,202,
    196,94,232,119,214,99,193,165,159,68,113,50,9,108,81,196,
    114,7,100,215,223,251,38,0,163,40,138,1,248,27,60,9,
    134,126,131,225,27,121,140,196,198,8,90,12,238,161,185,208,
    28,163,230,136,105,146,63,224,114,214,44,50,49,206,182,38,
    208,105,112,245,83,158,109,76,26,82,68,55,145,76,17,153,
    102,28,224,36,145,67,22,153,34,114,216,34,211,68,102,44,
    114,136,72,201,104,62,204,248,12,145,35,134,148,24,63,66,
    100,214,26,53,75,228,168,69,142,18,57,102,145,99,68,230,
    44,50,71,228,184,53,215,113,34,39,44,153,19,68,78,90,
    228,36,145,83,22,57,69,228,180,165,231,52,145,51,150,204,
    25,34,103,13,57,203,248,57,34,231,44,153,243,68,206,91,
    122,46,16,185,96,17,135,136,99,145,139,68,46,26,2,255,
    46,17,185,68,228,50,19,87,112,47,242,203,4,175,90,211,
    93,33,114,205,34,87,137,92,183,148,95,35,82,182,200,117,
    34,55,204,168,50,227,55,136,220,180,244,220,36,114,203,140,
    186,197,248,109,34,183,45,114,135,200,29,67,238,50,94,33,
    114,215,34,247,136,84,12,185,207,248,3,34,247,136,220,103,
    226,1,227,15,137,60,52,50,223,49,254,136,200,119,134,60,
    102,252,9,145,71,22,121,74,228,177,21,180,239,137,60,49,
    228,25,227,63,16,121,106,145,231,68,190,199,35,176,241,140,
    137,31,24,255,145,157,226,47,216,118,158,197,225,136,120,206,
    182,158,162,111,199,160,51,100,217,56,1,227,126,162,113,63,
    90,91,248,37,145,23,134,204,51,190,64,228,39,203,162,42,
    145,151,68,64,96,145,241,37,34,11,134,44,51,254,138,72,
    149,8,8,188,102,188,70,100,201,146,249,153,200,178,37,243,
    134,200,43,75,102,133,200,107,75,230,45,145,154,145,89,101,
    124,141,200,207,68,222,48,241,134,241,117,38,86,216,22,72,
    66,246,123,71,189,171,102,101,236,216,112,151,250,214,24,175,
    99,50,217,88,103,110,189,220,128,196,228,142,193,151,194,108,
    230,247,210,74,244,176,194,49,129,221,237,116,124,76,115,230,
    175,138,233,11,243,24,164,172,122,57,15,15,171,73,1,211,
    160,236,202,112,179,140,89,78,167,69,76,246,126,160,52,193,
    47,245,18,190,42,157,168,43,42,177,215,173,44,70,187,97,
    16,121,92,85,54,69,247,209,29,149,120,173,64,220,241,188,
    246,189,251,79,132,231,241,39,188,162,98,191,146,217,66,121,
    244,110,111,63,153,2,37,93,209,141,226,253,102,55,226,226,
    6,42,70,43,88,254,203,102,38,12,204,68,178,170,205,66,
    214,136,83,65,45,183,104,140,251,134,22,30,6,37,177,248,
    152,202,88,52,125,207,239,8,117,231,255,52,210,157,48,81,
    252,67,12,60,130,154,210,94,15,238,169,102,226,109,139,102,
    180,35,226,251,3,27,37,77,117,8,190,106,161,76,164,23,
    56,126,148,134,137,252,55,98,140,127,21,189,114,214,97,188,
    146,104,165,154,163,235,206,135,245,112,146,200,105,139,196,239,
    56,92,4,222,62,245,184,34,244,186,159,245,92,67,253,74,
    165,162,178,180,39,252,52,17,149,15,177,76,68,203,243,183,
    191,164,164,26,117,187,50,57,208,35,199,141,73,203,4,119,
    37,79,58,18,131,164,78,247,97,43,109,183,69,236,40,249,
    103,225,200,208,105,237,39,66,169,227,67,118,113,109,187,158,
    237,250,215,13,27,146,61,62,100,217,80,215,92,223,128,131,
    61,211,131,104,145,193,191,55,99,172,237,252,210,140,159,117,
    13,86,97,168,107,122,224,238,96,202,129,154,223,154,92,7,
    25,119,5,13,24,4,236,171,3,212,3,227,15,138,10,45,
    164,187,156,178,12,19,17,135,176,167,160,43,129,93,84,91,
    250,224,192,198,221,20,55,20,30,156,69,169,122,94,127,41,
    105,105,181,38,221,198,236,52,152,237,75,140,139,30,176,99,
    24,252,52,244,19,25,225,92,239,97,31,59,189,40,10,126,
    47,212,190,14,137,118,227,252,144,203,177,136,98,200,136,102,
    59,13,226,154,133,81,27,131,160,254,49,245,148,217,140,120,
    140,136,54,98,175,231,4,94,34,66,127,95,205,14,54,135,
    133,37,101,224,219,40,44,97,82,123,227,182,35,152,25,140,
    220,245,98,174,200,204,52,148,190,135,254,169,155,191,53,0,
    254,80,126,88,92,58,56,201,89,76,54,105,183,5,226,81,
    219,193,36,226,124,76,5,196,89,132,73,44,225,128,156,27,
    18,80,80,212,138,97,9,137,137,75,221,26,18,235,5,158,
    47,20,70,75,117,100,59,113,60,206,193,108,209,198,193,62,
    214,204,170,134,241,233,68,105,192,113,165,68,200,209,111,103,
    71,70,129,246,5,164,181,164,224,228,2,165,55,231,154,54,
    64,57,64,182,82,149,100,77,249,235,127,70,24,169,28,118,
    165,162,205,149,161,74,148,153,30,247,26,204,232,244,76,141,
    14,38,146,25,45,20,220,241,2,201,97,17,184,196,26,157,
    156,95,241,96,30,202,53,96,138,86,72,249,149,194,171,78,
    160,31,4,149,72,156,218,162,221,119,106,200,32,55,219,55,
    11,180,56,74,98,230,87,87,134,195,214,217,87,176,58,129,
    131,135,99,19,88,44,54,165,130,115,162,14,198,215,8,182,
    193,205,4,238,91,216,212,48,102,32,79,47,30,141,121,247,
    213,82,163,89,171,207,211,37,188,247,244,177,186,240,101,53,
    190,111,77,117,121,72,6,131,23,167,116,128,14,172,251,191,
    112,221,157,33,225,3,103,195,236,33,60,64,245,183,13,103,
    32,72,123,30,237,110,116,98,1,43,155,208,121,1,16,136,
    140,148,178,49,250,116,244,162,64,250,250,210,95,247,226,68,
    162,57,130,83,166,67,161,149,250,59,167,222,241,98,212,184,
    78,162,146,163,117,151,173,126,212,11,75,13,43,221,151,244,
    224,242,20,232,243,92,38,87,59,168,70,93,26,244,124,77,
    129,49,196,93,91,56,168,225,178,213,245,21,21,180,98,46,
    220,175,220,141,90,16,244,35,217,176,44,171,104,101,36,20,
    166,221,44,70,20,36,253,202,233,244,223,57,105,75,46,133,
    180,13,27,245,53,231,45,21,73,14,22,73,1,85,29,152,
    198,39,76,213,241,79,93,117,64,17,119,10,222,251,182,11,
    44,254,59,190,108,226,59,117,14,223,200,255,2,111,228,73,
    30,107,235,95,24,219,26,193,151,76,40,157,224,221,50,25,
    195,215,75,248,254,196,70,161,171,89,96,191,228,176,64,70,
    177,34,219,26,199,242,19,159,11,172,89,180,186,38,134,187,
    52,156,100,89,115,146,133,51,248,142,250,234,53,44,53,190,
    153,234,223,3,234,101,204,158,171,46,110,18,218,202,94,220,
    181,222,220,177,236,156,143,187,141,149,5,122,137,223,141,226,
    109,60,253,180,91,196,158,76,214,194,165,56,134,34,5,149,
    164,61,60,222,134,28,69,113,47,14,215,194,96,127,45,92,
    129,65,154,83,74,195,35,140,57,120,201,117,215,220,103,78,
    246,174,239,68,32,235,100,101,19,100,4,88,48,128,243,238,
    91,7,206,217,69,42,131,200,42,149,217,89,166,43,23,43,
    41,23,103,115,17,187,120,45,37,152,228,151,189,64,9,42,
    243,232,39,6,157,241,98,242,81,38,45,247,164,113,152,39,
    45,114,19,74,184,38,92,39,163,153,103,84,119,107,107,69,
    208,166,9,190,117,145,136,65,131,244,157,121,95,237,165,255,
    64,213,184,223,89,174,148,43,229,103,114,5,248,76,231,206,
    192,103,118,100,44,87,126,199,178,31,83,154,77,188,50,155,
    77,218,180,77,172,210,211,0,155,84,232,238,247,4,113,127,
    111,175,217,129,189,12,78,227,114,249,129,167,20,156,135,78,
    196,93,212,225,210,169,152,50,225,162,211,66,202,223,135,74,
    110,66,10,160,134,7,233,105,71,38,250,120,80,169,77,53,
    41,69,172,186,239,7,66,37,120,209,234,226,171,17,81,74,
    89,196,123,155,176,46,144,134,48,150,18,82,236,126,38,170,
    203,130,33,140,83,210,237,240,1,175,121,26,73,77,157,230,
    235,176,137,104,151,153,73,116,93,166,135,150,204,44,54,60,
    106,77,99,243,89,163,120,24,79,245,253,210,243,247,117,234,
    130,229,115,157,7,185,9,138,205,167,250,6,15,148,154,241,
    80,167,13,172,55,94,245,33,106,148,88,86,53,162,172,174,
    210,28,119,17,207,74,58,173,115,210,72,234,38,238,254,221,
    150,253,188,136,5,28,45,225,242,251,117,40,217,72,199,162,
    104,123,105,144,100,4,59,219,41,61,246,253,214,249,242,115,
    19,33,253,14,156,211,206,232,217,176,173,168,82,27,180,177,
    24,91,209,181,24,41,209,129,183,32,10,97,25,6,179,209,
    18,99,201,154,85,89,6,225,161,94,121,183,164,175,64,106,
    213,251,45,212,9,119,18,184,72,231,170,142,69,18,157,139,
    5,240,69,167,131,67,90,132,250,49,49,169,4,207,28,213,
    31,77,40,53,154,176,171,189,184,217,131,87,243,72,31,130,
    149,229,122,131,38,198,70,189,94,211,141,162,190,48,224,78,
    81,100,36,60,175,195,173,95,11,19,87,108,106,67,50,180,
    140,165,4,65,188,13,155,92,71,122,85,119,86,171,216,67,
    105,140,236,172,67,186,33,19,67,187,159,110,113,32,181,190,
    163,153,8,4,223,16,84,174,186,168,215,212,0,166,4,192,
    213,172,39,120,43,146,165,32,68,18,217,205,55,173,17,132,
    36,3,152,22,193,13,58,113,186,163,127,187,234,53,237,38,
    53,35,123,216,180,7,34,153,62,176,44,147,153,233,131,129,
    80,198,244,158,210,114,238,168,185,114,32,25,65,129,227,226,
    69,77,187,185,213,255,237,151,150,32,20,80,160,192,5,236,
    58,67,89,249,219,165,102,250,157,230,42,106,251,21,190,48,
    11,23,242,51,163,248,153,205,195,39,7,159,193,183,121,26,
    252,223,60,195,103,206,122,182,164,115,19,217,167,63,114,164,
    144,43,21,75,3,150,59,240,201,79,83,223,209,220,68,126,
    149,126,95,167,192,116,31,221,133,221,36,67,145,109,160,9,
    205,122,152,204,21,197,10,91,113,180,183,239,226,125,230,206,
    247,239,73,140,53,185,249,71,196,143,102,126,174,175,165,23,
    88,233,211,139,103,41,55,9,159,18,253,149,242,255,3,232,
    35,57,114,
};

EmbeddedPython embedded_m5_objects_O3CPU(
    "m5/objects/O3CPU.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/cpu/o3/O3CPU.py",
    "m5.objects.O3CPU",
    data_m5_objects_O3CPU,
    2531,
    6323);

} // anonymous namespace
