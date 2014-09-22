#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_SimpleDRAM[] = {
    120,156,221,88,107,115,219,198,21,5,68,138,146,40,201,140,
    105,155,118,252,10,108,199,14,157,196,124,233,97,101,154,166,
    145,40,123,38,51,118,162,1,149,113,162,47,24,8,88,146,
    176,65,128,193,46,45,169,159,58,227,252,144,252,156,126,110,
    126,80,122,207,5,22,36,213,105,38,105,227,116,26,211,188,
    220,123,119,177,216,187,231,158,221,99,123,70,246,167,64,223,
    207,45,195,144,127,167,134,79,127,77,35,52,140,195,172,101,
    82,107,193,16,11,70,223,52,252,130,241,189,97,188,49,140,
    111,143,10,134,95,212,209,197,60,90,52,252,146,142,46,229,
    209,69,195,95,54,132,201,209,149,60,90,50,252,178,33,74,
    28,93,205,163,75,134,191,166,163,235,121,116,217,240,47,232,
    104,37,143,174,24,254,59,58,122,49,143,150,13,191,106,244,
    234,151,40,153,224,39,250,83,55,169,165,96,62,84,203,100,
    159,139,81,207,27,10,223,211,249,47,208,119,15,249,95,166,
    134,48,140,35,51,219,133,193,130,113,180,96,216,189,58,70,
    168,34,153,190,215,151,170,132,70,130,102,29,91,199,147,58,
    78,228,142,132,227,168,50,59,163,216,159,132,112,241,208,107,
    55,148,117,188,104,106,228,30,153,230,48,30,137,102,226,142,
    154,251,241,73,20,198,174,47,155,3,49,218,122,36,149,123,
    28,138,71,174,219,111,181,31,11,215,245,31,251,77,153,120,
    205,145,24,53,123,193,104,28,138,125,123,247,121,99,124,102,
    35,171,38,230,195,10,75,134,90,162,159,93,223,79,158,187,
    227,60,189,130,78,239,234,185,244,8,214,65,33,203,48,79,
    196,118,247,220,238,176,27,79,157,184,59,100,167,27,167,125,
    60,214,70,98,246,34,76,105,46,179,223,52,61,164,179,61,
    147,30,54,247,192,29,136,231,110,68,246,63,200,16,171,142,
    199,34,82,235,89,195,113,125,119,172,130,215,66,33,23,47,
    140,165,248,29,243,91,165,105,62,57,151,223,116,196,191,230,
    247,163,57,147,31,165,69,41,18,17,69,17,137,190,161,246,
    34,216,240,114,9,60,37,118,190,161,242,93,158,137,44,114,
    100,133,35,101,16,149,40,138,200,170,142,44,131,158,136,48,
    3,95,174,131,160,68,77,68,152,125,47,43,32,39,209,18,
    17,102,222,203,139,32,38,81,18,145,42,71,46,129,148,68,
    71,68,46,207,68,170,28,185,162,215,115,137,23,92,211,238,
    101,118,175,178,123,205,240,175,176,251,174,238,173,177,123,93,
    187,87,217,189,161,221,107,236,222,212,238,187,134,127,157,223,
    117,75,191,253,6,15,184,173,221,155,236,190,167,221,91,236,
    90,218,189,205,238,29,237,190,199,238,93,237,90,236,222,211,
    238,29,118,223,215,238,93,118,239,107,247,30,187,15,180,251,
    62,187,31,232,117,222,103,183,142,218,124,128,58,171,160,16,
    170,100,80,39,146,171,192,241,169,188,26,195,161,228,194,8,
    221,215,194,26,199,137,10,44,12,189,69,230,203,201,232,88,
    36,86,220,183,78,146,64,9,235,187,137,152,8,139,234,59,
    9,132,148,55,231,70,36,194,245,231,7,4,79,49,207,13,
    50,135,195,68,200,97,28,250,150,138,45,234,27,12,232,25,
    158,82,6,92,240,215,206,15,146,42,30,103,35,236,130,30,
    65,71,107,156,156,89,18,199,235,36,12,162,1,45,55,12,
    188,51,27,231,135,172,101,135,19,173,204,26,185,227,241,76,
    255,26,250,111,103,236,182,64,195,73,34,104,16,104,62,162,
    213,102,227,152,191,237,86,36,153,233,61,229,170,192,179,250,
    73,28,41,17,249,86,232,210,143,119,198,239,201,250,142,93,
    239,213,108,87,11,55,156,171,92,235,120,34,173,147,192,87,
    67,43,136,172,227,64,73,171,31,39,150,112,189,161,5,234,
    89,190,120,29,120,162,233,13,131,177,188,14,250,77,18,169,
    172,80,68,131,161,178,234,123,207,30,242,115,194,85,82,62,
    208,203,174,39,241,9,77,220,239,139,228,161,37,131,191,18,
    88,180,139,179,51,89,115,128,164,61,50,235,194,216,196,141,
    94,49,30,51,168,33,196,157,222,208,141,34,17,242,70,79,
    251,143,243,126,140,12,76,93,68,211,17,250,57,249,14,170,
    108,183,7,244,186,244,227,139,208,61,147,56,130,224,233,29,
    194,179,54,229,49,78,4,61,152,80,90,42,24,9,126,118,
    183,123,136,103,15,236,39,217,179,173,124,99,252,73,66,251,
    29,71,86,29,219,184,191,111,211,62,232,13,163,61,110,90,
    29,203,59,243,66,33,31,166,47,16,125,148,82,26,75,95,
    112,109,54,28,143,8,122,159,182,88,137,4,183,39,10,249,
    5,23,56,189,159,235,88,158,4,138,246,141,42,232,252,242,
    240,147,46,15,10,224,27,203,245,232,120,79,23,119,18,68,
    126,124,34,239,160,82,221,83,43,202,183,40,27,35,36,80,
    77,71,213,31,206,221,2,92,121,234,108,44,248,140,246,78,
    79,157,33,45,67,36,106,69,51,243,128,136,201,163,192,80,
    190,77,14,92,98,47,95,157,95,71,50,24,68,194,87,23,
    201,97,210,56,105,157,56,40,19,133,213,35,171,185,32,238,
    190,3,145,120,84,253,170,150,63,54,12,6,67,71,49,17,
    29,2,221,83,87,242,174,48,62,153,237,97,109,192,83,211,
    121,226,48,37,157,140,109,152,90,129,114,46,177,209,201,168,
    200,151,145,130,25,83,49,103,67,121,21,207,210,210,80,224,
    156,100,94,57,154,115,78,86,54,188,194,172,47,227,92,222,
    133,37,164,165,78,249,73,135,89,199,111,231,18,113,210,18,
    225,109,77,143,143,30,178,191,50,125,136,88,117,126,179,50,
    230,32,79,7,101,207,251,202,76,225,80,86,241,234,2,94,
    146,71,121,32,208,208,132,72,49,181,187,251,138,181,65,247,
    89,250,107,31,100,29,187,61,150,122,106,239,107,187,119,152,
    197,158,118,25,90,101,63,121,250,69,26,122,113,104,103,125,
    246,126,218,248,102,247,5,175,114,90,121,78,24,140,2,245,
    150,68,4,238,141,191,96,190,79,33,34,76,250,20,215,10,
    85,179,90,172,150,170,248,53,233,215,172,22,42,11,21,147,
    191,102,133,122,42,5,254,44,211,183,200,173,98,197,100,81,
    68,220,221,112,218,219,173,150,115,186,189,153,43,16,83,43,
    144,200,56,167,64,12,40,16,232,144,34,183,23,33,69,32,
    237,169,205,170,3,210,189,0,213,1,91,134,44,33,189,65,
    82,132,52,6,41,144,163,117,136,16,82,23,164,61,142,42,
    144,31,164,43,72,117,28,93,132,240,32,69,65,151,35,74,
    37,0,110,12,79,251,213,94,0,181,36,81,153,237,141,198,
    227,173,40,5,114,3,13,140,192,47,48,218,104,225,162,64,
    235,113,99,103,162,91,212,43,129,234,118,163,163,159,220,164,
    113,1,26,252,111,134,25,229,119,23,230,30,204,125,24,190,
    165,63,128,1,126,246,135,48,31,193,124,12,243,8,166,1,
    3,65,110,227,108,180,219,48,29,152,13,152,77,227,173,41,
    201,45,154,230,7,204,135,53,151,22,74,133,82,145,190,233,
    199,76,139,34,247,41,162,112,10,63,59,32,176,59,78,111,
    211,105,183,182,183,157,211,141,206,47,192,155,21,39,48,46,
    50,162,139,108,75,28,95,98,116,151,217,174,176,45,51,210,
    171,140,244,26,35,189,206,72,95,224,200,207,225,109,105,208,
    109,232,127,190,214,210,203,63,7,173,19,205,64,202,128,183,
    55,52,224,27,141,79,38,41,204,237,86,163,149,61,177,245,
    7,128,249,79,52,205,63,254,13,204,248,44,148,74,12,184,
    134,25,103,195,139,192,23,95,124,229,116,64,233,118,103,231,
    119,196,120,133,49,46,51,198,171,140,241,90,134,49,78,154,
    224,111,48,69,77,236,77,34,246,20,230,157,72,218,127,214,
    94,167,149,33,220,105,207,33,108,227,196,155,129,217,198,49,
    200,167,195,255,49,196,159,211,52,75,230,47,103,114,69,51,
    89,31,219,191,49,141,23,103,142,237,95,13,241,148,198,187,
    154,198,12,26,99,107,239,192,124,134,189,155,146,118,254,148,
    158,197,245,191,161,238,163,28,220,255,25,174,79,104,154,157,
    57,92,231,137,91,152,167,238,151,117,214,99,144,151,163,173,
    198,24,82,82,178,160,217,61,150,42,33,105,145,138,37,230,
    199,19,82,178,44,247,88,217,177,144,99,85,192,183,2,159,
    25,92,85,188,132,183,147,29,235,170,79,211,255,245,250,12,
    219,206,50,191,108,150,139,181,165,90,161,86,170,185,181,78,
    109,171,214,254,39,186,157,56,180,
};

EmbeddedPython embedded_m5_objects_SimpleDRAM(
    "m5/objects/SimpleDRAM.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/mem/SimpleDRAM.py",
    "m5.objects.SimpleDRAM",
    data_m5_objects_SimpleDRAM,
    1913,
    5236);

} // anonymous namespace