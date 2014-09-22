#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_RawDiskImage[] = {
    120,156,197,88,109,111,227,198,17,158,37,41,218,146,173,179,
    125,190,55,223,185,53,139,192,168,26,244,172,228,90,247,138,
    230,96,52,237,21,200,21,136,147,80,69,125,81,210,48,52,
    185,146,40,83,164,64,174,207,81,96,127,169,15,109,191,245,
    71,20,253,208,255,209,255,213,206,204,146,52,109,223,1,1,
    18,200,182,184,88,46,119,103,119,102,158,103,118,118,3,40,
    254,26,248,252,214,1,200,7,2,32,196,159,128,24,96,34,
    160,47,64,72,1,225,58,28,53,32,251,37,132,13,120,13,
    208,55,64,26,112,142,21,19,190,48,32,89,230,49,54,196,
    38,183,8,152,181,64,90,208,111,192,65,178,6,150,180,225,
    168,5,217,215,32,132,72,4,188,12,23,32,92,132,215,40,
    29,43,77,22,184,8,97,139,43,77,8,151,184,210,130,217,
    42,200,37,232,163,240,5,232,183,81,212,187,40,234,22,139,
    250,47,137,10,241,203,29,8,219,212,29,215,242,57,245,180,
    168,39,207,113,139,165,172,148,43,91,133,254,90,89,191,93,
    171,175,215,234,119,184,190,2,114,13,198,119,97,124,15,198,
    247,1,13,18,174,86,51,60,0,105,194,120,3,250,27,32,
    241,247,0,206,209,62,225,90,109,196,67,30,113,187,26,241,
    136,71,108,66,127,19,36,254,30,233,17,54,244,58,119,209,
    230,209,255,240,175,131,54,7,181,140,197,43,153,229,81,154,
    120,81,50,72,35,131,190,219,84,144,135,2,42,22,10,87,
    253,158,92,245,31,96,63,133,70,225,170,51,64,193,130,116,
    137,13,56,227,202,153,1,179,14,156,10,24,91,16,154,112,
    138,211,52,104,1,67,1,231,6,124,105,82,135,51,44,45,
    52,238,143,193,82,218,79,99,54,174,150,180,0,103,13,56,
    109,64,239,229,169,65,13,71,77,200,254,13,223,110,178,208,
    69,22,106,192,41,150,22,156,91,112,102,195,1,118,194,166,
    113,147,212,23,47,79,81,83,108,233,117,44,92,237,126,77,
    93,82,37,140,178,196,159,72,181,142,117,111,234,103,254,196,
    115,253,147,231,81,126,244,98,226,15,101,167,85,118,76,243,
    157,169,175,70,46,143,52,201,36,147,169,98,137,105,34,213,
    18,86,6,81,18,122,147,52,60,142,165,90,36,113,222,32,
    138,165,231,241,199,23,147,105,154,169,63,100,89,154,185,100,
    85,110,140,83,191,26,65,54,13,226,52,151,29,154,141,167,
    113,73,188,162,222,131,41,75,164,5,240,106,105,112,40,243,
    32,139,166,10,157,165,37,82,111,146,214,33,55,113,145,123,
    88,116,71,233,68,118,81,177,238,243,244,36,161,41,243,238,
    80,78,118,31,231,202,63,140,229,99,223,31,188,247,254,83,
    233,251,225,211,176,123,120,28,197,97,247,67,247,227,238,116,
    166,70,105,210,157,236,118,163,68,73,52,82,220,189,110,158,
    29,236,117,155,38,58,137,134,94,196,42,122,35,25,79,101,
    214,166,214,135,180,8,177,42,150,133,45,76,209,17,109,172,
    53,240,49,197,166,177,36,246,35,82,50,32,197,9,98,102,
    9,170,127,1,187,15,189,127,100,64,182,73,144,25,227,79,
    144,143,17,56,61,250,102,240,183,207,200,58,186,117,108,18,
    16,116,227,41,195,12,241,134,61,159,145,231,19,96,172,52,
    96,108,131,198,16,66,79,131,42,155,81,137,221,73,140,129,
    194,45,200,255,121,89,66,178,10,104,125,164,53,54,221,195,
    169,254,202,176,236,117,104,225,251,140,13,53,138,114,180,45,
    123,128,234,76,164,30,218,228,211,217,39,135,99,25,168,124,
    11,27,62,79,143,157,192,79,146,84,57,126,24,58,190,82,
    89,116,120,172,100,238,168,212,217,206,59,77,114,249,90,9,
    175,74,222,108,90,194,137,92,143,112,210,47,97,20,40,124,
    97,220,122,108,255,92,42,132,198,40,13,115,108,39,17,67,
    169,92,90,164,186,133,197,135,229,116,140,193,142,93,34,38,
    151,241,64,181,24,124,126,158,123,60,29,181,51,206,104,244,
    43,63,62,150,138,250,35,96,20,206,74,85,61,209,156,145,
    246,128,116,45,85,37,243,121,73,154,132,51,92,105,20,108,
    211,34,30,48,222,150,129,16,119,23,209,182,128,165,13,109,
    68,223,170,17,144,82,86,129,53,198,217,61,50,1,176,239,
    69,17,66,16,115,231,24,104,58,6,71,10,214,142,137,232,
    80,141,6,187,4,105,247,17,21,155,84,252,168,52,192,252,
    172,208,190,106,133,39,52,179,193,170,7,102,161,100,69,166,
    253,75,100,218,184,32,19,134,198,30,145,194,32,234,92,144,
    194,36,51,100,123,5,3,136,110,8,2,252,92,195,61,27,
    199,93,37,165,237,18,178,46,225,176,14,198,97,13,140,46,
    249,133,145,232,110,188,205,144,91,55,105,200,161,54,228,46,
    205,188,92,96,168,205,216,105,137,128,0,96,20,102,101,147,
    62,199,202,236,62,153,180,110,204,251,184,221,29,36,109,222,
    183,120,239,227,12,64,199,17,109,99,93,177,8,109,3,19,
    238,21,251,81,78,180,159,102,233,55,51,39,29,56,10,202,
    53,60,219,206,119,182,243,15,48,48,56,123,28,106,116,104,
    208,228,207,228,52,67,146,55,249,69,19,215,99,18,123,197,
    118,130,70,167,189,157,125,197,166,230,56,149,171,140,194,211,
    156,237,220,170,236,76,203,254,128,166,109,177,145,77,184,143,
    79,75,240,218,188,148,35,37,231,24,252,21,159,223,145,185,
    73,99,9,148,5,186,61,189,114,86,138,212,115,127,122,9,
    53,115,83,201,125,23,231,120,94,210,206,134,10,35,244,152,
    180,104,98,194,223,129,243,48,1,127,3,194,3,186,189,224,
    14,179,148,30,114,235,58,117,255,10,56,8,189,97,203,51,
    52,7,141,34,76,33,69,243,167,220,85,239,128,127,132,127,
    212,34,216,185,9,130,118,43,179,200,180,234,187,149,85,177,
    150,129,244,157,118,36,235,50,189,201,77,35,63,167,110,154,
    200,102,69,228,139,96,88,229,71,24,157,230,135,177,69,61,
    161,71,107,123,113,129,48,218,10,30,137,117,163,134,155,159,
    83,241,184,130,140,40,219,230,178,204,173,171,177,187,182,131,
    121,58,94,126,68,107,177,120,245,43,54,39,18,117,33,21,
    61,26,37,61,158,84,244,144,28,196,95,115,70,78,165,65,
    40,56,55,4,30,135,48,121,161,147,136,5,178,1,125,155,
    136,196,233,166,40,120,38,202,208,70,33,241,210,14,193,246,
    217,215,150,171,128,160,125,76,197,55,115,14,37,228,230,103,
    177,63,57,12,253,189,3,154,148,102,14,74,230,25,165,26,
    171,117,53,136,53,226,109,154,240,235,123,165,58,175,230,28,
    70,222,199,57,42,53,152,52,97,26,112,236,248,211,72,58,
    19,57,57,196,195,216,40,154,58,131,216,31,178,183,204,66,
    205,79,74,53,21,187,251,234,238,156,83,128,218,79,157,32,
    77,48,230,31,7,42,205,156,80,226,1,69,134,206,99,135,
    55,12,39,202,29,255,16,191,250,129,210,44,184,76,105,78,
    2,253,108,152,115,190,119,116,66,213,27,240,182,135,7,209,
    8,19,221,151,165,153,244,241,168,138,254,156,219,106,74,225,
    38,138,7,15,53,211,65,238,87,84,252,140,138,109,184,153,
    77,162,139,115,252,153,38,35,3,218,24,135,154,130,15,75,
    245,126,159,210,200,252,58,171,63,254,46,172,214,215,27,5,
    183,237,242,94,100,1,228,34,157,125,251,45,186,80,160,150,
    37,34,252,194,247,37,60,211,228,6,8,242,213,15,202,115,
    247,201,13,107,225,254,2,138,252,224,109,28,23,117,21,219,
    154,227,99,81,166,227,117,253,248,216,191,241,70,72,121,65,
    38,125,37,181,231,30,206,93,103,14,26,122,9,95,87,254,
    43,21,171,14,39,79,43,229,206,57,69,154,221,169,165,203,
    236,78,113,128,249,12,166,212,167,172,191,103,232,172,250,2,
    167,86,101,6,58,205,37,242,196,187,110,10,157,56,211,130,
    252,233,84,38,225,69,82,204,95,230,12,6,138,75,62,92,
    100,40,152,1,223,193,231,58,53,73,181,154,166,236,200,70,
    69,198,249,187,148,97,60,40,157,217,225,204,164,10,195,46,
    185,82,7,222,42,230,186,191,129,50,14,87,64,13,101,44,
    149,124,131,147,20,141,45,206,142,161,196,109,41,157,225,145,
    134,15,6,248,30,123,222,77,68,240,95,227,28,127,129,226,
    64,70,17,220,198,24,222,180,154,130,183,201,43,183,163,122,
    109,148,3,235,244,119,150,187,204,247,149,202,0,124,133,87,
    110,82,140,125,58,192,237,251,19,125,39,195,183,12,238,79,
    168,120,167,52,36,195,86,159,140,248,208,161,143,124,136,115,
    222,195,121,203,118,119,168,157,78,210,147,221,157,82,161,29,
    173,80,165,13,223,34,78,118,223,210,173,23,77,244,61,149,
    90,187,242,61,204,124,172,223,189,210,154,203,44,242,227,232,
    91,125,49,85,54,43,210,244,234,188,180,222,234,141,183,196,
    107,1,139,157,158,201,97,148,163,28,22,114,149,192,228,7,
    206,157,223,16,231,234,131,111,0,35,58,37,213,135,238,61,
    186,150,205,247,176,160,11,168,230,74,83,216,6,93,118,154,
    162,37,218,194,18,203,237,166,217,180,155,13,19,113,68,45,
    235,162,101,54,91,203,66,255,111,33,174,90,198,86,171,41,
    254,15,141,217,152,141,
};

EmbeddedPython embedded_m5_internal_param_RawDiskImage(
    "m5/internal/param_RawDiskImage.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_RawDiskImage.py",
    "m5.internal.param_RawDiskImage",
    data_m5_internal_param_RawDiskImage,
    2086,
    6419);

} // anonymous namespace