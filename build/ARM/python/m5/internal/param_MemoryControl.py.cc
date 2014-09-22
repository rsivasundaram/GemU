#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_MemoryControl[] = {
    120,156,197,88,109,111,220,198,17,158,37,121,148,238,164,179,
    36,75,178,108,75,182,24,4,66,47,65,173,75,220,170,46,
    16,195,104,98,23,104,10,68,73,120,5,108,95,139,208,212,
    113,79,199,19,95,14,228,202,206,5,82,63,84,70,219,111,
    253,17,69,63,244,127,244,127,181,51,179,36,117,39,75,168,
    129,4,39,233,184,88,46,103,103,119,102,158,121,217,237,65,
    241,87,195,231,55,14,64,254,103,1,16,224,79,64,4,16,
    11,232,10,16,82,64,176,10,71,53,200,126,9,65,13,222,
    2,116,13,144,6,156,97,199,132,63,26,144,44,242,28,27,
    34,147,71,4,140,27,32,45,232,214,224,121,178,2,150,180,
    225,168,1,217,43,16,66,36,2,94,4,115,16,204,195,91,
    228,142,157,58,51,156,135,160,193,157,58,4,11,220,105,192,
    120,25,228,2,116,145,249,28,116,155,200,234,99,100,117,131,
    89,253,135,88,5,248,101,13,130,38,145,227,94,94,18,165,
    69,148,188,198,13,230,178,84,238,108,25,186,43,101,255,230,
    68,127,117,162,191,54,209,95,159,232,223,226,254,18,200,21,
    24,110,192,240,54,12,239,64,31,149,178,92,173,124,23,164,
    9,195,77,232,110,130,196,223,93,56,67,189,5,43,19,51,
    182,120,198,205,106,198,61,158,113,31,186,247,65,226,239,158,
    158,97,67,167,181,142,182,8,255,139,127,45,180,5,168,69,
    108,94,203,44,15,211,196,11,147,126,26,26,244,221,166,134,
    44,215,163,102,174,48,225,83,50,225,191,129,237,23,24,133,
    9,79,1,25,11,146,37,50,224,148,59,167,6,140,91,112,
    34,96,104,65,96,194,9,46,83,163,13,28,10,56,51,224,
    79,38,17,156,98,107,161,210,239,131,165,180,253,134,172,116,
    205,105,14,78,107,112,82,131,206,139,19,131,6,142,234,144,
    253,11,126,216,98,166,243,204,212,128,19,108,45,56,179,224,
    212,134,231,72,132,67,195,58,137,47,94,156,160,164,56,210,
    105,89,184,219,253,9,113,73,148,32,204,18,63,150,106,13,
    251,222,200,207,252,216,251,74,198,105,54,126,154,38,42,75,
    163,86,163,164,76,243,221,145,175,6,46,79,53,73,39,241,
    72,49,203,52,145,106,1,59,253,48,9,188,56,13,142,35,
    169,230,137,159,215,15,35,233,121,252,241,203,120,148,102,234,
    183,89,150,102,46,169,149,7,163,212,175,102,144,82,123,81,
    154,203,22,173,198,203,184,196,94,17,117,127,196,28,105,3,
    188,93,154,28,200,188,151,133,35,133,214,210,28,137,154,184,
    181,200,78,220,228,175,176,105,15,210,88,182,81,178,246,179,
    244,77,66,75,230,237,67,25,239,61,200,149,127,16,201,7,
    190,223,255,228,211,71,210,247,131,71,65,251,224,56,140,130,
    246,231,238,87,237,209,88,13,210,164,29,239,181,195,68,73,
    212,82,212,190,68,63,187,72,118,147,86,122,19,30,122,33,
    203,232,13,100,52,146,89,147,70,239,210,46,196,178,88,20,
    182,48,69,75,52,177,87,195,199,20,91,198,130,216,15,73,
    202,30,73,78,32,51,75,88,253,19,216,128,104,255,35,3,
    178,45,2,205,16,127,130,172,140,208,233,208,55,131,191,125,
    75,234,209,163,67,147,160,160,7,79,24,104,136,56,164,124,
    76,182,79,128,209,82,131,161,13,26,69,8,62,13,171,108,
    76,45,146,19,27,3,153,91,144,255,99,154,67,178,12,168,
    126,116,120,28,186,133,75,253,133,129,217,105,209,198,247,25,
    28,106,16,230,168,92,54,1,245,217,149,58,168,147,111,198,
    95,31,12,101,79,229,219,56,240,50,61,118,122,126,146,164,
    202,241,131,192,241,149,202,194,131,99,37,115,71,165,206,78,
    222,170,147,205,87,74,124,85,252,198,163,18,79,100,123,196,
    147,126,9,194,158,194,151,85,126,97,253,231,82,33,54,6,
    105,144,227,56,177,56,148,202,165,77,170,27,216,124,94,46,
    199,32,108,217,37,100,114,25,245,85,131,209,231,231,185,199,
    203,209,56,3,141,102,191,246,163,99,169,136,30,17,163,112,
    85,234,234,133,102,13,181,219,36,108,41,43,233,207,75,210,
    36,24,227,86,195,222,14,237,226,54,3,110,17,8,114,235,
    8,183,57,108,109,104,34,252,150,141,30,73,101,21,96,99,
    160,221,34,29,0,27,95,20,81,4,65,119,134,177,166,101,
    112,176,96,241,216,21,29,234,209,100,151,48,237,110,82,179,
    69,205,189,82,3,51,84,67,243,162,26,30,210,210,6,203,
    222,51,11,41,43,119,218,159,114,167,59,231,238,132,225,177,
    67,110,97,144,243,156,187,133,73,122,200,158,20,62,64,14,
    135,48,192,207,19,200,103,237,184,203,36,181,93,130,214,37,
    36,78,194,241,112,2,142,46,25,134,177,232,222,185,74,147,
    219,215,170,201,67,173,201,61,90,122,177,64,81,147,209,211,
    16,61,130,128,81,232,149,117,250,12,59,227,13,210,233,164,
    54,55,48,231,61,79,154,156,188,56,1,114,121,160,67,137,
    86,178,238,88,132,183,190,9,183,138,164,148,147,231,143,178,
    244,251,177,147,246,29,5,229,30,30,239,228,187,59,249,103,
    24,27,156,39,28,109,116,116,208,254,159,201,81,134,126,94,
    231,23,237,187,30,251,177,87,164,20,212,58,37,120,54,22,
    235,154,67,85,174,50,138,80,179,86,116,163,82,52,237,251,
    51,90,183,193,90,54,97,3,159,134,224,205,121,41,71,75,
    174,52,248,43,62,95,144,190,73,100,9,84,35,186,29,189,
    117,150,138,228,115,127,54,133,155,217,201,228,126,140,139,60,
    43,61,207,134,10,37,244,152,180,107,114,134,191,1,151,99,
    2,254,10,132,8,52,124,225,62,236,168,244,144,97,87,137,
    252,59,224,64,116,73,222,51,180,27,26,69,168,66,47,205,
    31,49,169,78,131,191,135,191,79,68,177,51,19,4,165,44,
    179,40,184,38,83,150,85,57,46,67,233,189,210,146,53,237,
    225,100,167,129,159,19,153,246,101,179,242,229,243,128,88,85,
    73,24,160,102,136,178,121,189,162,71,155,251,242,28,99,148,
    15,54,197,170,49,129,156,159,83,243,160,2,141,40,199,102,
    179,207,109,184,58,143,121,58,104,254,142,54,99,241,246,151,
    108,142,83,83,92,42,23,169,149,46,242,176,114,17,201,161,
    252,45,215,230,212,26,4,132,51,67,224,129,9,139,24,58,
    171,88,32,107,208,181,201,153,184,238,20,133,175,137,50,190,
    209,122,83,121,130,53,180,175,117,87,97,65,155,153,154,239,
    103,29,79,200,210,143,35,63,62,8,252,39,47,105,85,90,
    186,87,122,159,81,202,177,60,41,7,121,142,184,74,20,126,
    253,164,148,231,245,172,99,201,167,228,136,165,28,236,57,65,
    218,227,0,242,135,129,116,98,25,31,224,193,108,16,142,156,
    126,228,31,178,189,204,66,206,175,75,57,21,27,252,98,150,
    206,41,74,237,167,78,47,77,48,244,31,247,84,154,57,129,
    196,179,138,12,156,7,14,231,13,39,204,29,255,0,191,250,
    61,165,61,97,218,175,185,28,244,179,195,156,43,191,163,55,
    212,189,14,123,123,120,42,13,177,230,237,150,122,210,71,165,
    42,9,112,153,171,221,10,147,41,158,65,212,88,135,186,95,
    81,243,17,53,59,112,77,185,162,141,139,188,160,213,72,133,
    54,70,163,186,224,90,105,138,240,27,154,155,191,235,218,201,
    251,184,182,190,5,65,2,105,195,112,142,219,121,202,20,221,
    122,57,216,224,118,129,7,23,203,80,208,228,193,27,124,127,
    161,111,92,150,41,44,216,63,54,44,176,47,93,135,23,189,
    250,73,163,129,251,240,186,197,112,127,1,69,41,113,85,36,
    48,96,162,218,127,164,35,129,182,15,22,28,227,181,137,242,
    147,69,22,207,177,58,192,18,245,132,139,87,207,208,85,234,
    185,49,57,223,243,141,194,6,54,137,124,227,93,2,82,93,
    137,18,74,252,209,72,38,193,121,149,201,95,102,173,49,242,
    240,0,206,19,62,150,148,107,248,188,139,96,75,19,150,178,
    50,82,107,21,102,239,94,147,177,195,42,162,173,76,69,52,
    151,172,169,99,88,21,190,180,105,182,47,143,29,94,121,127,
    135,199,153,255,75,131,181,7,87,115,197,187,250,240,10,250,
    236,248,96,236,229,227,92,201,152,249,190,15,29,241,166,91,
    170,137,49,54,146,34,21,7,50,146,74,94,6,43,69,178,
    22,231,199,64,98,74,74,199,120,170,225,163,1,190,71,158,
    119,45,193,251,215,180,62,173,70,9,134,130,183,141,7,195,
    117,81,183,234,130,179,228,133,139,82,189,61,210,190,46,129,
    199,185,203,142,188,84,89,143,47,243,202,20,197,30,75,199,
    184,125,63,214,151,51,124,219,224,126,64,205,135,165,237,217,
    215,244,241,136,15,30,250,224,135,222,201,41,156,51,182,187,
    75,227,84,132,199,123,187,165,76,187,90,38,23,237,208,97,
    51,240,133,98,188,199,232,120,151,238,105,148,246,142,100,160,
    111,173,212,189,171,105,158,165,177,143,227,155,151,82,116,194,
    184,224,176,114,225,123,144,209,172,245,11,163,185,204,66,63,
    10,127,208,23,93,229,48,39,200,203,182,69,105,104,106,132,
    211,43,67,107,202,118,140,163,76,30,134,40,119,198,220,166,
    102,21,145,140,172,171,156,43,48,61,201,225,58,176,167,11,
    93,125,160,127,66,14,149,127,129,13,93,111,213,151,234,194,
    54,232,46,213,20,13,209,20,150,88,108,214,205,186,93,175,
    153,136,79,26,89,21,13,179,222,88,20,231,255,219,136,216,
    134,177,189,80,23,255,3,207,99,220,241,
};

EmbeddedPython embedded_m5_internal_param_MemoryControl(
    "m5/internal/param_MemoryControl.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_MemoryControl.py",
    "m5.internal.param_MemoryControl",
    data_m5_internal_param_MemoryControl,
    2139,
    6544);

} // anonymous namespace
