#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_EtherDevice[] = {
    120,156,197,88,109,111,27,199,17,158,189,35,41,145,22,45,
    201,242,187,101,235,156,68,49,27,212,98,226,86,117,129,24,
    70,93,43,64,93,192,138,115,12,98,91,45,122,61,221,45,
    201,163,238,133,184,91,201,97,32,125,169,140,182,223,250,35,
    138,126,232,255,232,111,234,215,118,102,246,238,116,122,107,131,
    180,160,36,114,49,220,247,153,121,158,217,217,245,32,255,171,
    227,247,23,22,64,246,79,1,224,227,71,64,8,16,9,216,
    18,32,164,0,127,9,118,234,144,254,20,252,58,188,7,216,
    50,64,26,112,136,130,9,191,49,32,158,227,49,13,8,77,
    174,17,48,105,129,172,193,86,29,94,199,139,80,147,13,216,
    105,65,250,123,16,66,196,2,222,248,51,224,207,194,123,156,
    29,133,38,79,56,11,126,139,133,38,248,151,88,104,193,100,
    1,228,37,216,194,201,103,96,171,141,83,125,130,83,93,230,
    169,254,65,83,249,216,114,21,252,54,117,199,189,188,165,158,
    53,234,201,107,92,230,89,230,139,157,45,192,214,98,33,95,
    169,200,75,21,249,106,69,190,86,145,175,87,228,27,21,249,
    102,69,190,85,145,111,87,228,59,21,121,185,34,223,173,200,
    247,42,242,74,69,182,88,158,7,185,8,163,251,48,250,0,
    70,31,66,31,157,177,80,106,252,17,72,19,70,171,176,181,
    10,18,63,31,193,33,250,203,95,172,140,248,152,71,92,41,
    71,60,224,17,29,216,234,128,196,207,3,61,162,1,189,206,
    53,196,64,240,47,252,235,32,6,64,205,97,177,39,211,44,
    72,98,39,136,251,73,96,80,123,131,10,66,140,71,197,76,
    14,157,231,4,157,191,3,227,198,55,114,232,28,0,78,44,
    72,151,208,128,3,22,14,12,152,116,96,95,192,168,6,190,
    9,251,184,76,157,54,48,16,112,104,192,111,77,234,112,128,
    101,13,157,125,15,106,74,227,102,196,206,214,51,205,192,65,
    29,246,235,208,123,179,111,80,197,78,19,210,191,193,119,203,
    60,233,44,79,106,192,62,150,53,56,172,193,65,3,94,99,
    39,172,26,53,73,125,241,102,31,53,197,154,94,167,134,187,
    221,172,168,75,170,248,65,26,187,145,84,87,80,118,198,110,
    234,70,206,23,106,40,211,13,185,23,120,178,211,42,250,37,
    217,218,216,85,67,155,7,154,100,145,104,172,120,194,36,150,
    234,18,10,253,32,246,157,40,241,119,67,169,102,105,54,167,
    31,132,210,113,184,241,69,52,78,82,245,69,154,38,169,77,
    70,229,202,48,113,203,17,100,82,47,76,50,217,161,213,120,
    25,155,166,87,212,187,63,230,25,105,3,188,89,26,236,203,
    204,75,131,177,66,95,233,25,169,55,205,214,33,47,113,145,
    253,14,139,238,48,137,100,23,245,234,110,36,239,98,90,50,
    235,14,100,180,254,48,83,238,118,40,31,186,110,255,211,207,
    30,75,215,245,31,251,221,237,221,32,244,187,207,236,151,221,
    241,68,13,147,184,27,173,119,131,88,73,180,81,216,61,101,
    157,53,236,68,118,203,222,5,3,39,96,13,157,161,12,199,
    50,109,83,237,109,218,131,88,16,115,162,33,76,209,17,109,
    148,234,248,53,197,178,113,73,108,6,164,163,71,122,19,192,
    204,2,82,127,5,118,30,250,126,199,128,116,153,0,51,194,
    143,32,15,35,108,122,212,102,112,219,87,100,28,93,59,50,
    9,6,186,114,159,65,134,104,195,158,79,200,239,49,48,82,
    234,48,106,128,70,16,2,79,67,42,157,80,137,221,105,26,
    3,39,175,65,246,151,227,51,196,11,128,198,199,32,131,85,
    215,113,169,63,48,40,123,29,218,248,38,67,67,13,131,12,
    77,203,14,32,153,105,212,67,155,188,154,124,185,61,146,158,
    202,86,176,226,109,178,107,121,110,28,39,202,114,125,223,114,
    149,74,131,237,93,37,51,75,37,214,106,214,105,146,199,23,
    11,116,149,243,77,198,5,154,200,243,136,38,253,195,15,60,
    133,63,150,248,7,219,63,147,10,145,49,76,252,12,235,105,
    138,129,84,54,109,82,93,198,226,89,177,28,67,176,211,40,
    0,147,201,176,175,90,140,61,55,203,28,94,142,234,25,102,
    52,122,207,13,119,165,162,254,136,23,133,171,146,168,23,154,
    46,208,110,146,170,133,166,100,61,39,78,98,127,130,27,13,
    188,85,218,195,77,134,219,28,16,224,174,33,216,102,176,108,
    64,27,193,183,96,120,164,83,45,135,26,195,236,58,89,0,
    216,245,34,143,31,8,185,67,140,50,29,131,195,4,43,199,
    52,180,72,162,193,54,33,218,190,67,197,50,21,119,11,253,
    167,102,132,246,73,35,60,162,133,13,214,220,51,115,29,75,
    42,109,30,163,210,173,35,42,97,88,236,17,37,12,34,206,
    17,37,76,178,66,250,52,199,63,145,13,33,128,205,21,212,
    179,109,236,5,210,185,81,0,214,38,20,86,161,56,168,64,
    209,38,183,48,14,237,91,231,217,113,229,2,237,56,208,118,
    92,167,133,231,114,4,181,25,57,45,225,145,251,141,220,170,
    108,209,13,20,38,55,200,162,85,91,222,192,147,238,117,220,
    230,35,139,143,61,78,70,116,16,209,38,214,66,141,176,214,
    55,225,122,126,20,101,196,249,113,154,124,59,177,146,190,165,
    160,216,195,147,213,108,109,53,251,28,163,130,245,148,227,140,
    142,11,154,249,169,28,167,200,240,38,255,208,172,117,152,193,
    78,126,148,160,205,233,88,103,87,177,165,57,72,101,42,165,
    216,52,93,51,183,74,51,211,174,63,167,85,91,108,99,19,
    110,224,183,37,120,107,78,194,81,146,179,11,110,197,239,47,
    201,218,164,176,4,202,71,237,158,222,56,235,68,218,217,15,
    142,97,102,90,26,217,159,224,18,27,5,231,26,80,34,132,
    190,38,237,153,104,240,39,224,4,76,192,31,129,208,128,78,
    207,137,195,20,165,47,57,117,201,202,183,188,47,206,58,237,
    12,77,64,35,15,81,200,207,236,49,119,213,135,223,175,225,
    207,149,232,117,104,130,160,131,202,204,83,172,234,65,85,43,
    41,203,48,250,94,135,81,237,56,183,201,75,67,55,163,110,
    154,197,102,201,226,163,64,88,102,70,24,154,166,134,176,89,
    189,158,67,91,123,113,132,47,58,5,238,136,37,163,130,154,
    31,83,241,176,4,140,40,234,166,177,203,149,147,97,187,114,
    118,57,58,84,254,138,182,82,227,205,207,55,56,193,171,204,
    81,82,163,94,80,227,81,73,13,201,225,251,61,231,225,84,
    26,4,129,67,67,224,165,12,147,22,186,15,213,64,214,97,
    171,65,36,226,44,83,228,28,19,69,84,163,104,120,236,108,
    96,235,108,106,187,149,40,208,14,166,226,219,233,70,17,242,
    241,147,208,141,182,125,247,41,89,34,163,133,189,130,117,70,
    161,197,66,85,11,98,140,56,79,17,254,249,105,161,205,222,
    116,35,200,103,160,47,82,172,5,243,197,79,60,14,27,95,
    15,165,21,201,104,27,47,96,195,96,108,245,67,119,192,190,
    50,115,45,191,44,180,84,236,236,147,167,114,70,177,105,51,
    177,188,36,198,96,191,235,169,36,181,124,137,183,18,233,91,
    15,45,62,41,172,32,179,220,109,108,117,61,165,25,112,156,
    205,156,250,185,233,32,227,44,111,231,29,137,211,247,181,131,
    119,207,0,179,91,191,176,146,190,18,149,97,159,19,90,77,
    39,60,60,241,182,161,38,58,188,253,140,138,31,81,177,10,
    23,114,58,116,113,137,109,90,139,204,215,192,8,212,20,106,
    241,56,149,95,209,184,236,52,161,223,126,31,66,235,247,21,
    236,32,27,48,154,225,114,150,78,6,122,65,169,229,47,40,
    84,169,95,80,244,179,77,155,120,223,248,95,121,207,116,153,
    62,81,70,255,87,186,219,143,46,86,9,251,39,144,103,8,
    231,81,221,128,74,250,254,88,83,93,251,6,243,136,201,213,
    74,70,201,10,139,215,120,232,99,214,185,207,249,168,99,232,
    196,243,200,145,124,140,243,227,0,145,39,150,239,156,83,72,
    212,169,37,225,195,29,143,101,236,31,165,141,220,50,93,107,
    17,129,35,56,58,197,49,71,188,138,223,211,200,37,197,42,
    122,50,66,235,37,86,111,95,136,155,211,50,92,93,62,22,
    174,108,242,163,14,80,101,108,210,78,89,63,43,52,56,252,
    140,194,203,245,93,79,58,24,206,99,204,142,233,73,206,75,
    118,99,69,55,150,31,56,18,147,15,78,69,254,75,63,118,
    3,223,183,125,25,74,37,79,131,70,145,54,249,101,207,151,
    120,158,36,19,188,132,112,46,143,191,67,199,185,128,216,251,
    115,208,175,148,124,131,162,216,219,192,91,92,179,214,20,124,
    188,157,120,201,212,91,35,119,232,140,117,146,217,76,208,249,
    210,55,252,222,86,156,46,204,68,186,113,109,186,145,126,65,
    225,71,1,251,62,21,31,22,158,101,22,233,187,12,223,19,
    244,29,13,89,199,103,47,31,181,246,26,213,211,205,55,90,
    95,43,244,89,211,250,188,242,2,173,13,63,249,69,235,234,
    246,217,221,66,87,245,147,52,82,119,207,108,126,17,171,244,
    121,130,69,18,178,11,79,247,232,77,50,37,163,83,155,144,
    241,110,228,188,148,81,146,78,94,38,190,84,247,79,180,63,
    203,243,6,221,197,217,147,148,96,40,235,204,37,142,247,61,
    71,95,108,212,207,99,140,201,211,237,207,195,196,219,145,126,
    222,231,108,109,185,207,70,18,185,88,127,246,42,189,160,88,
    101,241,68,187,159,210,168,107,39,106,51,153,6,110,24,124,
    39,213,242,73,3,248,126,106,187,241,64,22,186,159,189,224,
    70,228,230,110,60,199,205,65,146,183,19,196,138,70,69,200,
    59,137,3,194,79,249,139,147,11,118,105,5,248,76,193,84,
    14,2,116,105,202,115,148,253,243,240,78,180,80,247,206,12,
    23,213,177,211,39,172,78,235,245,147,197,83,186,239,100,223,
    96,65,143,119,205,249,166,104,24,244,78,108,138,150,104,139,
    154,152,107,55,205,102,163,89,55,145,212,84,179,36,90,102,
    179,53,39,254,243,255,10,146,191,101,172,180,154,226,223,12,
    142,54,136,
};

EmbeddedPython embedded_m5_internal_param_EtherDevice(
    "m5/internal/param_EtherDevice.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_EtherDevice.py",
    "m5.internal.param_EtherDevice",
    data_m5_internal_param_EtherDevice,
    2243,
    6900);

} // anonymous namespace
