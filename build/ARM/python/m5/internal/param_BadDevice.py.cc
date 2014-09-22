#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BadDevice[] = {
    120,156,197,88,235,110,219,216,17,158,67,82,178,37,91,190,
    196,151,220,156,152,217,93,183,234,162,177,118,211,166,41,186,
    65,208,236,122,129,110,129,120,83,170,64,178,234,162,92,154,
    60,146,41,75,164,64,30,59,171,133,253,167,14,218,254,235,
    67,20,253,209,247,232,19,244,133,218,153,57,36,69,203,54,
    186,64,91,197,22,15,14,207,125,102,190,111,102,14,125,200,
    254,42,248,252,210,6,72,255,41,0,2,252,9,24,0,12,
    5,116,4,8,41,32,88,131,163,10,36,63,133,160,2,111,
    1,58,6,72,3,206,177,98,194,239,12,136,22,121,78,21,
    6,38,183,8,24,215,65,90,208,169,192,171,104,21,44,89,
    133,163,58,36,223,128,16,34,18,240,58,152,131,96,30,222,
    226,234,88,169,241,130,243,16,212,185,82,131,96,129,43,117,
    24,175,128,92,128,14,46,62,7,157,6,46,245,33,46,181,
    196,75,253,131,150,10,176,103,29,130,6,13,199,179,124,69,
    35,45,26,201,123,44,241,42,203,249,201,86,160,179,154,215,
    111,148,234,107,165,250,122,169,190,81,170,111,150,234,55,75,
    245,91,165,250,237,82,253,78,169,126,183,84,223,42,213,239,
    113,125,25,228,42,244,239,67,127,27,250,54,116,81,209,43,
    133,52,15,64,154,208,127,15,58,239,129,196,223,3,56,71,
    91,4,171,165,25,239,243,140,27,197,140,15,120,198,14,116,
    118,64,226,239,3,61,163,10,237,230,6,218,55,252,23,254,
    53,209,190,160,22,177,56,145,73,26,198,145,27,70,221,56,
    52,168,191,74,5,161,193,167,98,46,131,197,103,4,139,191,
    3,99,34,48,50,88,156,1,46,44,72,150,129,1,103,92,
    57,51,96,220,132,83,1,125,11,2,19,78,113,155,10,29,
    160,39,224,220,128,175,77,26,112,134,165,133,134,188,15,150,
    210,152,232,179,33,245,74,115,112,86,129,211,10,180,95,159,
    26,212,112,84,131,228,111,240,221,22,47,58,207,139,26,112,
    138,165,5,231,22,156,85,225,21,14,194,166,126,141,196,23,
    175,79,81,82,108,105,55,45,60,237,126,73,92,18,37,8,
    147,200,27,74,181,130,117,119,228,37,222,208,253,212,11,246,
    228,73,232,203,102,61,31,21,167,187,35,79,29,58,60,205,
    36,125,12,71,138,151,139,35,169,22,176,210,13,163,192,29,
    198,193,241,64,170,121,90,203,237,134,3,233,186,220,249,197,
    112,20,39,234,243,36,137,19,135,84,202,141,131,216,43,102,
    144,66,253,65,156,202,38,237,198,219,56,180,188,162,209,221,
    17,175,72,7,224,163,210,228,64,166,126,18,142,20,90,74,
    175,72,163,105,181,38,217,136,139,244,107,44,90,135,241,80,
    182,80,170,214,94,252,38,162,45,211,86,79,14,31,63,76,
    149,119,48,144,15,61,175,251,209,199,79,164,231,5,79,130,
    214,193,113,56,8,90,207,157,23,173,209,88,29,198,81,107,
    248,184,21,70,74,162,134,6,173,41,221,236,226,144,27,180,
    203,155,176,231,134,44,159,123,40,7,35,153,52,168,245,14,
    157,64,172,136,69,81,21,166,104,138,6,214,42,248,152,98,
    203,88,16,251,33,73,232,147,212,4,46,51,135,211,95,129,
    13,135,118,63,50,32,217,34,176,244,241,39,200,186,8,153,
    54,245,25,220,247,27,82,141,110,237,155,4,1,221,120,202,
    0,67,164,225,200,167,100,243,8,24,37,21,232,87,65,163,
    7,65,167,225,148,140,169,196,225,180,140,129,139,91,144,254,
    229,226,10,209,10,160,234,209,121,96,211,38,110,245,7,6,
    100,187,73,7,223,103,96,168,195,48,69,197,178,250,169,206,
    20,106,163,78,94,142,191,60,232,75,95,165,219,216,240,85,
    124,108,251,94,20,197,202,246,130,192,246,148,74,194,131,99,
    37,83,91,197,246,78,218,172,145,189,87,115,108,21,235,141,
    71,57,150,200,238,136,37,253,18,132,190,194,151,53,126,97,
    253,167,82,33,46,14,227,32,197,118,90,162,39,149,67,135,
    84,75,88,60,207,183,99,0,54,171,57,92,82,57,232,170,
    58,35,207,75,83,151,183,163,118,6,25,205,62,241,6,199,
    82,209,120,68,139,194,93,169,170,55,154,37,204,110,145,160,
    185,156,164,59,55,138,163,96,140,199,12,253,29,58,193,45,
    6,219,34,16,220,54,16,106,115,88,86,161,129,208,91,49,
    124,146,200,202,128,198,32,219,36,249,129,13,47,50,207,129,
    128,59,71,255,210,52,216,65,176,104,76,65,155,106,52,217,
    33,60,59,119,169,216,162,226,94,46,253,140,84,208,152,86,
    193,35,218,214,96,185,125,51,147,176,160,209,254,5,26,221,
    158,208,8,221,97,155,232,96,16,105,38,116,48,73,7,201,
    179,12,251,68,52,52,63,118,151,16,207,154,113,200,73,50,
    126,24,172,14,33,176,12,195,94,9,134,14,25,133,49,232,
    220,190,78,139,219,239,76,139,61,173,197,199,180,237,98,134,
    158,6,163,166,46,124,50,189,145,233,148,245,185,135,149,241,
    77,210,103,89,147,55,49,190,189,138,26,28,168,56,216,113,
    122,161,221,135,86,176,174,88,132,179,174,9,155,89,0,74,
    137,237,163,36,254,118,108,199,93,91,65,126,134,167,59,233,
    238,78,250,9,250,3,251,25,123,24,237,17,52,231,19,57,
    74,144,219,53,126,209,124,117,153,187,110,22,66,80,227,20,
    204,217,80,172,103,118,79,169,74,200,43,205,82,201,245,66,
    201,116,230,79,104,207,58,107,216,132,155,248,212,5,31,204,
    141,217,59,114,70,193,189,248,124,74,186,38,113,37,80,126,
    233,180,245,177,89,34,146,205,249,225,5,188,204,70,30,231,
    67,220,96,47,103,91,21,10,116,208,99,210,137,137,0,127,
    2,78,185,4,252,17,8,9,104,240,140,50,76,78,122,200,
    160,107,52,252,247,192,142,231,138,24,103,104,234,25,153,107,
    66,102,166,79,120,168,14,121,191,134,63,151,188,214,185,9,
    130,194,147,153,37,85,229,240,100,21,100,101,8,125,175,16,
    100,93,100,53,217,232,208,75,105,152,230,175,89,240,119,226,
    0,139,108,8,157,210,140,208,53,175,119,115,233,96,95,76,
    176,69,190,255,174,88,51,74,136,249,49,21,15,11,176,136,
    188,237,255,127,198,237,105,103,93,138,87,174,118,144,191,162,
    131,88,124,244,229,42,179,186,88,161,160,68,37,167,196,163,
    130,18,146,93,246,91,206,185,169,52,200,248,231,134,192,203,
    21,38,41,116,175,177,64,86,160,83,37,242,112,78,41,50,
    110,137,220,151,145,15,188,16,15,88,51,251,90,103,133,253,
    181,105,169,248,118,150,190,131,172,251,116,224,13,15,2,239,
    217,55,180,35,109,235,231,108,51,114,25,86,202,50,16,83,
    196,117,98,240,235,71,185,44,39,179,244,27,31,227,6,133,
    12,204,146,32,246,217,89,252,246,80,218,67,57,60,192,139,
    214,97,56,178,187,3,175,199,118,50,51,25,191,204,101,84,
    108,232,233,40,156,146,71,218,143,109,63,142,208,189,31,251,
    42,78,236,64,226,253,67,6,246,67,155,99,131,29,166,182,
    119,128,189,158,175,52,242,47,114,152,211,60,47,233,165,156,
    209,29,189,161,234,172,237,236,226,13,51,196,60,214,203,117,
    164,175,62,133,163,231,212,85,211,8,131,37,222,43,212,88,
    187,180,159,81,241,35,42,118,224,29,196,131,22,29,139,118,
    34,213,85,209,235,212,132,90,46,19,248,37,205,73,47,211,
    248,224,251,208,88,127,29,201,200,92,165,145,114,142,46,185,
    84,214,40,36,116,234,249,183,150,5,110,92,164,15,27,120,
    151,167,150,37,162,253,220,127,75,123,230,203,172,153,210,251,
    159,178,221,121,244,46,69,112,126,2,89,90,112,29,211,69,
    89,190,134,102,122,95,228,201,119,89,56,190,219,111,94,134,
    151,235,39,210,83,82,27,236,206,108,165,101,167,161,247,15,
    11,179,229,34,21,151,144,39,133,88,231,156,19,141,215,75,
    153,49,91,81,188,194,4,6,179,231,83,150,220,53,116,2,
    61,193,166,85,40,128,116,25,201,55,238,148,18,116,130,76,
    167,241,70,35,25,5,147,228,151,123,102,9,0,114,74,71,
    48,201,71,48,211,93,199,231,50,17,73,168,146,140,108,191,
    74,65,189,25,91,146,113,59,202,109,216,36,55,54,113,192,
    14,89,80,187,220,194,219,58,191,40,108,114,255,10,80,6,
    252,194,31,43,240,134,245,159,134,96,118,196,183,133,73,19,
    171,145,225,30,200,129,84,114,218,224,250,43,157,14,10,129,
    196,248,22,143,241,26,196,247,9,124,31,184,238,204,163,193,
    207,113,131,46,237,68,209,138,162,65,21,227,193,134,168,89,
    53,193,225,118,234,11,170,62,218,15,32,207,155,199,169,195,
    30,99,185,80,43,127,233,203,227,29,115,136,178,195,125,84,
    142,254,124,104,211,200,7,84,188,159,91,134,25,160,239,83,
    124,91,209,183,68,228,11,231,2,28,250,157,93,106,167,185,
    195,199,187,185,60,187,185,60,105,232,191,12,99,45,20,127,
    113,28,62,86,119,175,28,59,25,118,235,202,254,246,56,85,
    114,120,105,178,140,142,135,238,11,57,140,147,241,139,56,144,
    234,193,84,255,243,44,87,209,67,220,19,73,73,205,53,199,
    189,56,246,154,115,98,167,254,252,198,201,248,229,254,207,6,
    177,127,36,131,108,204,189,235,199,236,197,67,15,219,175,222,
    165,29,230,187,172,78,245,7,9,205,218,152,106,77,101,18,
    122,131,240,59,169,182,166,21,16,4,137,227,69,61,153,203,
    78,48,200,123,213,58,97,226,42,91,45,49,193,202,77,156,
    158,92,12,24,76,151,68,246,66,52,77,194,107,93,156,146,
    249,82,66,50,159,107,154,177,229,233,179,230,151,190,17,232,
    47,28,207,232,187,119,250,18,11,250,206,87,91,174,137,170,
    65,31,148,77,81,23,13,97,137,197,70,205,172,85,107,21,
    19,57,72,45,107,162,110,214,234,139,226,186,255,109,228,104,
    221,216,94,168,137,127,3,138,253,24,203,
};

EmbeddedPython embedded_m5_internal_param_BadDevice(
    "m5/internal/param_BadDevice.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_BadDevice.py",
    "m5.internal.param_BadDevice",
    data_m5_internal_param_BadDevice,
    2235,
    6895);

} // anonymous namespace
