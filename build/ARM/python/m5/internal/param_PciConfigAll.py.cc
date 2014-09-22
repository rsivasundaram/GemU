#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_PciConfigAll[] = {
    120,156,197,89,109,111,219,200,17,158,37,37,217,146,173,216,
    142,237,188,58,103,166,57,95,212,67,99,221,165,77,83,244,
    130,160,185,184,64,83,32,190,148,58,32,57,181,40,75,147,
    43,153,50,69,10,228,218,57,29,236,47,117,208,246,67,129,
    254,136,162,31,250,63,250,107,250,39,122,51,179,36,77,219,
    114,238,128,43,228,88,90,140,246,101,118,103,230,153,151,221,
    120,144,253,171,226,247,87,22,64,250,95,1,224,227,71,64,
    8,48,20,208,21,32,164,0,127,25,246,170,144,252,12,252,
    42,188,3,232,26,32,13,56,70,194,132,223,27,16,205,243,
    154,26,132,38,247,8,24,55,64,86,160,91,133,215,209,18,
    84,100,13,246,26,144,252,9,132,16,145,128,55,254,12,248,
    179,240,14,185,35,81,103,134,179,224,55,152,168,131,63,199,
    68,3,198,139,32,231,160,139,204,103,160,219,68,86,31,35,
    171,43,204,234,63,196,202,199,145,21,240,155,52,29,207,242,
    21,205,172,208,76,222,227,10,115,89,200,79,182,8,221,165,
    156,190,90,162,151,75,244,74,137,94,45,209,215,74,244,245,
    18,125,163,68,223,44,209,183,74,244,237,18,189,86,162,239,
    148,232,15,74,244,58,211,11,32,151,96,96,193,224,46,12,
    126,4,61,52,192,98,33,229,61,144,38,12,62,132,238,135,
    32,241,115,15,142,209,70,254,82,105,197,6,175,184,90,172,
    248,136,87,220,135,238,125,144,248,249,72,175,168,65,167,181,
    138,118,15,254,135,255,90,104,119,80,243,216,28,200,36,13,
    226,200,9,162,94,28,24,52,94,163,134,80,226,81,51,147,
    193,229,57,193,229,223,192,88,241,141,12,46,71,128,140,5,
    201,18,26,112,196,196,145,1,227,22,28,10,24,84,192,55,
    225,16,183,169,210,1,250,2,142,13,248,131,73,19,142,176,
    173,160,129,63,128,138,210,88,25,176,129,53,167,25,56,170,
    194,97,21,58,111,14,13,234,216,171,67,242,47,248,102,141,
    153,206,50,83,3,14,177,173,192,113,5,142,106,240,26,39,
    97,215,160,78,226,139,55,135,40,41,246,116,90,21,60,237,
    118,73,92,18,197,15,146,200,29,74,181,140,180,51,114,19,
    119,232,188,242,130,231,113,212,11,250,207,194,176,213,200,39,
    198,233,230,200,85,187,54,175,52,73,37,195,145,98,142,113,
    36,213,28,18,189,32,242,157,97,236,239,135,82,205,18,59,
    167,23,132,210,113,120,240,197,112,20,39,234,215,73,18,39,
    54,105,149,59,195,216,45,86,144,78,189,48,78,101,139,118,
    227,109,108,98,175,104,118,111,196,28,233,0,124,90,90,236,
    203,212,75,130,145,66,99,105,142,52,155,184,181,200,76,220,
    164,14,54,237,221,120,40,219,40,88,123,43,126,27,209,150,
    105,187,47,135,143,30,164,202,221,9,229,3,215,237,125,242,
    233,99,233,186,254,99,191,189,179,31,132,126,251,153,253,178,
    61,26,171,221,56,106,15,31,181,131,72,73,84,82,216,62,
    175,158,77,156,117,149,54,122,27,244,157,128,69,116,118,101,
    56,146,73,147,122,111,209,33,196,162,152,23,53,97,138,150,
    104,34,85,197,175,41,214,140,57,177,29,144,144,30,9,78,
    16,51,115,80,253,19,216,124,104,253,61,3,146,53,130,204,
    0,63,130,108,140,192,233,208,152,193,99,191,35,237,232,222,
    129,73,64,208,157,135,12,51,196,27,206,124,66,150,143,128,
    177,82,133,65,13,52,134,16,122,26,84,201,152,90,156,78,
    108,12,100,94,129,244,31,167,57,68,139,128,218,199,208,130,
    93,215,112,171,63,51,44,59,45,58,248,54,99,67,237,6,
    41,234,150,45,64,52,59,82,7,117,242,106,252,197,206,64,
    122,42,93,199,142,175,226,125,203,115,163,40,86,150,235,251,
    150,171,84,18,236,236,43,153,90,42,182,54,210,86,157,76,
    190,148,195,171,224,55,30,229,112,34,211,35,156,244,15,63,
    240,20,254,96,220,58,172,255,84,42,132,198,110,236,167,216,
    79,44,250,82,217,116,72,117,5,155,103,249,118,140,193,86,
    45,71,76,42,195,158,106,48,248,220,52,117,120,59,234,103,
    156,209,234,3,55,220,151,138,230,35,96,20,238,74,164,222,
    104,202,72,187,65,178,230,162,146,250,156,40,142,252,49,158,
    52,240,54,232,16,55,24,111,243,64,136,91,69,180,205,96,
    91,131,38,162,111,209,240,72,168,74,134,53,198,217,53,82,
    1,176,237,69,22,66,16,115,199,24,104,90,6,71,10,150,
    142,29,209,34,138,22,219,4,105,251,54,53,107,212,220,201,
    21,48,61,45,52,207,106,225,33,237,108,176,232,158,153,9,
    89,56,211,246,41,103,186,121,226,76,24,26,59,228,20,6,
    185,206,137,83,152,164,134,228,105,230,1,228,110,8,2,28,
    46,225,158,149,99,47,146,208,181,28,178,54,225,176,12,198,
    126,9,140,54,217,133,145,104,223,188,72,145,235,151,169,200,
    190,86,228,35,218,121,62,195,80,147,177,211,16,30,1,192,
    200,212,202,42,221,66,98,124,157,84,90,86,230,117,76,119,
    175,163,38,231,45,206,125,92,133,232,56,162,117,172,137,10,
    161,173,103,194,181,44,31,165,228,246,163,36,254,122,108,197,
    61,75,65,126,134,39,27,233,230,70,250,25,6,6,235,41,
    135,26,29,26,180,243,39,114,148,160,147,215,249,135,118,92,
    135,157,216,201,210,9,42,157,114,59,219,138,85,205,113,42,
    85,9,133,167,41,235,185,81,232,153,142,253,25,109,219,96,
    37,155,112,29,191,13,193,103,115,98,142,148,92,99,240,40,
    126,63,39,117,147,196,18,168,18,181,59,250,228,44,20,137,
    103,223,63,133,154,169,137,100,127,140,123,108,229,110,87,131,
    2,35,244,53,233,208,228,9,127,5,174,195,4,252,5,8,
    15,104,246,204,119,216,75,233,75,102,93,166,233,127,4,14,
    66,19,82,158,161,125,208,200,194,20,186,104,250,152,167,234,
    12,248,91,248,91,41,130,29,155,32,40,91,153,89,165,85,
    206,86,149,194,107,25,72,223,43,35,85,78,187,55,153,105,
    215,77,105,154,118,100,179,112,228,147,96,88,212,71,24,157,
    166,135,177,89,189,161,67,103,123,113,130,48,74,5,183,197,
    178,81,194,205,79,168,121,80,64,70,228,125,83,57,230,250,
    217,216,93,202,96,142,142,151,191,161,179,84,248,244,11,53,
    46,36,202,76,10,247,168,230,238,241,176,112,15,201,65,252,
    29,87,228,212,26,132,130,99,67,224,149,12,139,23,186,13,
    85,64,86,161,91,35,71,226,114,83,100,126,38,242,208,70,
    33,241,84,134,96,253,108,107,205,21,64,208,54,166,230,235,
    41,135,18,50,243,147,208,29,238,248,238,211,29,218,148,118,
    246,114,207,51,114,49,22,203,98,144,215,136,139,36,225,159,
    159,228,226,28,76,57,140,124,138,123,20,98,176,211,248,177,
    199,177,227,203,93,105,13,229,112,7,47,99,187,193,200,234,
    133,110,159,173,101,102,98,126,145,139,169,216,220,103,179,115,
    74,1,106,59,182,188,56,194,152,191,239,169,56,177,124,137,
    23,20,233,91,15,44,78,24,86,144,90,238,14,142,186,158,
    210,94,112,218,165,185,8,116,147,126,202,245,222,222,91,34,
    47,193,218,14,94,68,3,44,116,189,92,77,250,122,84,68,
    127,174,109,181,75,97,18,197,139,135,26,235,32,247,115,106,
    126,76,205,6,92,78,146,104,147,2,105,51,82,96,13,227,
    80,93,240,101,169,60,239,21,173,76,207,123,245,223,191,143,
    87,235,39,150,204,183,107,52,83,206,208,141,152,218,58,165,
    138,110,35,239,156,227,118,158,59,155,121,231,21,110,23,184,
    115,49,127,218,89,226,206,171,208,93,166,247,8,234,89,161,
    120,49,243,67,227,5,123,217,37,248,87,240,127,13,19,246,
    195,75,150,194,254,41,100,229,197,69,33,66,148,69,108,234,
    16,49,16,121,53,95,150,143,95,13,110,78,68,164,227,37,
    210,85,82,91,238,214,212,101,230,152,163,143,176,87,216,47,
    23,172,184,219,60,46,132,59,230,10,107,188,82,170,182,217,
    156,226,53,150,67,88,145,31,178,252,142,161,139,242,19,156,
    86,10,53,208,101,48,146,111,157,243,170,208,117,55,29,200,
    29,141,100,228,159,212,212,60,50,101,48,80,88,139,225,164,
    192,193,2,122,5,191,231,93,147,68,43,73,202,134,172,22,
    206,56,125,147,50,140,85,110,76,126,121,60,137,226,54,153,
    82,199,237,34,100,219,191,44,140,115,107,50,70,119,246,83,
    186,186,189,111,24,203,44,206,171,72,171,219,147,167,165,193,
    55,146,217,188,111,156,248,240,107,9,254,224,42,110,194,196,
    81,232,170,94,156,12,153,217,119,205,33,134,252,164,151,117,
    176,97,217,25,125,25,74,37,39,0,81,145,126,178,235,181,
    47,49,115,199,99,188,245,241,221,9,127,135,142,115,25,73,
    238,23,184,199,46,109,70,153,141,146,92,13,211,220,42,255,
    213,43,117,193,245,196,153,103,100,125,66,106,245,61,97,156,
    218,28,217,22,10,123,179,98,242,108,206,94,78,55,221,109,
    119,168,31,175,248,57,198,190,75,205,189,28,50,236,160,250,
    10,201,183,51,125,55,70,143,230,98,135,107,27,123,51,199,
    210,240,209,102,46,214,102,38,86,102,6,126,109,29,62,82,
    119,38,206,122,17,169,4,165,71,205,135,252,24,117,126,70,
    103,156,42,57,100,40,149,7,101,180,63,116,94,202,97,156,
    140,95,198,190,84,119,207,140,63,203,10,49,61,197,57,144,
    84,177,41,107,226,22,167,231,158,219,74,79,194,65,253,242,
    200,64,60,63,254,60,140,189,61,233,103,115,38,75,203,115,
    182,226,161,139,253,147,119,233,4,249,46,75,103,198,253,132,
    86,173,158,233,77,101,18,184,33,185,208,218,89,5,248,126,
    98,187,81,95,190,95,246,207,221,52,240,94,5,241,150,60,
    8,60,121,193,169,78,198,9,71,249,160,90,33,80,77,226,
    66,21,228,233,46,46,221,206,101,70,246,188,68,246,3,52,
    113,194,236,78,175,202,210,5,121,196,69,238,95,230,112,9,
    222,170,239,79,250,133,232,41,253,31,66,250,37,54,244,90,
    90,95,168,139,154,65,47,243,166,104,136,166,168,136,249,102,
    221,172,215,234,85,19,61,154,122,150,69,195,172,55,230,197,
    251,254,214,209,227,27,198,122,179,46,190,5,75,106,139,30,
};

EmbeddedPython embedded_m5_internal_param_PciConfigAll(
    "m5/internal/param_PciConfigAll.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_PciConfigAll.py",
    "m5.internal.param_PciConfigAll",
    data_m5_internal_param_PciConfigAll,
    2320,
    7258);

} // anonymous namespace