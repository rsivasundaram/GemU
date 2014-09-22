#include "sim/init.hh"

namespace {

const uint8_t data_m5_util_grammar[] = {
    120,156,181,87,109,111,27,199,17,158,61,190,137,52,101,41,
    182,171,90,105,146,158,219,42,98,155,216,108,139,188,52,129,
    32,36,78,156,180,105,32,4,167,160,69,141,2,236,233,110,
    73,29,117,188,35,110,151,145,4,72,72,97,7,104,190,228,
    255,228,111,228,31,228,67,255,68,63,20,237,60,179,183,148,
    156,6,69,190,80,36,87,179,187,179,59,115,51,207,188,92,
    66,245,95,131,127,239,132,68,230,83,38,82,254,42,202,137,
    30,47,105,69,143,3,79,55,132,14,72,55,105,172,40,109,
    208,23,68,79,137,254,242,184,69,105,147,116,91,86,91,203,
    213,14,14,29,14,218,124,113,246,31,254,59,176,61,38,63,
    137,43,163,31,85,85,89,37,94,135,128,127,15,161,195,29,
    38,52,75,87,56,201,66,191,96,233,13,138,14,7,224,72,
    26,181,186,248,189,7,246,23,153,176,68,83,69,23,68,23,
    138,158,6,164,46,2,76,70,162,243,225,64,49,199,193,0,
    7,108,151,135,71,103,137,158,219,172,44,236,26,207,70,163,
    172,200,236,104,100,91,216,47,79,116,225,56,155,60,24,157,
    143,109,135,137,153,54,38,158,232,8,59,3,242,131,249,136,
    135,225,113,57,211,195,42,158,13,223,47,79,139,188,140,83,
    51,156,232,217,235,247,141,141,143,114,125,63,142,199,191,254,
    205,155,58,142,211,55,211,161,169,146,225,252,220,30,151,197,
    112,246,250,112,97,179,124,56,225,163,179,184,122,48,63,143,
    240,124,247,112,47,100,147,218,84,7,3,80,181,154,69,60,
    211,172,102,79,38,179,50,93,228,152,130,225,160,44,180,28,
    190,82,109,37,250,225,206,16,247,66,86,91,137,101,62,116,
    219,75,47,54,189,23,63,185,242,34,1,12,0,141,18,162,
    1,244,128,104,2,61,105,23,176,1,96,24,88,109,74,219,
    178,197,168,233,8,177,70,233,154,16,93,32,0,15,159,168,
    26,44,248,253,17,162,118,72,144,201,190,63,105,83,117,23,
    104,96,9,79,2,42,4,16,64,130,114,72,128,255,14,44,
    176,232,12,104,94,90,146,97,102,194,184,8,179,60,215,147,
    56,15,99,107,171,236,104,97,181,160,206,222,228,225,93,191,
    36,184,21,71,228,250,108,116,114,26,87,19,35,108,145,184,
    11,215,215,139,171,114,133,125,78,208,105,23,243,143,245,153,
    174,62,136,19,91,86,231,47,99,177,45,216,233,243,71,12,
    213,172,67,69,12,245,249,255,49,20,187,162,94,175,200,42,
    154,6,52,109,32,10,158,42,250,107,64,151,252,101,134,6,
    77,37,202,121,209,252,89,142,183,188,157,3,28,103,239,237,
    49,19,251,46,196,108,141,246,174,92,208,118,46,64,4,30,
    68,48,222,15,48,190,216,178,92,216,249,194,26,60,5,63,
    186,65,132,124,122,172,67,183,28,142,51,62,62,91,24,27,
    234,34,13,79,51,123,28,194,64,221,229,193,52,171,178,127,
    115,226,145,37,54,180,147,56,0,120,163,27,88,130,231,74,
    35,145,52,143,237,177,36,2,51,207,51,27,109,248,232,227,
    171,13,174,182,56,112,30,39,137,247,122,211,123,61,90,7,
    47,116,100,121,242,159,69,173,206,255,183,188,255,37,145,122,
    0,188,66,117,250,20,0,4,125,117,87,109,168,134,234,170,
    158,122,46,72,124,204,52,125,230,252,189,18,231,48,34,78,
    2,170,94,160,9,137,163,100,156,178,175,176,23,200,222,91,
    178,164,144,209,21,168,192,109,54,100,243,79,178,217,88,110,
    54,221,102,83,54,255,65,108,207,105,27,95,198,202,133,187,
    59,160,175,132,149,49,129,105,219,29,104,203,129,175,229,64,
    7,223,171,3,77,127,160,35,211,142,59,208,145,3,223,214,
    234,154,111,4,96,110,194,208,219,230,204,178,45,124,61,225,
    91,83,53,223,191,40,189,241,12,95,191,230,91,23,190,87,
    60,223,61,197,187,194,215,165,105,143,14,217,249,233,77,178,
    125,200,123,42,81,52,14,104,235,73,93,100,54,150,169,37,
    71,72,154,168,15,84,108,121,84,240,162,128,94,96,6,4,
    89,0,38,89,84,149,46,236,72,142,72,121,204,200,167,27,
    191,103,202,69,149,104,137,235,189,130,51,253,126,134,184,182,
    253,235,199,179,66,75,92,236,238,152,221,176,60,154,234,196,
    134,199,177,9,139,242,42,150,66,108,14,32,53,122,30,3,
    100,56,37,183,159,209,116,158,159,71,63,193,202,11,24,128,
    254,232,69,159,214,32,168,40,93,216,72,174,59,159,235,107,
    153,15,3,228,173,14,247,55,164,250,77,180,133,152,209,232,
    45,136,120,187,6,124,87,117,24,242,189,107,227,75,215,198,
    174,106,170,141,239,252,239,5,13,101,240,132,123,134,141,84,
    76,246,197,250,9,18,0,124,213,241,129,242,79,146,22,131,
    61,110,165,191,224,138,99,165,156,89,159,34,183,234,36,42,
    96,200,91,116,217,170,145,205,88,229,184,184,100,160,51,189,
    38,253,137,114,97,34,160,226,85,206,156,140,35,172,102,77,
    57,116,131,166,125,196,214,27,245,100,29,177,228,39,55,17,
    21,126,178,129,160,121,227,146,131,130,67,102,147,184,42,96,
    245,22,93,116,160,229,37,139,91,163,233,109,4,10,203,97,
    48,95,72,185,189,96,141,169,113,217,165,90,139,59,46,114,
    187,228,234,235,129,249,57,220,88,77,22,51,198,150,75,173,
    71,58,140,67,103,163,87,195,83,6,22,176,36,120,21,143,
    204,43,206,169,9,90,42,35,48,137,133,22,56,76,74,91,
    74,230,213,40,154,227,69,145,72,126,21,192,11,149,234,163,
    197,68,146,172,173,226,228,132,37,12,126,132,9,84,201,76,
    198,55,198,69,162,101,122,20,27,237,148,16,4,70,63,3,
    31,130,97,44,192,45,244,169,0,87,174,77,114,14,21,161,
    178,130,139,128,64,222,233,54,159,115,54,119,224,70,227,16,
    161,106,70,187,24,6,62,221,123,169,81,232,87,62,142,92,
    162,149,27,231,32,93,172,148,115,177,217,21,252,211,152,161,
    9,57,46,108,163,95,97,23,121,57,26,98,248,229,146,47,
    75,172,240,205,221,189,32,43,109,22,185,93,93,240,244,189,
    184,145,179,226,223,32,35,148,232,65,153,184,205,81,177,161,
    214,213,22,211,235,242,217,12,238,114,244,172,171,196,151,13,
    252,164,147,40,190,19,17,213,46,104,110,17,108,80,151,20,
    0,80,81,241,192,179,5,194,246,16,52,215,10,230,44,110,
    195,18,28,49,108,83,23,22,8,34,23,53,96,146,192,97,
    196,114,124,124,21,44,123,55,184,187,50,191,253,62,132,106,
    46,208,186,90,2,53,44,43,105,14,174,0,59,144,100,246,
    42,134,251,222,15,224,16,2,237,245,21,172,162,61,207,80,
    233,56,189,86,230,31,80,93,235,163,223,209,10,123,109,193,
    187,243,149,104,8,9,155,181,167,218,234,22,143,125,206,93,
    155,174,168,195,50,45,159,171,208,165,179,249,170,161,116,122,
    82,201,96,77,169,87,83,121,113,251,177,115,193,184,73,91,
    220,214,21,155,210,250,9,35,191,185,97,137,159,219,181,115,
    236,177,39,117,231,44,239,25,230,167,60,30,158,23,54,62,
    11,37,158,185,186,132,59,230,237,157,148,191,189,253,253,29,
    179,183,39,213,201,108,127,15,35,250,179,114,204,252,3,40,
    27,161,251,139,238,45,43,139,62,155,115,19,134,157,207,226,
    124,161,229,101,67,124,238,98,75,106,158,196,220,204,76,86,
    103,118,164,252,249,72,84,254,59,174,239,137,205,219,28,17,
    47,7,155,106,105,237,101,11,245,26,249,142,122,42,217,127,
    123,253,186,177,199,13,103,207,198,255,218,83,208,108,144,133,
    254,80,55,189,201,113,204,9,208,50,130,119,76,248,78,40,
    54,149,122,228,208,247,158,55,87,244,16,195,179,230,137,222,
    199,240,104,165,136,132,105,172,51,205,151,180,124,209,104,112,
    130,56,112,57,16,11,145,180,213,210,90,72,87,241,11,242,
    61,4,2,42,122,23,195,7,24,62,164,149,191,174,162,234,
    32,195,27,168,211,86,221,86,247,102,119,247,214,157,110,191,
    187,230,50,1,154,104,231,241,252,252,1,26,52,228,66,73,
    247,88,64,147,22,41,111,107,247,38,34,157,149,92,188,90,
    221,69,137,61,247,166,178,143,154,38,93,10,58,250,126,176,
    213,250,47,247,2,181,171,
};

EmbeddedPython embedded_m5_util_grammar(
    "m5/util/grammar.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/python/m5/util/grammar.py",
    "m5.util.grammar",
    data_m5_util_grammar,
    1863,
    4535);

} // anonymous namespace
