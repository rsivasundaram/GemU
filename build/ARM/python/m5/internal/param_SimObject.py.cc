#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_SimObject[] = {
    120,156,197,89,109,111,219,200,17,158,37,41,217,146,37,91,
    126,141,157,184,13,139,34,168,122,109,172,187,235,229,82,244,
    2,227,94,82,180,87,160,190,59,234,218,36,186,180,44,45,
    174,36,202,18,169,146,235,36,58,216,95,234,160,237,135,2,
    253,17,69,63,244,127,220,255,106,103,102,73,154,114,108,92,
    1,183,178,45,46,150,203,229,236,188,60,51,59,179,236,66,
    250,87,194,235,67,27,32,249,66,0,248,248,19,48,2,24,
    11,232,8,16,82,128,191,14,71,37,136,223,3,191,4,175,
    1,58,6,72,3,206,176,99,194,87,6,132,53,126,167,12,
    35,147,71,4,76,171,32,45,232,148,224,73,184,10,150,44,
    195,81,21,226,63,128,16,34,20,240,212,95,0,127,17,94,
    35,117,236,84,152,224,34,248,85,238,84,192,95,226,78,21,
    166,13,144,75,208,65,226,11,208,169,35,169,183,144,212,50,
    147,250,134,72,249,248,100,3,252,58,77,71,94,158,209,76,
    139,102,242,26,203,76,101,37,227,172,1,157,213,172,191,198,
    253,21,144,171,48,92,135,225,6,12,55,179,254,22,12,111,
    65,207,0,191,145,83,221,6,105,194,112,7,58,59,32,241,
    183,13,103,168,19,127,149,120,235,161,102,214,242,137,183,121,
    226,29,232,220,1,137,191,219,122,98,25,218,205,77,84,111,
    240,111,252,107,162,122,65,213,176,121,33,227,36,136,66,55,
    8,123,81,96,208,243,50,53,100,140,46,53,11,169,85,62,
    33,171,252,11,216,36,190,145,90,229,20,144,176,32,113,70,
    6,156,114,231,212,128,105,19,78,4,12,45,240,77,56,193,
    101,74,196,64,95,192,153,1,207,77,154,112,138,173,133,122,
    252,46,88,74,155,100,200,122,212,148,22,224,180,4,39,37,
    104,63,61,49,104,224,168,2,241,63,225,235,93,38,186,200,
    68,13,56,193,214,130,51,11,78,203,240,4,39,225,208,176,
    66,226,139,167,39,40,41,142,180,155,22,114,123,80,16,151,
    68,241,131,56,244,198,82,53,176,239,78,188,216,27,187,237,
    96,252,217,225,80,118,85,179,154,205,138,146,189,137,167,6,
    14,191,102,146,62,198,19,197,228,162,80,170,37,236,244,130,
    208,119,199,145,127,60,146,106,145,104,185,189,96,36,93,151,
    31,126,58,158,68,177,250,121,28,71,177,67,42,229,193,81,
    228,229,111,144,66,187,163,40,145,77,90,141,151,113,136,188,
    162,217,189,9,83,36,6,152,85,122,217,151,73,55,14,38,
    10,45,165,41,210,108,162,214,36,27,113,147,60,199,166,53,
    136,198,178,133,82,181,30,71,47,67,90,50,105,245,229,248,
    193,253,68,121,135,35,121,223,243,122,111,191,243,80,122,158,
    255,208,111,29,30,7,35,191,245,145,243,235,214,100,170,6,
    81,216,26,63,104,5,161,146,168,161,81,235,130,110,246,112,
    202,26,173,242,50,232,187,1,203,231,14,228,104,34,227,58,
    141,222,38,14,68,67,212,68,89,152,162,41,234,216,43,225,
    101,138,93,99,73,28,4,36,97,151,164,38,112,153,25,156,
    254,1,108,56,180,251,145,1,241,46,129,101,136,63,65,214,
    69,200,180,233,153,193,207,190,32,213,232,209,161,73,16,208,
    131,39,12,48,68,26,206,124,68,54,15,129,81,82,130,97,
    25,52,122,16,116,26,78,241,148,90,156,78,100,12,36,110,
    65,242,247,89,10,97,3,80,245,232,187,56,180,133,75,253,
    137,1,217,110,18,227,7,12,12,53,8,18,84,44,171,159,
    250,236,66,109,212,201,231,83,173,167,228,46,14,60,139,142,
    237,174,23,134,145,178,61,223,183,61,165,226,224,240,88,201,
    196,86,145,125,47,105,86,200,222,171,25,182,114,122,211,73,
    134,37,178,59,98,73,223,248,65,87,225,205,58,223,176,254,
    19,169,16,23,131,200,79,112,156,72,244,165,114,136,73,181,
    140,205,71,217,114,12,192,102,57,131,75,34,71,61,85,101,
    228,121,73,226,242,114,52,206,32,163,183,95,120,163,99,169,
    104,62,162,69,225,170,212,213,11,205,19,102,219,36,104,38,
    39,233,206,13,163,208,159,34,155,65,247,30,113,176,205,96,
    171,1,193,109,19,161,182,128,109,25,234,8,189,134,209,37,
    137,172,20,104,12,178,45,146,31,216,240,34,141,28,8,184,
    51,140,47,77,131,3,4,139,198,46,104,83,143,94,118,8,
    207,206,29,106,118,169,249,78,38,253,156,84,80,191,168,130,
    119,105,89,131,229,238,154,169,132,185,27,29,204,184,209,206,
    185,27,97,56,108,147,59,24,228,52,231,238,96,146,14,226,
    253,20,251,228,104,104,126,124,92,64,60,107,198,161,32,201,
    248,97,176,58,132,192,34,12,251,5,24,58,100,20,198,160,
    179,115,149,22,239,222,152,22,251,90,139,15,104,217,90,138,
    158,58,163,166,42,186,100,122,35,213,41,235,243,49,118,166,
    183,72,159,69,77,222,194,253,237,73,88,231,141,138,55,59,
    222,221,117,248,208,10,214,29,139,112,214,51,97,43,221,128,
    18,242,246,73,28,189,154,218,81,207,86,144,241,240,232,94,
    178,119,47,249,0,227,129,189,207,17,70,71,4,237,243,177,
    156,196,232,219,21,190,209,254,234,178,239,186,233,22,130,26,
    167,205,156,13,197,122,230,240,148,168,152,162,210,60,149,92,
    205,149,76,60,127,64,107,86,89,195,38,220,194,171,42,152,
    49,55,226,23,56,163,224,167,120,125,76,186,38,113,37,80,
    122,231,180,53,219,44,17,201,230,252,96,6,47,243,145,199,
    121,11,23,120,156,121,91,25,114,116,208,101,18,199,228,0,
    127,1,78,185,4,252,25,8,9,104,240,212,101,216,57,233,
    34,131,174,211,244,223,3,7,158,75,246,56,67,187,158,145,
    134,38,244,204,228,33,79,213,91,222,175,224,175,133,168,117,
    102,130,160,237,201,76,147,170,226,246,100,229,206,202,16,250,
    175,182,32,107,214,171,201,70,3,47,161,105,218,127,205,220,
    127,207,3,96,158,13,97,80,154,19,186,22,245,106,46,49,
    246,233,57,182,40,246,223,17,235,70,1,49,63,166,230,126,
    14,22,145,141,253,255,121,188,123,49,88,23,246,43,87,7,
    200,95,18,35,22,179,190,82,102,175,206,41,228,46,81,202,
    92,98,144,187,132,228,144,253,154,115,110,106,13,50,254,153,
    33,176,182,193,36,133,202,10,11,100,9,58,229,172,28,90,
    160,178,34,45,99,22,178,50,102,49,45,99,210,10,103,41,
    171,112,106,228,114,148,92,50,198,179,139,35,32,69,206,153,
    93,132,245,121,160,53,157,163,70,3,130,154,87,243,140,56,
    132,137,71,35,111,124,232,123,251,191,161,21,105,217,110,230,
    163,70,38,67,163,40,3,249,151,184,74,12,190,125,59,147,
    229,197,60,163,205,59,184,64,46,3,251,150,31,117,57,196,
    124,57,144,246,88,142,15,177,60,27,4,19,187,55,242,250,
    108,39,51,149,241,179,76,70,197,240,184,184,119,39,20,199,
    14,34,187,27,133,184,41,28,119,85,20,219,190,196,170,69,
    250,246,125,155,119,20,59,72,108,239,16,159,122,88,250,176,
    191,204,122,62,39,135,94,220,79,56,15,60,122,73,221,121,
    219,217,197,186,52,192,236,247,183,185,157,51,156,230,118,174,
    107,29,12,69,150,204,20,141,204,181,210,114,209,223,152,160,
    22,247,54,204,55,31,33,125,210,234,79,103,132,49,82,163,
    254,162,96,80,45,12,130,246,111,151,201,179,62,35,15,49,
    217,198,184,35,207,115,1,231,189,57,75,70,17,45,231,227,
    217,117,108,53,43,27,105,75,203,118,35,6,171,164,6,99,
    22,58,215,17,107,109,70,172,88,246,137,100,114,51,82,145,
    95,101,28,124,117,29,161,54,46,8,133,219,223,13,138,85,
    101,177,50,30,158,95,71,176,213,25,193,144,185,88,29,79,
    110,70,42,157,213,51,3,191,203,68,106,146,4,231,41,50,
    199,55,157,128,96,153,49,145,177,154,234,100,240,125,106,126,
    72,13,149,205,206,79,169,249,25,53,143,168,217,167,230,67,
    106,62,134,27,200,179,91,184,192,151,180,18,185,70,25,179,
    185,10,214,239,149,252,95,173,20,109,240,57,189,159,188,153,
    42,97,106,242,237,169,146,62,0,78,19,166,50,200,5,58,
    69,164,196,168,148,38,70,248,46,13,46,113,91,163,68,188,
    83,207,6,151,185,93,225,193,70,54,184,202,237,26,15,174,
    83,22,85,134,107,102,81,156,126,204,59,241,56,132,255,101,
    242,228,188,123,147,34,56,63,129,180,54,187,42,113,202,246,
    89,150,239,97,30,3,206,184,130,155,110,20,234,120,22,87,
    60,193,114,11,107,253,19,14,19,174,161,203,253,115,35,90,
    80,220,178,66,249,210,189,128,86,93,206,19,50,188,201,68,
    134,126,97,123,94,157,179,166,40,16,248,112,94,61,97,93,
    190,129,215,155,136,181,244,196,76,70,70,102,41,199,232,156,
    99,31,27,56,200,131,222,198,76,208,115,62,161,166,49,27,
    225,222,207,109,66,135,126,190,28,73,37,47,154,69,159,252,
    235,112,233,75,204,126,163,169,235,234,51,10,188,31,97,127,
    251,205,200,163,15,100,251,82,93,253,16,55,30,46,149,213,
    206,37,51,38,211,232,112,200,239,95,253,20,9,240,41,44,
    223,41,251,146,121,242,133,12,213,31,49,41,242,229,43,38,
    246,173,147,136,38,157,53,21,7,231,30,233,233,52,197,131,
    52,168,80,164,199,56,111,81,140,223,196,127,46,55,46,124,
    119,210,204,177,50,249,180,97,154,56,236,226,43,185,121,249,
    251,72,182,215,17,18,56,85,59,64,59,232,143,46,164,24,
    231,123,212,124,63,67,8,123,162,62,133,226,51,30,125,182,
    134,126,203,181,16,151,62,206,30,164,155,255,248,193,94,38,
    209,158,31,123,216,55,120,84,109,94,120,152,200,56,240,70,
    193,215,250,224,62,27,102,59,234,247,104,149,199,212,35,61,
    242,221,249,43,252,237,32,189,163,199,188,39,50,116,11,201,
    7,226,20,147,181,32,65,202,172,73,181,123,137,209,139,179,
    230,109,95,93,143,235,83,201,125,58,78,72,40,199,160,179,
    249,202,10,238,233,6,125,4,50,69,85,212,133,37,106,245,
    138,89,41,87,74,38,98,128,70,214,69,213,172,84,107,56,
    247,71,213,138,168,26,91,181,138,248,15,63,244,196,79,
};

EmbeddedPython embedded_m5_internal_param_SimObject(
    "m5/internal/param_SimObject.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/param_SimObject.py",
    "m5.internal.param_SimObject",
    data_m5_internal_param_SimObject,
    2303,
    7692);

} // anonymous namespace
