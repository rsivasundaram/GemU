#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_enum_MemSched[] = {
    120,156,197,86,93,111,27,69,20,189,179,187,118,98,39,110,
    156,134,126,209,64,141,80,132,169,104,12,136,170,15,173,42,
    10,69,162,72,13,176,126,104,107,16,203,102,119,28,175,99,
    239,90,187,147,22,163,228,133,84,192,27,63,2,241,192,255,
    224,55,241,10,247,220,89,59,78,85,36,94,8,78,118,116,
    119,118,230,126,156,123,238,157,137,168,252,85,248,249,176,69,
    84,252,201,66,204,255,138,70,68,99,69,61,69,74,43,138,
    215,104,191,66,249,7,20,87,232,57,81,207,33,237,208,49,
    11,46,125,229,80,186,42,123,170,52,114,101,70,209,180,78,
    218,163,94,133,30,165,235,228,233,42,237,215,41,255,150,148,
    82,169,162,199,241,18,197,203,244,156,181,179,80,19,133,203,
    20,215,69,168,81,188,34,66,157,166,77,210,43,212,99,229,
    75,212,107,176,170,235,172,234,156,168,250,3,170,98,254,210,
    164,184,129,229,236,203,19,172,244,176,82,108,156,19,45,107,
    164,93,26,54,169,215,20,97,157,122,235,34,156,167,222,121,
    17,54,168,183,1,215,187,237,117,14,61,249,139,127,109,197,
    146,89,229,225,169,206,139,36,75,131,36,237,103,137,131,239,
    85,12,64,44,194,176,84,66,247,49,160,251,157,4,183,216,
    41,161,59,34,82,120,39,26,57,116,36,194,145,67,211,54,
    29,42,26,122,20,187,116,200,102,42,116,172,104,79,209,177,
    67,95,187,88,112,196,163,199,193,190,78,158,177,184,13,37,
    88,171,105,137,142,42,116,88,161,238,227,67,7,19,251,53,
    202,127,163,239,55,69,233,178,40,117,232,144,71,143,142,61,
    58,170,210,35,94,196,83,195,26,32,82,143,15,57,82,158,
    233,182,61,246,118,103,33,92,132,18,39,121,26,142,181,57,
    199,114,160,211,131,113,240,80,143,187,209,64,199,237,250,108,
    77,86,108,79,66,51,240,101,147,11,52,198,19,35,202,178,
    84,155,21,22,250,73,26,7,227,44,62,24,105,179,12,77,
    65,63,25,233,32,144,143,15,198,147,44,55,159,228,121,150,
    251,0,84,38,71,89,56,223,1,56,163,81,86,232,54,172,
    137,25,31,234,13,86,247,39,162,17,14,136,163,216,28,235,
    34,202,147,137,225,60,89,141,88,13,109,109,100,72,134,162,
    199,67,103,144,141,117,39,15,199,157,251,217,179,20,38,139,
    206,158,30,223,188,81,152,112,119,164,111,132,97,255,221,247,
    110,233,48,140,111,197,157,221,131,100,20,119,238,249,15,59,
    147,169,25,100,105,103,124,179,147,164,70,51,62,163,206,41,
    100,182,121,193,121,216,120,150,236,5,137,68,23,12,244,104,
    162,243,6,102,95,133,125,213,84,171,170,170,92,213,86,13,
    150,42,252,184,106,211,89,81,59,9,226,139,16,51,136,229,
    206,168,244,43,73,210,56,231,251,14,229,155,32,202,144,255,
    21,50,203,116,233,226,155,35,223,190,4,48,118,118,232,34,
    253,118,242,80,200,197,44,227,149,119,144,239,148,132,33,21,
    26,86,201,50,135,9,103,169,148,79,49,242,114,168,113,88,
    185,71,197,47,167,53,164,77,98,224,185,184,120,234,34,155,
    250,65,200,216,109,195,241,29,161,133,25,36,5,195,42,224,
    67,150,242,233,50,38,95,76,63,223,29,234,200,20,215,120,
    226,73,118,208,138,194,52,205,76,43,140,227,86,104,76,158,
    236,30,24,93,180,76,214,218,42,218,53,100,123,125,198,172,
    185,190,233,100,198,36,100,157,153,100,95,226,36,50,252,178,
    33,47,130,127,161,13,179,98,144,197,5,207,67,197,158,54,
    62,156,20,74,223,155,153,19,250,181,171,51,178,20,122,212,
    55,117,225,93,88,20,129,152,195,188,80,12,187,159,134,163,
    3,109,176,158,185,98,216,42,68,107,232,236,72,118,25,97,
    206,162,4,114,65,154,165,241,148,157,76,162,45,216,191,44,
    84,91,37,144,237,2,19,109,137,199,42,53,152,120,77,39,
    66,60,94,73,51,161,216,69,68,79,146,118,85,246,12,166,
    219,49,119,150,182,35,173,65,2,147,242,107,65,194,102,31,
    108,246,175,98,216,196,240,218,44,246,51,1,160,241,34,0,
    239,195,168,35,81,71,110,25,223,188,132,118,78,149,208,149,
    147,18,226,54,216,69,41,56,40,152,147,82,112,129,64,126,
    183,228,61,138,140,83,207,159,23,216,46,184,248,77,196,91,
    157,17,213,7,251,22,41,184,183,64,65,31,41,17,254,249,
    87,254,9,195,107,255,19,134,123,22,195,155,48,186,90,50,
    167,33,140,169,171,8,105,119,74,68,5,205,251,44,76,47,
    1,205,69,28,47,241,169,246,40,109,200,241,36,71,156,28,
    188,182,113,88,120,173,224,129,99,125,151,46,150,199,78,129,
    58,159,228,217,119,211,86,214,111,25,154,249,112,103,171,216,
    222,42,110,115,39,104,221,149,222,98,123,129,173,246,92,79,
    114,174,234,154,188,216,74,13,164,106,131,242,232,96,188,47,
    0,71,103,134,178,52,166,194,228,232,71,103,7,113,125,14,
    49,60,190,13,139,117,193,215,165,75,252,212,149,184,21,100,
    210,21,229,22,33,95,249,249,8,72,35,88,77,184,119,249,
    93,235,180,196,131,200,252,183,78,113,229,44,162,241,175,179,
    250,251,179,58,171,210,156,25,120,92,248,11,234,255,196,39,
    179,2,57,126,36,176,128,147,93,22,139,148,37,30,36,115,
    3,203,191,33,105,56,47,57,217,28,91,116,78,217,146,184,
    38,139,91,178,212,30,116,159,209,207,11,221,234,216,37,133,
    67,201,45,175,81,139,135,146,55,47,83,161,207,191,58,120,
    188,211,245,140,12,13,194,2,203,108,229,186,243,202,61,105,
    124,243,27,16,183,163,51,97,214,178,181,21,192,173,7,39,
    188,66,199,191,170,54,156,5,182,188,131,225,198,156,40,106,
    54,247,95,123,120,237,197,22,189,112,70,5,182,45,126,10,
    55,60,113,124,173,42,184,190,112,177,182,46,191,50,7,119,
    90,248,152,241,215,48,56,179,126,192,221,131,175,88,102,106,
    239,170,98,114,62,133,22,177,195,103,183,189,101,182,176,239,
    13,12,111,98,120,27,26,96,204,22,160,16,220,54,149,84,
    63,147,182,34,25,247,183,169,188,29,244,163,126,33,27,250,
    185,136,104,84,59,11,113,203,162,232,105,152,159,109,101,10,
    10,119,108,231,187,139,123,112,129,227,16,103,127,109,173,166,
    170,14,174,152,174,170,243,61,192,83,171,141,154,91,171,214,
    42,46,223,5,48,179,161,234,110,173,94,83,252,231,252,13,
    67,73,3,159,
};

EmbeddedPython embedded_m5_internal_enum_MemSched(
    "m5/internal/enum_MemSched.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/enum_MemSched.py",
    "m5.internal.enum_MemSched",
    data_m5_internal_enum_MemSched,
    1460,
    3587);

} // anonymous namespace