#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_IdeDisk_vector[] = {
    120,156,205,92,123,115,27,213,21,63,43,201,178,165,216,177,
    29,191,29,67,196,195,32,104,19,3,37,101,10,153,12,165,
    233,35,76,107,232,154,105,130,67,89,214,218,107,121,109,105,
    87,236,174,157,40,181,91,138,83,160,45,29,166,239,215,159,
    157,254,209,239,209,111,210,15,210,158,223,185,187,178,100,201,
    29,64,182,54,150,117,230,122,31,247,158,223,121,221,123,206,
    222,117,133,226,159,33,254,190,90,34,10,107,6,145,195,191,
    6,213,136,234,6,173,27,100,40,131,156,41,218,25,162,224,
    69,114,134,232,1,209,122,134,84,134,14,185,145,165,59,25,
    242,70,229,158,60,213,178,114,196,160,102,145,84,142,214,135,
    232,150,55,73,57,149,167,157,34,5,239,145,97,24,158,65,
    183,157,97,114,70,232,1,247,206,141,130,116,56,66,78,81,
    26,5,114,206,73,163,72,205,9,82,231,104,157,59,31,166,
    245,49,238,234,89,238,234,188,116,245,111,116,229,240,153,105,
    114,198,112,57,243,242,54,174,204,225,74,25,227,188,244,50,
    78,206,56,122,217,100,12,19,173,11,185,227,44,109,79,210,
    250,36,41,254,157,160,67,134,25,67,184,64,235,83,9,156,
    233,182,246,76,91,123,182,173,61,215,214,158,151,246,100,50,
    224,133,214,128,11,50,224,34,173,47,146,226,223,5,61,96,
    158,214,202,51,44,121,247,191,252,83,102,201,83,52,202,100,
    79,5,161,235,123,150,235,109,250,110,6,231,243,32,208,83,
    5,100,56,86,216,183,160,176,127,145,104,203,201,196,10,59,
    32,238,216,0,67,181,12,29,72,227,32,67,205,50,237,27,
    180,157,35,39,75,251,60,204,16,24,168,26,116,152,161,119,
    178,184,224,128,105,142,69,252,40,229,34,173,173,109,17,177,
    238,105,152,14,134,104,127,136,214,110,239,103,112,96,167,64,
    193,63,233,254,146,116,58,34,157,102,104,159,105,142,14,115,
    116,144,167,91,124,17,31,218,46,0,190,113,123,159,145,242,
    145,181,114,142,185,93,109,131,11,40,142,27,120,118,93,69,
    227,220,182,110,58,234,134,27,238,88,123,170,18,249,65,185,
    152,92,228,135,87,26,118,180,101,202,93,89,136,163,222,136,
    164,55,223,83,209,57,110,108,186,158,99,213,125,103,183,166,
    162,17,116,101,109,186,53,101,89,114,242,102,189,225,7,209,
    183,131,192,15,76,72,84,14,214,124,187,117,7,228,89,169,
    249,161,42,99,52,25,198,68,247,17,174,222,108,72,143,96,
    64,56,197,205,142,10,43,129,219,136,88,81,186,71,92,141,
    222,202,80,145,144,240,14,147,149,45,191,174,86,2,187,190,
    114,195,191,235,97,200,112,165,170,234,87,47,135,145,189,81,
    83,151,109,123,243,185,231,95,82,182,237,188,228,172,108,236,
    186,53,103,229,155,230,15,86,26,205,104,203,247,86,234,87,
    87,92,47,82,44,160,218,74,167,104,174,240,21,23,48,200,
    93,183,106,185,2,207,218,82,181,134,10,198,112,116,17,12,
    24,19,198,168,145,55,178,70,217,24,227,214,16,127,179,198,
    82,230,156,177,234,2,96,5,160,97,90,217,196,152,254,65,
    162,54,214,250,78,134,130,37,152,202,54,255,26,208,45,27,
    204,26,206,101,228,220,15,33,25,125,116,59,11,3,208,7,
    247,197,188,216,206,248,202,107,208,184,71,98,35,67,180,157,
    39,109,59,108,114,218,152,130,38,40,95,142,110,50,220,121,
    142,194,207,58,123,240,38,136,37,207,78,205,135,102,121,168,
    15,197,28,215,202,96,124,85,236,34,218,114,67,150,171,72,
    31,109,113,160,53,150,201,155,205,55,54,182,89,82,225,37,
    62,240,182,191,91,170,216,158,231,71,37,219,113,74,118,20,
    5,238,198,110,164,194,82,228,151,150,195,114,1,234,158,76,
    76,171,213,95,179,145,152,18,212,206,166,164,255,112,220,74,
    196,127,76,201,31,34,255,80,69,108,22,91,190,19,242,113,
    116,81,85,145,9,38,163,243,76,190,153,12,39,246,87,206,
    39,214,18,170,218,102,84,20,195,179,195,208,146,225,112,92,
    108,12,119,239,217,181,93,21,225,122,54,150,136,71,69,83,
    15,52,64,43,155,7,206,4,38,68,103,121,190,231,52,153,
    75,183,178,12,6,230,197,214,70,9,214,54,195,150,54,204,
    52,79,99,108,121,19,153,10,0,229,98,59,19,27,155,5,
    124,18,189,27,113,216,96,123,59,228,224,82,206,72,116,16,
    100,226,128,37,180,112,179,9,115,54,47,130,44,129,60,146,
    128,31,140,4,198,142,75,224,5,140,154,17,216,149,108,12,
    176,229,68,171,29,78,180,112,228,68,28,10,215,224,12,25,
    184,204,145,51,100,33,130,224,122,108,249,112,51,86,62,159,
    110,179,119,17,140,57,1,192,249,196,84,77,216,95,187,17,
    86,219,140,208,132,78,196,2,205,133,147,132,120,41,45,33,
    86,181,16,175,98,212,209,216,118,198,196,102,138,70,5,138,
    207,196,34,21,113,222,224,70,115,14,226,108,23,228,28,79,
    109,183,188,49,153,163,100,158,147,57,95,199,14,45,95,221,
    200,193,202,54,179,52,27,207,61,33,92,189,17,248,247,154,
    37,127,179,20,81,194,195,181,229,240,202,114,248,10,7,131,
    210,117,9,47,58,28,104,135,15,84,35,96,199,46,200,31,
    218,89,45,113,92,43,158,62,88,224,152,199,69,79,34,102,
    137,77,97,20,32,36,13,80,198,197,150,140,193,242,43,24,
    178,40,2,206,210,28,127,139,134,240,101,249,18,25,101,45,
    33,103,249,251,26,68,13,180,138,176,230,51,215,52,215,2,
    8,208,204,167,59,172,101,32,112,204,103,185,255,27,137,171,
    229,169,101,27,248,102,193,48,172,255,99,146,181,150,65,31,
    17,236,128,213,29,251,139,120,38,190,80,231,20,46,127,151,
    36,232,244,152,222,50,218,239,50,113,88,98,183,12,95,146,
    75,245,108,247,58,125,210,22,177,14,179,100,96,102,202,198,
    171,169,246,153,41,215,242,84,49,160,207,53,251,228,58,93,
    26,42,218,178,67,92,166,157,55,219,114,222,163,224,215,90,
    7,113,68,26,140,109,141,232,193,44,240,117,243,200,178,16,
    246,47,26,83,153,54,123,249,42,200,229,150,169,24,201,177,
    51,103,241,210,241,56,221,54,83,89,58,54,126,15,124,228,
    132,243,241,188,204,206,122,177,112,147,59,181,185,155,150,83,
    12,37,78,241,159,150,83,40,9,217,15,100,189,13,154,129,
    254,15,51,6,167,60,188,68,65,182,145,35,53,68,235,121,
    82,195,88,22,35,151,25,138,115,25,94,235,163,113,142,142,
    114,31,201,107,144,254,24,200,89,226,244,231,124,146,254,112,
    226,50,42,141,137,56,195,225,92,37,206,105,46,32,167,65,
    99,42,206,105,214,167,145,116,160,49,19,39,29,235,179,72,
    217,208,152,67,142,132,198,60,57,51,210,88,32,103,86,26,
    139,228,204,73,227,34,252,29,51,134,120,88,242,149,232,139,
    168,221,49,129,137,58,87,181,162,91,54,171,205,17,228,222,
    0,163,29,44,242,90,205,174,111,56,246,245,53,12,136,81,
    43,73,128,200,36,16,38,218,33,192,185,141,147,80,200,159,
    207,37,80,246,6,24,233,158,135,33,38,16,196,175,29,191,
    34,225,237,173,45,85,170,171,250,6,231,132,91,110,163,180,
    89,179,171,162,165,108,12,241,141,4,98,36,118,121,124,209,
    16,34,134,174,250,165,138,239,241,116,180,139,241,74,142,226,
    92,73,57,165,203,37,153,203,74,110,88,178,55,248,172,93,
    137,180,179,118,70,29,89,147,218,65,53,148,229,231,206,93,
    52,7,172,101,139,115,97,151,215,220,111,181,180,220,101,168,
    96,210,57,178,79,193,49,212,10,63,139,52,208,137,11,234,
    188,213,197,107,203,34,199,180,186,182,141,100,193,215,110,142,
    146,76,78,119,5,38,75,146,129,84,208,96,194,185,157,160,
    209,153,99,107,22,110,119,178,22,36,61,131,118,163,154,234,
    70,229,122,149,160,109,217,132,59,188,1,26,23,172,6,44,
    188,221,21,62,250,70,230,168,118,100,230,55,6,170,51,1,
    6,14,214,59,172,48,1,246,221,94,192,62,237,101,136,115,
    61,128,113,150,109,123,21,213,6,238,197,193,130,67,72,72,
    184,184,211,39,192,30,158,166,222,223,181,107,233,161,67,212,
    18,22,222,233,39,130,244,176,200,138,223,104,166,17,64,196,
    24,49,248,143,79,25,144,167,238,69,169,1,194,224,239,246,
    3,168,135,107,89,2,201,178,82,1,21,87,186,132,1,235,
    148,129,53,2,181,231,250,187,97,106,192,18,6,222,235,51,
    92,204,118,99,179,157,189,116,195,33,18,198,152,9,187,79,
    120,51,189,108,82,189,207,22,153,26,186,188,152,37,120,216,
    56,11,112,158,74,31,28,120,168,244,9,174,103,52,113,109,
    199,73,19,94,188,118,23,46,156,51,1,24,238,110,60,4,
    0,133,11,117,250,161,197,178,82,86,160,148,11,53,19,155,
    103,1,47,101,245,105,120,194,68,149,142,207,120,29,169,229,
    254,81,106,41,236,165,55,65,187,124,222,178,182,18,118,203,
    82,19,108,149,104,165,164,165,107,96,141,192,111,168,32,106,
    234,106,228,215,65,158,1,89,238,8,136,142,170,169,72,89,
    157,154,137,38,168,245,88,193,81,97,20,248,77,203,138,165,
    197,55,88,150,228,132,230,203,32,215,64,174,131,188,10,242,
    26,8,138,182,230,119,64,80,111,51,95,7,249,62,8,158,
    141,152,111,130,152,32,200,231,205,31,129,220,234,16,229,96,
    242,218,21,12,140,129,80,156,204,27,23,141,130,145,231,47,
    62,163,252,41,156,248,17,33,235,126,146,7,214,221,117,67,
    199,248,28,117,67,189,73,34,174,30,230,147,114,225,112,82,
    46,28,65,137,48,222,21,49,146,84,18,11,73,37,81,87,
    12,71,147,138,225,88,82,49,60,159,84,12,199,147,138,225,
    68,82,49,156,76,42,134,23,146,138,225,84,82,49,156,78,
    42,134,51,73,197,112,54,169,24,206,37,21,195,249,164,98,
    184,64,206,124,82,67,92,136,107,136,206,162,52,150,200,185,
    40,141,71,200,89,146,198,163,228,60,34,141,75,228,60,42,
    141,18,57,151,164,241,24,57,37,105,60,78,206,99,210,120,
    130,156,199,165,241,36,57,79,72,99,153,156,39,165,241,20,
    169,167,105,187,76,235,207,144,179,44,71,158,69,225,242,169,
    158,78,251,5,10,151,82,242,27,112,117,40,108,133,155,211,
    168,87,154,47,164,136,192,252,26,197,143,98,78,170,85,126,
    137,252,161,211,187,36,232,33,48,165,22,119,19,6,118,233,
    196,105,162,216,210,219,225,177,58,228,187,148,206,140,33,81,
    117,175,139,227,47,160,138,197,110,85,88,120,168,114,95,5,
    126,74,105,42,54,223,180,241,112,183,31,120,61,44,141,211,
    12,223,175,165,154,130,107,6,238,245,3,108,182,23,176,154,
    242,82,194,165,87,14,50,126,179,31,88,23,186,97,53,252,
    70,42,144,176,248,229,177,239,119,192,249,226,171,226,139,189,
    20,85,85,81,88,115,43,169,102,166,163,162,178,35,78,126,
    114,22,64,195,135,6,232,17,39,251,103,1,148,23,205,15,
    9,208,35,78,14,250,4,218,115,110,224,238,121,170,172,167,
    137,243,92,130,83,51,242,211,179,128,201,126,241,112,192,108,
    49,242,179,179,128,25,62,44,48,91,140,124,208,39,204,153,
    110,152,118,163,161,60,39,221,26,160,230,225,231,212,199,220,
    56,221,141,76,213,27,81,58,79,124,228,25,22,70,255,176,
    31,72,83,221,144,66,247,126,42,15,193,245,134,93,30,252,
    240,148,117,84,169,41,59,157,180,70,111,121,231,209,31,244,
    233,83,189,212,116,215,110,164,231,81,162,43,230,224,23,253,
    232,106,169,27,85,21,219,223,106,53,191,146,90,42,10,182,
    59,184,248,232,148,205,113,67,85,93,47,53,115,148,209,63,
    238,7,82,143,12,1,193,61,173,12,129,199,254,164,31,56,
    61,166,171,32,61,21,97,166,210,195,255,178,31,80,61,226,
    69,144,150,146,16,42,48,248,175,250,1,212,163,140,192,169,
    161,181,97,87,118,210,123,224,29,51,240,107,234,47,184,247,
    90,86,4,118,152,226,227,110,89,91,128,133,79,59,160,101,
    219,161,189,124,4,141,113,233,87,224,166,219,94,14,144,253,
    222,198,45,111,2,47,16,236,203,182,111,43,163,223,33,56,
    170,221,229,58,162,138,167,238,90,157,130,208,149,88,188,67,
    208,182,69,27,194,144,19,3,44,243,225,217,206,111,136,146,
    45,217,227,70,150,166,249,251,37,53,62,223,195,154,119,195,
    45,109,206,169,105,93,222,219,72,216,248,109,63,222,218,195,
    162,55,3,223,75,103,39,17,140,89,70,255,221,41,71,212,
    212,130,15,188,6,131,255,190,207,192,211,43,83,11,67,183,
    234,165,156,169,9,15,127,56,125,112,129,146,220,38,85,112,
    154,135,63,158,62,56,215,11,85,16,165,11,78,243,240,167,
    62,193,245,168,172,179,212,84,176,151,242,6,176,152,137,63,
    247,19,70,122,172,99,42,118,195,174,184,41,85,17,176,142,
    73,24,248,75,23,176,142,253,25,15,213,214,255,191,37,188,
    150,191,66,237,155,51,76,236,162,215,219,49,142,118,98,224,
    225,160,60,168,51,223,3,217,0,193,94,41,19,27,110,76,
    236,243,48,183,65,106,32,30,72,3,36,160,120,237,97,226,
    1,159,137,167,69,38,30,70,152,168,95,155,40,122,154,31,
    128,160,0,99,34,195,55,145,42,154,200,70,76,44,118,245,
    134,16,172,26,204,207,64,48,165,154,136,217,38,220,223,132,
    29,153,127,5,249,123,135,95,199,219,69,142,45,132,176,203,
    220,188,211,33,232,193,72,27,91,164,33,137,16,91,76,176,
    133,35,255,127,182,109,156,240,201,117,31,147,245,220,177,127,
    189,160,81,161,186,175,223,187,107,134,216,59,66,230,120,75,
    62,122,159,105,188,233,6,170,150,69,203,170,93,215,47,126,
    203,235,204,230,99,32,79,36,38,32,129,73,191,142,41,111,
    59,234,119,76,121,169,41,111,230,200,139,56,230,21,16,236,
    85,145,71,11,199,182,80,97,139,78,192,25,89,200,7,34,
    92,94,191,122,37,145,215,149,134,205,82,79,148,36,255,201,
    160,126,85,2,88,251,69,202,219,149,107,110,222,144,254,187,
    239,199,205,55,235,118,85,157,112,126,205,173,235,87,237,163,
    201,99,231,157,192,230,246,204,177,163,28,167,92,187,198,243,
    140,104,175,215,243,146,118,80,3,182,40,253,82,153,126,171,
    247,58,106,207,225,107,76,240,102,123,97,188,96,228,51,248,
    15,10,89,163,104,140,25,57,99,116,172,144,45,228,11,67,
    89,182,58,28,153,50,138,217,66,113,118,174,96,20,51,163,
    198,209,103,246,153,130,241,63,194,213,239,169,
};

EmbeddedPython embedded_m5_internal_IdeDisk_vector(
    "m5/internal/IdeDisk_vector.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/m5/internal/IdeDisk_vector.py",
    "m5.internal.IdeDisk_vector",
    data_m5_internal_IdeDisk_vector,
    3308,
    17770);

} // anonymous namespace