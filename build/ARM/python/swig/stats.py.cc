#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_stats[] = {
    120,156,237,93,105,144,220,198,117,110,204,236,193,189,184,247,
    46,201,93,146,203,123,72,138,92,146,18,69,29,20,117,81,
    7,117,144,20,86,177,164,177,236,17,118,128,221,157,229,92,
    2,48,75,174,76,58,142,169,196,74,34,199,71,236,248,118,
    92,190,18,59,78,202,169,28,118,46,231,114,226,196,118,149,
    143,36,174,84,156,252,112,142,170,56,229,164,156,84,82,101,
    167,156,163,191,7,52,6,24,96,41,82,192,244,236,154,18,
    103,91,61,15,141,238,247,186,95,191,247,250,189,55,64,158,
    185,255,181,243,191,187,166,24,179,14,241,154,206,63,10,43,
    50,86,82,88,86,97,138,161,48,253,110,118,174,157,153,55,
    49,189,157,61,207,88,54,197,140,20,187,204,43,105,246,234,
    20,43,247,210,61,29,172,152,38,136,194,150,187,153,209,198,
    178,237,236,137,242,32,107,51,58,216,185,110,102,62,195,20,
    69,41,43,236,73,189,147,233,235,216,243,188,119,94,233,162,
    14,215,49,189,155,42,93,76,239,161,74,55,91,30,96,70,
    15,203,242,206,59,89,182,143,119,181,143,119,181,158,186,250,
    60,186,210,249,149,123,152,222,135,230,28,151,167,208,178,13,
    45,105,140,245,212,75,63,211,251,209,203,28,167,97,192,107,
    200,59,78,179,197,65,150,29,100,6,255,12,176,203,156,76,
    125,80,52,28,242,26,14,81,195,97,150,29,102,6,255,12,
    81,67,128,70,88,118,132,25,35,108,113,148,101,71,169,50,
    198,178,99,84,25,103,217,113,170,108,96,217,13,84,217,200,
    178,27,169,178,137,101,55,81,101,130,101,39,168,50,201,178,
    147,84,217,204,178,155,169,178,133,101,183,80,101,43,203,110,
    101,250,48,145,49,69,120,76,161,162,143,0,15,160,57,234,
    161,185,141,46,111,103,217,237,204,224,159,109,14,61,99,162,
    225,184,215,112,7,53,220,201,178,59,153,193,63,59,60,122,
    118,177,236,46,186,182,155,101,119,83,101,15,203,238,97,250,
    6,49,39,27,189,62,50,116,121,47,203,238,101,6,255,100,
    156,193,54,137,193,38,188,134,251,168,225,126,150,221,207,12,
    254,217,231,52,156,20,13,55,123,13,111,160,134,7,88,246,
    0,51,248,231,6,167,225,22,209,112,171,215,240,32,53,156,
    102,217,105,102,240,207,65,167,225,20,8,65,195,109,94,195,
    67,212,240,48,203,30,102,6,255,28,114,26,110,23,196,236,
    240,26,30,161,134,55,178,236,141,204,224,159,35,78,195,157,
    98,232,93,94,195,155,168,225,81,150,61,202,12,254,185,201,
    105,184,91,244,184,199,107,120,51,53,60,198,178,199,152,193,
    63,55,59,13,51,162,225,94,175,225,45,212,240,86,150,189,
    149,25,252,115,139,211,112,159,104,184,223,107,120,27,53,188,
    157,101,111,103,6,255,220,230,52,188,65,52,60,224,53,60,
    78,13,239,96,217,59,152,193,63,199,157,134,7,69,195,105,
    175,225,9,106,120,39,203,222,201,12,254,57,225,52,60,68,
    172,118,23,93,187,11,21,253,48,65,238,38,200,221,168,232,
    71,8,114,15,65,238,65,69,191,145,32,247,18,228,94,84,
    244,155,8,114,146,32,39,81,209,143,18,228,62,130,220,135,
    138,126,51,65,238,39,200,253,168,232,199,8,242,0,65,30,
    64,69,191,133,32,15,18,228,65,84,244,91,9,114,138,32,
    167,80,225,251,93,191,141,36,201,67,4,124,8,21,253,118,
    106,246,48,65,30,70,69,63,78,144,71,8,242,8,42,250,
    29,4,121,148,32,143,162,162,159,32,200,105,130,156,70,69,
    191,147,32,103,8,114,6,21,253,46,130,156,37,200,89,84,
    184,208,155,201,220,203,165,103,225,255,248,127,25,133,215,236,
    94,94,44,25,166,85,168,148,115,133,242,92,165,144,194,245,
    14,20,144,181,121,20,157,174,208,189,23,66,247,19,140,36,
    174,158,114,133,238,37,198,87,67,129,76,45,166,216,37,170,
    92,74,177,229,12,187,168,176,69,78,113,154,93,228,195,180,
    99,213,230,21,118,57,197,158,78,163,193,37,94,182,113,49,
    185,133,181,217,142,196,93,36,49,233,244,212,201,46,181,179,
    139,237,108,230,201,139,41,0,206,117,49,243,99,236,185,73,
    234,116,29,117,154,98,23,121,217,198,46,183,177,75,29,236,
    9,222,136,131,22,187,192,51,202,147,23,57,165,28,50,147,
    105,227,216,158,246,145,11,82,244,130,89,214,74,134,13,34,
    115,150,173,217,86,166,91,92,171,88,7,171,154,189,160,82,
    227,52,102,161,84,181,169,147,74,217,176,123,120,101,174,80,
    214,115,165,138,94,43,26,246,58,244,144,155,43,20,141,92,
    142,46,158,42,85,43,166,125,159,105,86,76,21,19,73,192,
    98,69,243,238,192,52,230,139,21,203,200,96,52,26,70,69,
    247,54,90,207,85,169,71,32,64,8,226,102,221,176,242,102,
    161,106,243,245,113,122,68,107,244,150,193,202,80,97,61,204,
    139,233,133,74,201,152,54,181,210,244,201,202,249,50,134,180,
    166,231,141,210,209,3,156,192,217,162,113,64,211,230,14,29,
    62,102,104,154,126,76,159,158,173,21,138,250,244,221,234,163,
    211,213,101,123,161,82,158,182,206,23,230,167,105,42,14,114,
    200,16,58,229,144,92,129,200,201,45,24,197,170,97,246,1,
    186,9,3,42,3,74,175,210,161,164,149,140,210,199,107,237,
    252,47,173,76,166,122,148,211,5,16,148,7,145,152,220,180,
    224,153,143,48,90,29,190,184,231,82,204,156,4,71,44,242,
    143,130,37,228,124,49,131,107,41,186,246,24,102,194,129,46,
    166,177,206,14,240,34,113,17,103,39,222,242,56,22,182,204,
    136,21,218,217,98,7,115,88,132,115,150,195,51,230,50,74,
    222,28,221,164,120,231,109,204,122,75,176,135,242,0,227,51,
    205,247,35,7,141,241,161,222,72,92,55,147,1,226,167,137,
    15,236,133,130,197,231,145,102,27,117,218,39,51,124,78,206,
    46,159,153,93,52,242,182,181,149,3,158,170,212,166,242,90,
    185,92,177,167,52,93,159,210,108,219,44,204,214,108,195,154,
    178,43,83,187,172,76,23,150,119,80,176,146,215,223,114,85,
    176,14,150,153,179,142,243,69,47,228,109,254,101,152,190,208,
    252,91,134,205,217,96,161,162,91,28,142,46,230,13,91,5,
    146,246,122,94,220,45,134,35,126,203,116,8,238,176,140,226,
    156,221,77,140,166,89,86,142,134,3,156,120,10,119,47,105,
    197,154,195,254,88,114,62,42,170,206,64,77,228,170,13,180,
    221,92,178,48,85,185,114,165,172,47,115,172,10,249,93,24,
    112,3,241,86,47,3,119,141,114,206,234,228,101,7,235,227,
    156,54,144,202,131,128,54,151,175,136,167,198,64,46,163,117,
    86,92,105,192,249,235,50,151,25,153,20,109,122,162,132,54,
    216,20,106,184,89,5,251,170,19,40,38,81,108,22,196,54,
    135,226,190,70,138,143,96,148,20,145,153,79,187,4,121,155,
    228,116,96,147,108,172,111,18,46,209,102,192,236,41,108,137,
    58,179,167,65,178,121,194,229,108,108,35,190,184,252,178,143,
    159,105,34,212,1,16,216,33,88,81,5,127,249,153,108,222,
    199,100,42,214,128,56,76,221,184,210,164,109,149,53,105,243,
    206,164,29,197,40,189,46,111,244,17,79,116,43,121,44,108,
    202,157,66,154,190,147,188,178,60,142,233,243,79,220,56,215,
    72,79,148,251,72,181,144,122,34,115,219,145,5,206,124,58,
    149,54,112,209,92,154,141,185,42,195,194,214,173,154,149,11,
    203,83,149,185,41,155,9,28,142,239,178,14,238,178,110,231,
    155,123,234,4,137,11,103,123,59,27,216,52,170,38,223,168,
    93,244,197,217,124,57,218,136,57,87,252,243,9,30,197,196,
    165,196,180,146,172,177,108,19,34,166,137,115,218,237,205,41,
    80,188,29,67,116,211,132,166,217,56,255,235,86,8,143,92,
    133,36,27,169,124,186,202,255,238,193,212,130,58,131,225,120,
    165,206,56,88,18,1,32,69,221,19,224,134,166,160,175,238,
    227,253,157,20,91,167,131,121,107,143,191,52,16,4,55,191,
    137,145,221,168,176,159,96,88,103,190,156,46,255,211,78,195,
    31,150,107,24,205,95,203,72,104,68,168,163,148,179,143,82,
    174,88,225,219,204,58,70,77,29,237,244,16,123,193,39,113,
    46,167,153,2,77,146,118,141,28,191,38,105,243,118,30,49,
    200,85,105,139,182,224,22,197,146,44,104,22,154,57,155,49,
    237,109,198,186,240,242,236,20,46,97,154,195,59,235,156,206,
    115,192,227,84,157,115,32,166,39,148,225,148,143,31,110,64,
    113,192,99,5,69,192,18,71,105,107,163,92,245,105,146,156,
    35,203,30,196,184,109,132,105,127,7,105,75,71,121,159,178,
    13,83,179,43,166,199,228,237,130,201,255,193,99,114,131,68,
    236,243,100,230,162,76,97,125,47,167,148,108,26,38,3,108,
    234,54,102,180,179,108,7,51,58,97,141,194,13,208,238,186,
    1,184,137,141,74,15,171,187,13,200,37,0,207,129,130,227,
    190,235,57,88,47,60,7,252,204,223,75,149,1,215,57,192,
    143,249,174,59,96,8,238,0,84,134,93,119,0,63,197,243,
    115,63,42,163,238,185,159,159,226,221,67,247,56,206,218,168,
    108,112,207,218,252,20,207,15,213,168,108,114,15,213,252,20,
    207,247,47,36,60,237,32,241,71,210,19,82,55,160,112,104,
    249,78,59,11,235,241,164,195,110,40,46,52,81,90,129,227,
    142,23,181,210,172,174,157,152,193,0,24,37,47,54,124,74,
    160,60,224,71,25,155,85,89,9,107,250,122,72,160,190,212,
    68,73,117,24,140,38,80,166,125,169,87,242,36,158,30,95,
    48,166,74,70,105,150,31,181,22,10,213,169,185,162,54,79,
    171,144,118,73,58,35,72,178,137,239,26,149,184,5,25,120,
    186,50,149,175,148,185,186,168,229,57,11,79,233,6,63,139,
    24,250,212,129,41,210,53,83,5,107,74,155,229,87,181,188,
    237,108,190,160,212,32,27,80,51,231,45,50,247,206,157,71,
    181,201,171,152,227,71,202,2,183,105,31,247,86,49,196,120,
    64,74,175,243,27,225,221,238,137,143,77,172,169,138,5,203,
    245,68,8,55,143,195,250,156,229,88,84,132,129,229,103,47,
    58,140,141,132,4,75,142,140,107,41,216,67,1,60,41,176,
    119,78,94,158,86,244,111,18,143,4,71,163,133,169,24,14,
    83,81,40,231,77,159,153,130,59,202,77,100,22,112,1,134,
    124,42,180,221,99,83,162,27,126,74,212,91,155,186,38,68,
    8,70,204,6,184,74,16,242,64,20,33,47,70,49,214,120,
    4,33,252,20,170,149,243,134,143,152,155,154,75,12,182,176,
    24,245,213,49,9,138,216,41,198,179,53,173,40,143,26,72,
    21,26,242,233,56,59,62,130,195,242,149,234,178,140,13,79,
    204,133,193,94,147,48,1,101,227,130,45,141,0,12,246,218,
    56,4,68,108,141,28,145,144,203,73,33,194,245,220,208,128,
    185,132,9,169,154,198,82,161,82,179,164,17,34,6,124,38,
    230,246,30,11,211,162,233,75,114,197,21,14,76,238,160,90,
    76,114,70,163,120,204,120,150,115,152,52,106,200,59,77,99,
    206,54,131,152,178,33,159,24,140,153,143,73,76,228,238,47,
    240,243,180,76,114,92,219,150,70,213,155,66,144,85,155,109,
    1,65,52,170,145,188,40,200,229,36,47,16,185,179,156,65,
    231,154,65,142,228,229,113,200,161,65,231,89,163,198,9,28,
    165,46,214,143,82,132,142,60,133,88,224,179,147,203,45,8,
    244,50,228,163,242,92,132,228,130,113,124,54,85,179,82,53,
    76,123,217,241,142,221,140,98,47,138,93,1,129,165,27,69,
    195,54,114,193,153,183,7,152,231,182,214,13,126,212,173,44,
    231,114,238,236,240,27,114,57,58,19,169,183,161,56,142,226,
    4,138,187,80,220,131,2,78,68,245,126,20,240,15,169,15,
    161,120,4,5,124,239,234,89,20,42,10,156,87,213,87,161,
    120,34,48,117,205,57,199,77,99,32,116,12,103,89,135,50,
    161,116,41,29,252,15,255,122,249,191,174,21,255,145,61,117,
    170,60,87,9,123,179,52,229,101,120,179,82,174,67,11,101,
    23,14,88,217,110,1,236,161,178,151,128,125,2,184,158,202,
    126,2,14,8,224,32,149,67,4,28,22,192,17,42,71,9,
    56,38,128,227,84,110,32,224,70,1,220,68,229,4,1,39,
    5,112,51,149,91,8,184,149,234,83,44,187,77,120,221,182,
    11,175,219,14,120,218,80,217,73,190,55,134,52,17,55,101,
    103,183,112,188,237,17,142,183,140,112,188,237,21,142,183,125,
    194,241,182,31,105,29,24,229,0,12,55,36,114,56,104,76,
    83,121,136,208,56,12,95,90,127,228,62,188,6,95,26,121,
    165,154,236,224,40,121,18,35,9,23,154,122,68,34,198,234,
    141,204,245,238,183,222,125,70,50,94,61,218,92,234,33,13,
    203,236,138,18,190,165,206,178,101,22,79,153,246,186,2,11,
    142,251,211,90,73,242,225,192,29,244,185,152,68,12,250,136,
    152,49,170,26,169,38,121,148,244,58,148,120,35,191,46,196,
    47,215,112,6,237,22,180,228,23,140,252,57,41,230,2,165,
    192,96,180,139,113,16,95,47,16,159,213,44,227,94,105,200,
    35,182,235,141,120,41,14,1,61,130,0,163,12,44,164,96,
    143,99,152,51,220,235,227,160,238,237,226,170,9,38,148,131,
    59,54,176,59,222,143,38,194,241,166,129,152,169,44,142,167,
    209,222,16,7,241,46,129,248,115,134,89,145,230,174,195,96,
    63,198,226,9,76,111,202,151,10,86,193,150,235,241,165,33,
    223,40,8,200,192,122,169,159,74,84,132,79,86,62,135,244,
    9,188,41,77,107,222,176,27,32,124,69,41,20,79,169,77,
    174,70,112,133,242,140,109,22,202,148,194,178,242,69,126,187,
    13,11,178,1,94,31,4,185,127,193,97,9,130,251,156,56,
    131,149,167,92,61,177,23,243,5,74,227,196,29,17,96,220,
    230,100,181,184,16,26,92,52,50,141,103,233,198,70,24,238,
    234,112,238,226,95,235,186,175,160,231,242,149,90,217,166,155,
    194,80,220,134,115,161,0,212,197,29,135,224,22,255,119,52,
    78,81,99,58,109,59,147,197,231,67,155,55,206,242,153,41,
    89,116,199,10,151,44,119,126,2,80,55,117,146,142,144,184,
    71,69,60,70,69,12,67,133,239,89,125,6,5,252,106,42,
    220,55,42,124,4,42,142,174,234,34,138,34,138,178,144,116,
    78,142,156,147,86,80,223,132,69,195,178,104,21,168,226,105,
    35,24,166,14,186,13,32,160,9,126,164,111,205,62,81,34,
    194,1,34,44,88,236,226,68,57,218,240,47,250,76,217,175,
    208,97,82,108,244,7,66,178,233,69,22,222,225,234,11,76,
    24,161,205,221,210,52,208,59,24,19,89,116,36,90,102,242,
    90,81,51,163,207,193,112,208,95,235,57,216,61,86,118,138,
    99,229,58,113,172,236,98,70,55,142,194,72,233,112,206,151,
    125,56,5,118,177,53,112,10,124,143,16,129,107,230,20,232,
    97,124,93,158,2,223,203,98,152,10,3,129,93,33,57,37,
    225,253,113,48,31,12,98,206,173,166,90,81,142,145,6,37,
    231,12,247,129,4,167,222,174,216,90,81,154,145,73,163,125,
    48,132,254,234,241,31,124,72,224,150,129,122,188,26,67,140,
    28,186,111,69,241,246,0,147,8,255,176,55,213,117,21,223,
    108,237,250,102,222,223,187,153,43,105,130,254,218,46,71,35,
    189,202,128,108,137,214,72,239,99,205,244,204,186,186,107,189,
    208,93,253,66,119,13,8,149,53,40,92,162,67,116,227,48,
    101,21,118,187,89,133,92,155,245,68,114,206,42,211,102,31,
    99,107,77,155,121,24,95,151,218,236,227,33,153,116,141,58,
    161,190,163,36,250,75,232,92,240,201,56,168,247,7,81,183,
    10,207,201,113,150,208,207,128,248,96,191,20,7,249,129,32,
    242,114,173,136,79,37,200,49,242,172,8,210,84,191,156,224,
    164,75,179,31,72,189,254,74,8,243,213,99,58,124,90,224,
    150,129,35,224,170,125,56,227,13,59,176,54,11,215,141,115,
    68,95,233,154,112,94,8,64,68,67,248,94,86,234,196,185,
    230,235,132,0,142,147,225,93,44,100,211,248,205,153,122,71,
    242,204,25,28,167,63,138,142,225,168,242,59,11,2,161,102,
    144,114,178,96,217,39,53,91,11,27,53,63,114,53,225,102,
    231,33,11,48,91,58,216,98,39,149,235,200,108,233,18,192,
    110,42,123,8,216,43,128,125,84,174,39,96,191,0,14,80,
    57,72,192,33,1,28,166,114,132,128,163,2,56,70,229,56,
    1,55,8,224,70,42,55,17,112,66,0,39,169,220,76,192,
    45,2,184,149,202,41,2,110,19,192,237,84,238,32,224,78,
    1,220,69,229,110,2,238,17,192,12,149,123,9,184,79,0,
    247,83,121,3,1,15,8,91,239,32,1,167,89,246,144,48,
    226,40,196,220,17,185,27,87,153,57,246,89,79,110,172,21,
    115,204,195,248,165,205,49,225,239,38,114,142,121,2,252,50,
    253,204,108,121,196,247,83,66,162,78,121,162,60,128,159,27,
    94,36,25,159,75,57,191,56,172,47,81,155,39,154,224,92,
    44,27,231,115,98,91,57,63,37,196,130,107,213,170,81,214,
    125,142,242,193,230,206,8,4,231,23,208,177,251,219,46,37,
    205,70,248,223,42,212,2,95,246,180,192,138,158,252,6,5,
    48,232,19,92,244,91,104,207,99,29,132,194,153,143,36,117,
    210,192,222,165,82,193,241,167,135,128,16,239,224,10,94,111,
    184,168,93,136,184,131,3,189,59,180,11,20,19,240,46,206,
    214,242,231,12,155,140,67,47,96,16,121,209,114,253,230,62,
    24,229,213,7,16,227,102,26,245,18,121,1,61,116,58,88,
    227,123,67,35,142,100,244,221,238,5,239,110,231,59,121,228,
    189,70,181,178,110,152,115,197,202,121,207,89,31,113,73,196,
    32,60,8,229,122,121,13,43,75,190,46,162,175,8,181,42,
    0,193,149,204,47,25,249,240,250,18,84,68,77,240,37,184,
    56,86,173,20,94,49,0,197,138,241,122,112,66,172,103,107,
    154,233,26,17,145,23,196,76,185,223,131,232,20,43,110,136,
    32,12,21,72,226,75,67,207,90,169,90,140,28,210,189,224,
    13,233,124,167,61,77,231,15,215,180,240,164,140,52,195,226,
    195,188,191,207,160,227,157,204,49,44,194,49,8,95,52,162,
    205,103,100,68,123,78,240,227,160,100,60,39,157,194,139,223,
    94,247,226,175,9,69,251,53,182,214,20,173,135,241,117,233,
    247,248,186,183,94,171,79,149,126,67,224,118,213,190,216,128,
    62,117,226,209,144,64,126,81,86,135,122,81,106,254,165,81,
    14,201,61,224,192,102,248,42,58,238,101,254,3,14,151,55,
    32,220,57,114,173,44,117,222,153,152,212,185,234,76,90,87,
    44,13,10,39,174,223,83,219,89,247,212,174,91,11,18,235,
    155,107,78,98,121,24,95,151,18,235,111,67,18,235,26,156,
    87,163,161,253,36,219,91,251,173,56,232,15,135,209,151,229,
    177,37,199,208,223,135,144,95,61,218,226,31,5,110,47,215,
    253,22,214,25,43,93,195,73,236,43,184,121,34,98,61,252,
    254,187,43,93,71,39,31,88,185,147,186,255,238,74,215,209,
    201,135,64,139,231,187,243,255,54,37,120,151,60,125,6,147,
    234,111,208,49,206,1,193,236,30,215,89,215,235,209,116,68,
    143,214,106,95,98,107,237,247,33,174,82,20,191,15,201,78,
    10,237,184,25,186,176,51,114,215,172,50,93,248,109,182,214,
    116,161,135,241,117,169,11,255,133,37,162,76,156,61,40,91,
    19,254,71,8,249,213,163,76,254,83,224,150,217,194,174,65,
    153,108,108,156,210,128,54,88,249,170,167,11,162,154,212,53,
    193,202,87,133,30,240,41,11,183,201,242,74,26,41,124,29,
    7,34,248,87,235,32,202,210,13,52,191,224,37,239,54,128,
    129,1,61,193,40,116,109,57,250,150,101,26,15,11,186,236,
    51,203,220,139,158,155,44,250,10,6,251,28,115,213,30,49,
    114,64,227,57,77,229,233,59,24,228,255,204,34,2,84,254,
    76,86,114,140,222,95,49,75,181,162,22,173,242,30,101,49,
    82,65,141,117,80,121,254,7,124,173,21,165,243,125,182,214,
    148,142,135,241,117,169,116,254,155,197,80,58,253,193,77,144,
    227,244,72,201,55,33,47,185,109,254,32,132,251,234,209,57,
    255,43,112,203,192,58,190,26,157,243,111,222,180,66,186,186,
    34,208,55,187,242,36,224,119,120,127,223,99,13,30,172,46,
    225,193,154,169,106,166,101,60,184,98,112,30,191,100,72,60,
    56,239,10,74,17,156,71,162,97,187,155,104,184,86,156,233,
    105,101,173,73,70,15,227,22,70,173,177,23,16,181,14,114,
    157,51,35,31,245,102,68,86,212,186,83,89,11,81,235,94,
    177,110,153,193,149,100,79,132,223,36,56,195,185,124,73,171,
    122,126,147,168,107,94,132,147,127,177,55,133,27,249,67,136,
    87,184,12,235,239,139,66,28,6,158,197,17,92,113,105,210,
    239,187,96,103,204,224,122,230,143,35,82,196,48,40,255,94,
    137,27,174,192,128,131,107,78,212,13,94,189,168,251,33,52,
    2,135,148,213,107,72,141,121,194,236,154,226,134,65,169,21,
    246,4,71,93,243,60,193,145,82,72,174,13,6,251,107,64,
    137,138,34,66,2,156,169,217,213,154,29,150,61,11,47,71,
    246,92,241,169,210,168,248,126,127,232,254,152,99,61,68,81,
    247,90,16,69,19,107,78,20,77,92,215,162,104,114,21,139,
    162,173,33,220,174,225,172,220,235,109,219,220,172,49,95,40,
    75,251,149,30,141,54,21,7,245,238,58,234,200,19,149,117,
    194,231,99,109,75,104,198,151,180,98,65,14,226,238,11,111,
    10,250,246,0,234,226,124,116,109,207,46,18,216,75,125,142,
    5,61,138,96,135,167,117,225,227,185,166,71,88,184,138,211,
    193,221,247,36,4,120,68,84,172,161,10,235,154,70,105,182,
    18,133,131,104,19,40,193,41,178,241,167,147,92,149,34,55,
    180,200,85,59,189,138,44,172,77,223,166,92,181,39,195,213,
    169,87,252,53,191,171,68,187,133,18,237,17,63,132,236,21,
    15,137,235,19,15,137,91,47,30,18,215,47,30,18,55,32,
    222,206,48,40,222,206,48,36,222,206,48,44,222,206,48,34,
    222,206,48,42,222,206,48,38,222,206,48,46,222,206,176,65,
    188,157,97,35,94,96,232,132,56,157,23,24,102,39,152,190,
    137,42,147,238,155,10,179,155,153,62,73,149,45,238,43,9,
    241,214,197,45,84,153,114,223,61,136,231,226,77,137,231,226,
    109,19,207,197,219,46,158,139,183,67,60,23,111,167,120,46,
    222,46,241,92,188,221,226,185,120,123,196,115,241,50,226,185,
    120,123,235,207,197,195,171,15,15,50,125,31,65,166,97,118,
    236,143,84,14,171,204,236,200,172,57,179,195,195,248,165,205,
    142,107,148,195,67,254,93,70,15,238,164,39,152,201,144,198,
    200,89,19,3,238,91,217,178,232,246,150,228,114,131,125,1,
    135,143,20,251,2,79,254,220,31,71,219,141,6,102,57,135,
    119,212,224,97,74,146,158,82,142,112,152,111,204,27,226,80,
    50,212,64,201,108,165,82,148,250,176,117,103,192,3,113,104,
    24,108,160,161,104,148,37,145,224,60,158,150,198,59,24,135,
    130,190,0,5,213,74,85,154,221,199,199,154,142,105,60,141,
    53,76,63,63,245,91,197,66,94,234,67,200,123,105,33,234,
    35,31,74,152,38,171,101,52,213,71,62,156,48,77,220,118,
    108,17,77,245,145,143,196,164,169,81,16,243,158,185,10,42,
    201,36,169,71,144,228,12,124,99,194,20,113,158,110,13,69,
    222,192,55,37,76,145,213,42,138,188,129,143,198,164,104,32,
    64,81,232,23,157,77,62,50,34,18,119,115,28,109,211,31,
    64,223,40,85,109,57,239,184,161,183,244,96,180,99,113,176,
    95,31,192,94,106,18,247,45,201,205,122,190,104,104,114,204,
    114,231,181,212,124,180,91,99,114,125,195,196,159,215,170,242,
    54,48,61,244,132,143,120,91,156,21,24,15,16,48,143,23,
    95,22,139,149,188,180,3,18,48,12,140,122,123,114,252,36,
    205,205,73,174,165,227,201,25,187,146,156,156,228,12,187,35,
    14,218,65,161,111,202,243,43,211,211,235,104,184,19,201,201,
    77,83,150,115,25,59,23,131,221,153,220,41,149,159,89,114,
    179,154,164,167,130,227,148,42,6,188,43,166,4,109,80,188,
    166,102,73,124,74,62,105,95,12,121,119,128,138,180,159,138,
    219,234,84,40,120,26,236,165,84,48,159,40,21,200,39,74,
    189,84,62,17,22,22,249,68,30,205,190,84,162,134,104,151,
    132,124,162,123,20,38,242,137,250,253,249,68,215,190,142,195,
    65,118,172,89,11,14,63,74,91,75,122,214,179,24,246,254,
    228,180,200,156,89,41,203,123,110,58,141,246,64,114,50,77,
    154,76,0,135,99,176,7,147,61,71,88,86,97,190,44,247,
    197,109,206,152,167,18,165,195,52,200,40,151,74,135,51,230,
    67,137,210,81,40,91,134,41,241,145,246,160,195,25,243,225,
    56,123,162,65,56,113,213,37,111,87,147,92,18,35,62,18,
    115,53,70,88,72,202,58,148,72,91,17,40,139,250,184,143,
    38,188,75,74,149,37,233,187,4,99,158,78,206,10,174,149,
    11,207,202,121,112,36,225,239,12,119,38,185,88,129,105,44,
    25,166,37,239,37,39,238,120,103,19,244,127,84,76,121,239,
    247,197,96,143,37,106,3,151,12,115,94,178,13,76,67,170,
    161,21,80,216,106,201,124,122,92,224,150,161,112,244,149,51,
    202,17,51,165,120,166,218,133,90,15,138,62,20,253,40,144,
    105,171,14,163,24,69,49,142,98,35,10,228,189,145,29,174,
    110,65,141,30,93,137,252,31,21,185,40,234,46,20,245,20,
    146,189,40,16,47,85,17,166,83,17,47,114,18,82,224,189,
    87,111,162,175,40,224,30,83,225,161,81,225,31,80,113,90,
    85,113,100,82,97,121,171,39,81,192,86,85,97,54,169,208,
    213,117,145,226,230,177,212,15,10,210,114,63,219,57,30,123,
    48,227,120,49,164,147,129,190,242,91,25,87,248,215,118,165,
    171,116,10,90,162,95,87,230,244,74,141,227,23,78,126,209,
    149,87,146,95,146,78,126,49,246,224,17,157,72,117,217,229,
    166,186,168,51,153,221,145,91,126,149,229,179,60,229,9,167,
    20,91,27,249,44,30,198,137,231,179,140,53,110,30,169,57,
    45,36,94,95,189,178,174,88,37,185,44,79,199,177,39,54,
    134,102,88,114,62,11,105,174,215,196,33,33,204,36,50,19,
    89,72,233,190,54,14,254,163,17,248,75,75,98,33,115,33,
    23,215,166,14,162,47,41,131,133,140,156,103,98,26,164,155,
    34,38,191,5,41,44,100,171,105,77,160,165,5,169,43,100,
    114,206,54,129,150,22,164,172,144,229,156,143,73,75,148,140,
    149,159,170,66,246,191,222,4,82,228,231,168,208,41,198,104,
    2,41,242,147,83,232,44,54,23,147,148,145,16,41,173,200,
    74,153,143,163,69,134,67,36,72,203,76,161,67,240,66,28,
    228,135,66,200,75,77,76,41,36,59,241,210,146,83,200,231,
    176,24,147,247,35,230,94,102,110,10,185,76,206,197,89,128,
    137,16,1,210,115,83,200,229,83,76,150,139,228,166,164,148,
    146,181,96,101,166,165,148,227,160,30,150,252,242,82,83,200,
    51,88,73,86,110,202,74,76,33,135,102,53,217,83,167,204,
    196,20,242,197,62,27,83,116,70,232,92,169,73,41,228,75,
    54,3,68,52,51,31,5,219,28,249,40,1,146,91,154,147,
    98,41,44,161,156,148,241,48,55,74,207,75,161,128,192,249,
    100,181,136,180,224,53,5,50,46,36,43,204,164,9,3,196,
    95,150,147,63,67,200,205,72,161,0,210,115,137,83,33,57,
    31,133,34,96,175,75,156,10,201,217,40,20,194,187,24,147,
    138,176,95,147,175,133,97,202,76,123,112,162,238,52,232,165,
    100,149,125,94,171,106,249,130,164,159,78,32,11,85,12,248,
    250,16,29,10,91,45,177,235,55,8,220,232,151,227,107,55,
    118,141,163,144,10,187,188,190,29,221,16,117,208,118,144,22,
    166,70,50,213,147,152,220,135,88,146,97,106,127,112,218,178,
    205,66,121,254,149,224,244,43,193,233,43,237,240,31,247,164,
    79,138,173,141,224,180,135,113,51,131,211,206,230,145,31,156,
    126,211,202,202,96,149,4,167,95,136,163,118,125,46,122,119,
    134,91,17,156,254,201,132,44,7,143,4,201,193,233,159,138,
    131,255,104,4,254,114,131,211,63,29,7,253,193,16,250,50,
    131,211,47,198,180,160,55,133,176,111,93,112,250,205,77,160,
    165,85,193,233,159,105,2,45,173,10,78,191,37,38,45,81,
    50,182,69,193,233,183,54,129,148,22,5,167,223,214,4,82,
    90,20,156,126,123,114,206,12,151,148,86,4,167,127,54,142,
    22,25,14,145,32,55,56,253,142,56,200,15,133,144,151,26,
    156,126,103,178,19,47,55,56,253,115,49,121,63,98,238,165,
    7,167,223,21,103,1,38,66,4,180,38,56,253,238,100,185,
    72,110,112,250,61,201,90,176,50,131,211,239,141,131,122,88,
    242,75,14,78,191,47,89,185,41,53,56,253,254,100,79,157,
    210,131,211,31,136,41,58,35,116,174,252,224,244,7,3,68,
    72,12,78,59,36,183,52,56,253,243,74,19,130,211,130,27,
    91,19,156,254,72,178,90,68,110,112,250,163,201,10,51,169,
    193,233,143,37,127,134,104,65,112,250,227,137,83,209,138,224,
    244,47,36,78,69,43,130,211,191,24,147,138,176,95,83,118,
    112,154,162,140,159,72,86,203,203,140,74,83,128,244,147,33,
    252,21,182,90,162,209,159,18,184,253,176,71,163,93,99,65,
    90,52,26,121,147,207,43,201,71,163,251,235,252,124,114,197,
    119,30,190,18,143,126,37,30,93,223,227,191,234,201,159,20,
    91,27,241,104,15,227,196,227,209,27,194,219,71,126,68,250,
    215,86,86,8,171,36,34,253,235,113,84,174,207,31,230,205,
    113,43,98,210,191,17,135,136,40,70,145,30,149,254,205,56,
    20,140,71,82,32,55,46,253,153,132,14,147,30,1,50,35,
    211,159,141,105,62,79,70,46,64,171,98,211,191,213,20,106,
    90,21,157,254,237,166,80,211,170,248,244,239,196,164,38,90,
    226,182,40,66,253,187,77,33,166,69,49,234,223,107,10,49,
    45,138,82,127,46,38,49,99,17,196,180,34,78,253,251,113,
    180,202,104,4,17,114,35,213,127,16,7,253,145,8,244,165,
    198,170,255,48,233,201,151,27,173,254,163,152,123,32,114,254,
    165,199,171,255,56,206,34,108,142,32,161,53,17,235,207,39,
    205,75,114,99,214,127,146,180,117,43,51,106,253,167,113,144,
    143,210,4,146,227,214,95,72,90,138,74,141,92,255,89,210,
    103,83,233,177,235,63,143,41,72,35,245,176,252,232,245,23,
    3,100,52,51,122,141,13,239,139,94,11,162,91,26,191,254,
    146,146,84,252,218,151,140,89,231,201,214,68,176,191,146,180,
    86,145,27,195,254,106,210,130,77,106,20,251,107,205,56,99,
    180,32,142,253,245,38,208,209,138,72,246,95,52,129,142,86,
    196,178,255,50,38,29,81,222,208,150,68,179,255,42,105,205,
    47,61,158,253,141,16,5,10,91,45,241,236,191,22,184,253,
    112,197,179,177,11,131,241,108,207,124,144,22,209,198,111,28,
    62,173,36,30,209,166,232,181,216,6,15,132,182,193,139,44,
    98,27,32,83,75,95,46,107,165,66,62,55,147,215,138,154,
    121,170,60,87,113,152,172,201,91,24,145,177,191,195,44,164,
    8,239,184,216,191,138,86,83,26,246,8,137,125,43,30,246,
    3,62,236,193,132,210,112,71,48,236,159,226,225,62,22,154,
    121,169,20,32,26,246,237,120,20,140,132,40,56,162,75,195,
    31,1,177,127,141,135,255,176,15,255,251,43,102,169,86,212,
    164,161,143,8,216,191,39,199,64,51,85,205,180,140,7,101,
    50,16,162,94,255,213,64,129,160,130,244,111,183,71,65,99,
    218,9,161,143,119,75,23,202,5,123,166,80,154,65,159,205,
    86,25,136,107,125,63,222,140,175,115,81,126,220,184,96,75,
    153,99,132,175,254,39,30,206,120,255,140,149,95,48,116,76,
    242,125,75,70,89,14,230,136,85,41,169,58,230,47,195,190,
    132,112,175,26,102,161,162,23,242,64,254,100,173,228,196,219,
    233,229,66,206,149,38,82,128,0,85,123,42,30,127,215,170,
    186,102,27,52,235,77,231,111,68,114,186,98,224,11,15,79,
    213,172,228,13,203,82,249,89,196,126,172,102,212,140,102,35,
    141,0,78,95,12,164,7,234,72,131,61,164,224,140,168,205,
    224,203,199,89,157,99,205,183,143,49,198,104,140,121,197,115,
    184,140,50,134,213,155,141,42,194,71,27,99,160,138,151,216,
    81,103,143,112,237,215,108,100,17,36,218,92,71,150,188,164,
    5,212,11,244,58,192,250,232,143,1,53,92,181,150,45,21,
    16,181,159,249,213,8,103,90,46,193,236,101,58,11,18,13,
    167,181,146,113,159,105,86,76,117,10,45,183,161,216,193,220,
    51,34,137,188,202,236,34,183,178,212,125,226,142,92,217,56,
    159,47,106,124,191,194,225,169,30,68,49,205,220,64,255,12,
    199,251,236,242,41,55,157,48,7,50,76,99,158,79,145,97,
    170,79,139,253,126,202,121,53,120,253,18,189,55,44,191,164,
    57,149,114,165,108,80,5,74,143,120,66,47,88,213,162,198,
    207,95,130,188,170,62,71,255,207,243,255,183,57,13,108,66,
    182,92,65,134,29,189,194,139,119,163,149,233,118,222,95,177,
    192,251,196,146,230,132,203,67,87,95,0,230,111,102,174,103,
    164,126,136,10,98,253,14,209,160,126,78,9,226,142,57,57,
    105,44,21,52,187,80,41,19,54,176,168,169,2,203,72,253,
    48,115,125,170,190,96,169,175,255,47,251,47,135,135,255,154,
    152,217,160,177,30,108,244,77,230,186,159,253,246,112,176,201,
    119,152,235,59,241,153,156,193,22,223,245,214,208,179,234,194,
    216,126,47,220,40,220,211,15,152,107,230,158,169,217,213,154,
    29,188,136,87,105,145,17,233,127,75,188,239,58,158,97,22,
    241,90,135,64,155,106,67,27,239,71,211,190,54,56,171,71,
    230,56,5,90,225,44,75,71,66,58,91,209,241,132,108,124,
    178,148,201,216,36,11,142,76,34,178,46,72,65,147,214,35,
    45,66,98,153,228,30,73,20,218,169,77,150,6,180,145,143,
    151,42,122,173,104,156,232,193,0,88,185,1,165,87,233,234,
    239,82,58,82,189,74,154,255,235,86,250,148,54,165,183,175,
    43,221,213,209,213,158,86,58,8,50,172,116,167,187,186,199,
    198,187,148,238,212,216,70,94,174,228,165,72,225,223,88,23,
    181,235,67,233,192,199,198,8,210,73,101,79,253,106,0,226,
    212,187,169,60,72,229,222,134,50,237,244,255,210,255,122,87,
    188,242,255,43,197,143,152,
};

EmbeddedPython embedded_m5_internal_stats(
    "m5/internal/stats.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/build/ARM/python/swig/stats.py",
    "m5.internal.stats",
    data_m5_internal_stats,
    7815,
    66061);

} // anonymous namespace
