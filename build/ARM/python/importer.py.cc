#include "sim/init.hh"

namespace {

const uint8_t data_importer[] = {
    120,156,181,85,81,111,27,69,16,158,189,179,207,62,59,105,
    154,150,24,42,80,49,18,72,22,34,113,65,170,42,80,64,
    64,106,137,10,145,162,179,74,105,132,116,58,223,174,147,59,
    159,239,78,187,235,166,150,236,151,134,223,202,3,127,2,102,
    230,238,210,240,78,18,101,53,187,51,251,205,55,223,206,92,
    98,168,127,92,252,251,126,8,96,126,66,67,2,40,128,185,
    0,41,224,79,128,43,128,87,103,184,113,64,186,144,57,112,
    230,128,18,116,122,230,130,114,33,109,193,89,11,148,3,105,
    27,82,143,78,174,4,8,12,157,142,90,8,102,119,112,57,
    41,164,122,182,44,11,109,149,142,155,156,2,255,126,164,156,
    95,0,39,164,28,64,9,49,65,149,25,19,96,86,50,90,
    148,155,140,54,4,21,110,44,106,8,135,240,9,102,23,141,
    4,96,3,16,114,13,211,17,185,79,121,181,29,92,150,133,
    92,101,202,84,7,132,97,84,54,31,17,21,94,204,9,46,
    227,139,98,169,198,58,90,142,159,22,151,121,86,68,210,140,
    207,213,242,241,161,177,209,44,83,135,81,52,127,244,229,19,
    21,69,242,137,28,27,29,143,203,181,189,40,242,113,82,87,
    119,132,251,46,226,132,97,146,39,54,12,63,33,96,135,203,
    141,219,184,182,107,177,153,241,215,104,108,92,98,156,2,44,
    60,208,31,131,101,213,241,112,240,214,129,28,221,184,113,96,
    211,130,121,19,135,190,99,174,142,80,79,205,71,184,126,102,
    134,81,166,85,36,215,195,121,177,202,229,48,201,135,13,33,
    14,11,184,228,59,184,252,96,173,78,102,43,171,38,90,23,
    122,68,116,2,138,96,210,243,36,83,121,180,84,172,86,52,
    51,101,100,47,26,229,216,102,225,241,45,111,65,180,30,165,
    148,50,172,30,233,83,130,246,88,182,61,113,215,137,221,90,
    182,235,199,126,31,42,109,174,165,187,79,246,244,198,179,191,
    171,155,88,159,22,185,26,185,255,173,118,149,101,92,45,249,
    169,188,91,168,170,207,170,230,77,89,159,223,44,171,229,196,
    196,165,95,119,5,143,66,137,132,37,207,65,6,176,117,26,
    91,192,214,109,108,7,182,45,88,239,83,79,164,46,105,240,
    96,219,134,151,249,33,180,84,11,22,61,208,18,132,16,27,
    158,71,244,226,48,162,127,211,126,23,127,156,11,248,125,253,
    23,233,133,199,161,199,34,86,72,127,184,176,245,96,219,129,
    109,151,250,44,237,64,218,173,7,255,10,1,125,154,250,173,
    15,27,31,100,159,85,255,155,195,122,144,246,97,211,161,92,
    120,162,118,200,150,30,95,217,229,43,61,80,119,96,211,227,
    115,12,115,9,63,135,230,238,30,108,60,186,43,187,176,112,
    64,15,5,82,73,239,130,244,233,144,40,238,243,83,163,113,
    15,242,1,217,233,253,218,43,123,240,160,114,32,4,25,239,
    193,166,75,70,122,0,173,23,47,243,135,168,202,128,85,89,
    8,82,165,209,224,219,183,0,36,3,70,78,71,244,133,58,
    77,254,193,31,123,128,230,47,143,195,231,191,77,130,224,217,
    211,73,248,235,171,112,250,252,69,112,50,177,244,68,243,40,
    51,85,187,88,189,82,150,218,105,173,140,165,38,211,124,172,
    222,168,216,244,111,140,63,245,0,185,143,18,238,45,122,111,
    211,220,55,245,253,17,245,50,131,97,231,88,10,41,12,111,
    205,218,84,13,76,221,250,179,90,243,200,242,160,228,234,178,
    238,40,222,134,33,117,167,210,97,200,211,170,242,215,137,46,
    114,198,56,87,150,169,103,197,165,210,1,85,106,61,38,154,
    24,107,152,51,13,61,27,244,1,225,251,113,177,44,233,144,
    210,206,34,83,125,18,8,196,148,89,98,235,207,27,13,12,
    230,235,215,155,120,17,157,43,220,19,29,93,70,218,38,54,
    65,10,85,44,165,64,95,181,145,73,140,194,88,31,55,147,
    55,177,42,41,110,212,111,38,51,96,139,158,33,24,208,114,
    173,13,150,203,228,112,208,8,46,160,42,130,110,163,78,241,
    90,105,157,200,234,77,48,228,150,70,153,110,214,194,63,34,
    236,175,40,131,183,35,240,215,113,197,190,216,197,177,222,71,
    203,23,3,231,67,241,80,28,136,3,231,3,113,15,189,3,
    225,59,123,232,239,137,142,51,242,26,218,97,72,218,214,178,
    133,53,114,24,6,252,161,34,133,2,250,167,22,28,53,197,
    252,255,101,5,4,54,36,64,226,227,9,223,245,61,223,227,
    97,56,29,117,154,118,41,102,169,138,45,199,86,47,210,173,
    186,149,81,152,40,191,39,182,73,196,141,193,151,162,178,84,
    185,188,37,222,204,224,184,18,236,187,107,254,131,111,118,80,
    123,95,252,11,129,24,222,239,
};

EmbeddedPython embedded_importer(
    "importer.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/python/importer.py",
    "importer",
    data_importer,
    1048,
    2264);

} // anonymous namespace
