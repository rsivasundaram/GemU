#include "sim/init.hh"

namespace {

const uint8_t data_m5_event[] = {
    120,156,181,85,75,111,19,87,20,62,119,198,158,120,18,7,
    40,164,166,10,72,13,11,144,55,196,69,34,202,162,21,173,
    2,136,46,170,144,78,216,224,205,104,50,115,237,140,153,71,
    152,185,134,68,10,171,240,19,144,216,247,143,182,231,59,227,
    235,152,68,234,10,143,61,199,231,62,124,30,223,119,206,189,
    49,205,158,54,191,127,108,17,213,95,89,73,248,171,40,35,
    26,206,117,69,67,71,116,7,122,238,210,208,165,188,69,195,
    54,41,94,30,174,96,83,226,210,103,214,59,148,180,72,187,
    52,226,153,54,125,38,186,32,122,59,244,41,241,72,251,50,
    187,50,159,93,165,164,131,193,112,141,18,95,148,46,37,171,
    48,192,187,19,158,236,210,184,77,195,117,152,63,236,175,115,
    108,233,191,252,236,247,29,86,205,26,139,131,51,115,92,22,
    47,63,232,194,152,30,143,95,101,229,81,148,29,166,249,95,
    101,121,242,242,52,53,178,20,99,63,94,151,223,231,200,243,
    33,43,231,10,41,189,235,80,245,19,25,69,19,135,62,41,
    42,216,176,75,231,178,122,225,176,87,197,59,247,251,45,56,
    132,216,47,11,109,0,87,227,19,49,188,208,163,104,154,153,
    240,160,74,3,128,217,68,7,87,229,209,196,116,248,247,164,
    74,203,42,53,103,125,89,134,168,159,179,24,28,151,185,30,
    84,81,62,120,81,126,44,178,50,74,234,193,88,231,59,143,
    107,19,29,101,250,113,20,141,126,121,178,171,163,40,217,77,
    6,117,21,15,78,36,219,65,190,51,208,240,190,205,99,143,
    237,196,149,142,140,126,4,179,24,146,234,242,39,64,0,177,
    37,24,33,237,33,241,13,86,52,3,173,132,50,18,202,92,
    10,14,37,232,235,56,237,252,63,78,60,100,168,24,167,73,
    235,18,51,197,187,45,110,64,42,128,69,9,39,16,24,49,
    87,79,79,116,37,216,132,97,90,164,38,12,27,212,176,161,
    214,217,40,240,44,82,223,21,174,192,135,157,111,129,154,145,
    219,196,82,68,185,14,67,179,42,131,188,76,166,25,15,155,
    4,252,111,34,250,190,97,1,156,109,24,132,39,79,25,148,
    250,65,85,142,43,93,215,77,5,91,34,149,37,114,243,146,
    72,146,214,113,192,40,148,75,58,221,25,149,45,75,231,107,
    100,74,11,180,57,104,68,133,138,103,77,102,195,22,104,132,
    210,134,156,180,105,226,65,49,43,52,233,96,55,38,91,116,
    247,10,209,64,39,0,134,193,186,5,75,90,32,229,38,1,
    214,76,119,90,38,162,74,214,239,5,240,58,62,214,192,216,
    32,220,124,135,125,112,212,211,234,77,26,191,235,75,193,128,
    136,224,22,196,205,229,21,4,138,188,238,8,186,119,212,77,
    229,171,88,205,144,158,3,183,75,114,16,50,118,19,5,16,
    248,32,188,215,123,245,39,176,96,20,229,192,152,175,93,5,
    8,137,236,215,189,5,74,31,108,189,73,115,189,85,148,31,
    183,30,142,234,49,175,252,243,101,51,223,107,186,229,54,196,
    157,121,222,63,204,147,87,22,144,37,28,35,77,245,199,81,
    150,133,225,175,48,220,154,193,33,205,17,224,152,11,186,150,
    216,96,99,137,173,128,242,121,106,35,240,148,239,205,201,112,
    44,25,183,200,162,205,224,163,144,213,188,12,93,219,202,92,
    119,186,42,162,76,78,27,49,46,77,53,214,205,141,240,247,
    84,79,181,252,71,54,164,69,162,79,151,144,204,93,182,243,
    187,237,107,82,215,83,217,184,158,202,98,87,73,15,252,8,
    129,242,145,12,184,61,174,100,32,117,178,132,154,128,183,122,
    17,175,189,203,76,2,116,170,180,43,223,181,184,103,5,198,
    60,74,139,247,253,27,182,138,205,141,5,34,182,197,110,147,
    12,76,4,18,250,61,178,247,195,125,8,24,108,110,10,57,
    69,0,95,176,105,61,133,161,148,231,146,10,79,170,230,183,
    230,200,127,246,51,12,35,5,190,32,156,158,227,57,221,78,
    207,235,173,249,174,239,254,7,71,56,216,20,
};

EmbeddedPython embedded_m5_event(
    "m5/event.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/python/m5/event.py",
    "m5.event",
    data_m5_event,
    892,
    2356);

} // anonymous namespace