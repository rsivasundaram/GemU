#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Process[] = {
    120,156,181,85,109,111,35,53,16,158,77,210,180,73,123,125,
    201,1,18,39,144,140,196,135,45,208,230,144,238,229,3,136,
    151,131,47,72,220,129,146,114,18,69,40,114,215,78,226,222,
    122,189,178,189,109,138,248,86,196,95,228,231,192,204,236,110,
    46,160,187,111,92,94,156,241,99,175,231,241,204,60,147,12,
    154,87,23,191,95,9,128,112,134,134,194,79,2,57,128,77,
    224,60,129,132,230,29,200,59,112,214,88,93,180,186,160,19,
    152,39,160,122,240,7,192,45,192,207,231,61,80,91,160,123,
    140,246,215,232,22,168,109,152,166,59,120,176,249,27,95,105,
    130,86,28,224,48,53,246,135,139,75,157,197,26,162,225,163,
    184,141,227,143,222,101,58,132,108,147,221,19,98,247,39,26,
    26,136,20,50,57,239,128,238,2,121,69,154,232,184,15,151,
    219,68,15,169,221,118,224,124,167,69,152,22,33,131,22,233,
    19,37,66,134,140,236,130,222,131,203,59,160,118,24,220,103,
    240,0,212,0,212,144,145,67,208,71,160,118,249,74,232,107,
    4,147,105,186,135,84,38,29,28,194,62,13,198,142,203,154,
    245,233,114,25,137,112,102,138,48,194,223,185,201,117,33,173,
    22,115,231,69,136,202,20,177,71,203,174,138,225,238,43,214,
    17,175,55,104,239,95,185,129,240,123,228,243,38,68,109,69,
    227,86,92,155,60,23,190,42,132,171,29,60,122,240,244,73,
    120,23,13,43,87,198,86,86,4,243,155,22,110,46,226,82,
    227,49,50,123,145,17,253,246,251,141,104,174,242,123,66,1,
    189,173,243,62,77,105,237,89,248,24,199,11,231,114,97,101,
    153,126,173,148,23,87,18,199,79,4,219,101,109,227,205,216,
    201,241,103,41,101,141,31,173,67,145,135,230,206,74,215,75,
    52,132,47,113,24,47,157,213,99,47,237,248,91,119,93,228,
    78,170,48,94,104,251,240,4,25,94,228,250,68,202,249,253,
    79,31,107,41,213,99,53,14,62,27,83,164,155,250,56,45,
    111,34,49,214,171,210,249,56,179,58,46,157,98,170,129,92,
    67,39,165,240,69,170,188,217,140,34,56,155,197,33,79,172,
    83,85,78,83,98,21,111,74,205,198,153,175,52,239,150,23,
    33,122,153,69,222,157,173,86,179,165,150,24,246,184,69,181,
    41,145,108,236,83,249,70,111,138,5,131,166,40,49,107,4,
    98,246,90,19,243,228,26,115,202,169,98,19,159,215,69,228,
    176,200,226,134,161,58,145,236,236,169,182,206,223,76,49,136,
    124,51,76,221,140,83,53,163,184,198,93,142,165,12,161,190,
    234,100,208,134,242,255,13,42,151,245,7,116,24,213,111,63,
    169,223,35,126,191,157,140,58,204,227,123,115,165,95,171,211,
    191,254,163,83,82,104,151,90,195,229,214,134,66,251,160,183,
    25,233,177,172,118,154,233,2,54,4,187,126,164,223,10,182,
    71,130,69,253,54,106,221,219,64,6,140,220,217,64,134,173,
    162,215,200,46,35,7,27,200,94,171,241,26,193,62,176,207,
    200,17,35,35,80,216,10,14,25,185,75,194,63,34,225,191,
    247,26,225,115,10,62,228,154,212,89,197,209,22,169,187,194,
    82,48,74,7,145,89,245,203,253,95,133,153,139,160,227,113,
    72,89,19,214,202,66,137,220,20,184,117,227,177,50,175,130,
    144,126,81,89,172,151,112,28,222,162,83,139,43,227,93,65,
    8,157,16,177,254,2,107,60,171,60,149,149,184,118,254,5,
    130,66,25,143,125,21,75,201,40,98,68,77,181,10,218,11,
    163,2,241,215,243,57,46,99,6,69,139,82,217,47,176,94,
    75,154,140,254,181,101,13,83,129,182,221,198,40,67,105,231,
    211,74,46,105,177,177,196,97,56,169,195,83,229,50,26,87,
    136,210,81,127,144,200,113,105,178,165,136,142,154,144,167,142,
    209,110,73,233,138,19,242,50,161,10,155,112,139,61,160,225,
    144,6,114,197,34,121,25,36,174,196,231,124,211,90,151,220,
    109,172,226,95,12,86,61,191,174,231,223,53,178,171,140,98,
    185,107,50,8,88,180,192,162,1,202,6,40,201,32,133,254,
    132,207,62,122,192,189,1,233,242,77,222,132,238,168,170,198,
    116,216,189,181,238,70,201,65,163,188,151,239,103,41,113,138,
    20,30,251,240,116,253,47,58,33,30,252,199,138,104,73,225,
    8,204,152,102,222,173,106,89,179,143,55,192,157,61,125,94,
    247,213,47,222,167,67,9,56,76,134,201,176,243,206,254,63,
    199,139,242,163,
};

EmbeddedPython embedded_m5_objects_Process(
    "m5/objects/Process.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/sim/Process.py",
    "m5.objects.Process",
    data_m5_objects_Process,
    996,
    2152);

} // anonymous namespace
