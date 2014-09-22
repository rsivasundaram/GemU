#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_WireBuffer[] = {
    120,156,181,144,65,75,195,64,16,133,103,211,180,104,17,241,
    44,8,61,6,161,93,5,75,47,34,82,60,87,72,4,177,
    151,101,155,157,152,74,182,9,217,4,155,179,254,111,157,73,
    172,185,121,115,147,60,222,176,179,179,95,94,12,63,107,64,
    223,253,4,192,221,144,49,244,10,200,0,158,216,121,144,9,
    176,30,172,61,16,102,0,232,65,34,192,248,240,9,240,1,
    240,178,30,128,25,66,20,140,232,224,246,139,86,32,200,85,
    44,151,157,61,38,137,182,246,113,243,134,113,85,157,82,21,
    214,155,230,121,91,226,178,78,18,44,227,3,4,119,47,25,
    226,156,12,2,172,5,163,208,189,68,195,215,16,131,15,97,
    20,48,108,232,241,232,49,73,63,200,93,80,105,209,202,146,
    230,75,215,184,138,124,191,61,75,211,96,200,167,142,72,148,
    218,105,139,74,181,35,148,178,185,169,51,46,125,110,104,10,
    108,169,227,253,94,197,153,118,174,237,226,42,69,109,176,12,
    152,182,23,183,34,145,105,110,81,150,218,202,135,252,125,151,
    229,218,56,249,138,118,62,117,149,222,100,56,213,58,185,186,
    94,160,214,102,97,164,43,99,249,7,104,209,180,191,55,225,
    217,156,235,72,240,179,10,252,67,156,118,62,43,52,221,229,
    170,147,174,250,205,55,20,135,112,254,31,178,13,242,182,139,
    238,142,179,119,12,56,22,103,222,55,205,146,139,185,
};

EmbeddedPython embedded_m5_objects_WireBuffer(
    "m5/objects/WireBuffer.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/mem/ruby/system/WireBuffer.py",
    "m5.objects.WireBuffer",
    data_m5_objects_WireBuffer,
    318,
    600);

} // anonymous namespace