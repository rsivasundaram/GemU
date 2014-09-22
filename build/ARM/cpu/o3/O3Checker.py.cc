#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_O3Checker[] = {
    120,156,181,80,177,78,195,48,16,61,39,37,130,78,32,177,
    34,117,180,144,26,131,74,213,5,33,68,59,3,74,97,160,
    139,101,108,151,32,226,38,138,91,209,206,229,191,225,206,73,
    224,11,176,236,167,119,246,249,222,187,211,208,174,24,207,237,
    0,192,95,33,49,184,25,20,0,79,196,34,40,24,56,6,
    11,6,204,196,96,25,44,25,152,30,124,1,236,1,94,22,
    17,152,3,152,243,4,63,190,127,227,226,12,217,154,224,188,
    161,125,132,105,110,245,135,173,167,143,207,235,35,12,31,70,
    237,133,238,12,80,230,29,25,56,69,98,129,212,80,156,138,
    163,114,12,217,156,71,248,144,17,248,19,4,93,109,68,57,
    18,186,41,147,230,57,239,145,214,33,130,148,43,229,172,148,
    65,88,74,87,154,77,65,97,72,216,85,54,220,235,237,86,
    230,86,25,91,115,146,255,3,63,67,16,121,233,172,168,149,
    19,179,242,115,85,148,202,120,241,102,221,120,232,215,234,181,
    176,67,165,150,23,151,19,171,148,153,24,225,107,45,90,59,
    191,125,165,213,46,120,29,80,69,18,78,88,194,238,57,141,
    57,12,192,141,211,74,97,125,159,177,174,173,255,242,17,102,
    114,221,76,225,230,172,243,211,103,199,209,15,139,46,108,167,
};

EmbeddedPython embedded_m5_objects_O3Checker(
    "m5/objects/O3Checker.py",
    "/home/ram/Downloads/gem5-stable-aaf017eaad7d/src/cpu/o3/O3Checker.py",
    "m5.objects.O3Checker",
    data_m5_objects_O3Checker,
    288,
    510);

} // anonymous namespace
