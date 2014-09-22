/*
 * DO NOT EDIT THIS FILE! Automatically generated
 */

#include "base/debug.hh"
#include "debug/GDBAcc.hh"
#include "debug/GDBExtra.hh"
#include "debug/GDBMisc.hh"
#include "debug/GDBRead.hh"
#include "debug/GDBRecv.hh"
#include "debug/GDBSend.hh"
#include "debug/GDBWrite.hh"

namespace Debug {

CompoundFlag GDBAll("GDBAll", "All Remote debugging flags",
    GDBAcc,
    GDBExtra,
    GDBMisc,
    GDBRead,
    GDBRecv,
    GDBSend,
    GDBWrite);

} // namespace Debug