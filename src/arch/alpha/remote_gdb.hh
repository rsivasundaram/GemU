/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Nathan Binkert
 */

#ifndef __ARCH_ALPHA_REMOTE_GDB_HH__
#define __ARCH_ALPHA_REMOTE_GDB_HH__

#include <map>

#include "arch/alpha/kgdb.h"
#include "arch/alpha/types.hh"
#include "base/pollevent.hh"
#include "base/remote_gdb.hh"
#include "base/socket.hh"
#include "cpu/pc_event.hh"

class System;
class ThreadContext;

namespace AlphaISA {

class RemoteGDB : public BaseRemoteGDB
{
  protected:
    Addr notTakenBkpt;
    Addr takenBkpt;

  protected:
    void getregs();
    void setregs();

    void clearSingleStep();
    void setSingleStep();

    // Machine memory
    bool acc(Addr addr, size_t len);
    bool write(Addr addr, size_t size, const char *data);

    virtual bool insertHardBreak(Addr addr, size_t len);

  public:
    RemoteGDB(System *system, ThreadContext *context);
};

} // namespace AlphaISA

#endif // __ARCH_ALPHA_REMOTE_GDB_HH__
