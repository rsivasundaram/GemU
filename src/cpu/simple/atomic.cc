/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 * Authors: Steve Reinhardt
 */

#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "arch/utility.hh"
#include "base/bigint.hh"
#include "base/output.hh"
#include "config/the_isa.hh"
#include "cpu/simple/atomic.hh"
#include "cpu/exetrace.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/SimpleCPU.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "mem/physical.hh"
#include "params/AtomicSimpleCPU.hh"
#include "sim/faults.hh"
#include "sim/system.hh"
#include "sim/full_system.hh"
//#include "base/bitunion.hh"



using namespace std;
using namespace TheISA;

int idleCycles=0, begCycles=0, endCycles=0, changed=0, mode;
uint32_t fault;
double total, prevTotal=0, slp_energy=0, act_energy=0;
uint64_t cycleCount, slp_count, prev_slp=0, prev_act=0;
Tick slp_tick;

#define ADAPTIVE 0

/*
void getBits()
{ 	
	class Data;
	prev_act=BitfieldBackend::BitfieldBase<Data>::getBits(24,21);
	
	
}
*/
void enableFaults(uint32_t x)
{	
	if (x==0){
		fault=0;
		return;
	}	
	fault = fault | x;
}

void disableFaults(uint32_t x)
{
	x = (x ^ 512);
	//Check to see if the user tried to disable a fault that is not enabled.
	uint32_t check=checkFaults();
	if ((check | x) != check){
		return;
	}	
	fault = fault ^ x;
}

uint32_t checkFaults()
{
	//000000001 multiply to divide
	// divide to multiply
	//000000010 add to subtract
	//000000100 subtract to add
	//000001000 multiply to add
	//000010000 add to multiply
	//h divide to add
	//000100000 add to divide
	//001000000 multiply to subtract
	//010000000 subtract to multiply
	//l divide to subtract
	//100000000 subtract to divide
	
	return fault;

	/*if (fault == 0){
		return 'd';	
	}else if ((fault & 0x000000001) == 0x000000001){
		return 'a';
	}else if ((fault & 0x000000010) == 0x000000010){
		return 'c';
	}else if ((fault & 0x000000100) == 0x000000100){
		return 'e';
	}else if ((fault & 0x000001000) == 0x000001000){
		return 'f';
	}else if ((fault & 0x000010000) == 0x000010000){
		return 'g';
	}else if ((fault & 0x000100000) == 0x000100000){
		return 'i';
	}else if ((fault & 0x001000000) == 0x001000000){
		return 'j';
	}else if ((fault & 0x010000000) == 0x010000000){
		return 'k';
	}else if ((fault & 0x100000000) == 0x100000000){
		return 'm';
	}else{
		return 'z';
	}
	*/

	
}
void parse()
{
	FILE * pFile;
	//double val;
	char temp[80];
	uint8_t count=0;
	total=0;
	if (prevTotal==0){
		pFile = fopen("params.txt", "r");
		while (fgets(temp,sizeof(temp),pFile)!=NULL){
			if (*temp != '#'){
				if (total==0){
					setClockFrequency(atof(temp));
					total+=atof(temp);
					continue;
				}
				gemu_pm_change_parameter(1, count, atof(temp));
				total+=atof(temp);
				count++;
			}				
		}
		fclose(pFile);
		prevTotal=total;
	}else{
		pFile = fopen("params.txt", "r");
		while (fgets(temp,sizeof(temp),pFile)!=NULL){
			if (*temp != '#'){
				total+=atof(temp);
			}				
		}
		fclose(pFile);
		if (prevTotal!=total){
		//Params have been changed, so edit energy accordingly before changing params
			if (mode==0){
				gemu_pm_slp_energy();
			}else{
				//1 in the parameter is just a filler class for now
				gemu_pm_act_energy(1);
			}
			total=0;
			pFile = fopen("params.txt", "r");
			while (fgets(temp,sizeof(temp),pFile)!=NULL){
				if (*temp != '#'){
					if (total==0){
						setClockFrequency(atof(temp));
						total+=atof(temp);
						continue;
					}
					gemu_pm_change_parameter(1, count, atof(temp));
					total+=atof(temp);
					count++;
				}				
			}	
			fclose(pFile);
			prevTotal=total;	
		}				
	}

	/*
	int fd = open("/dev/chdev", O_RDONLY);
	if (fd >=0){
		garbage=read(fd, temp ,sizeof(double));	
		close(fd);	
		vemu_pm_change_parameter(1, 0, atof(temp));
	}
	*/
}

typedef struct {
	double T;	// temperature in celsius
	double Vdd;	// supply voltage
	double Nd;	// dynamic power multiplier
	double Bd;	// nominal value: 10.7
	double Cd;	// nominal value: 0.605
	double Nl;	// leakage power multiplier
	double Al;	// nominal value -9210
	double Bl;	// nominal value 890
	double Vnl;	// nominal value 0.2
	double Vpl;	// nominal value 0.232
	double delta_vtp;	// pmos threshold degradation
	double aging;	// 1 or 0 for upper/lower bound for aging
	double Vdd_scalar;	// new Vdd scalar to compensate aging
	//double Freq_scalar; // alternative new frequency scalar to compensate aging
} esweek_pm;

static union {
	esweek_pm parameters;
	double pm_array[13];
} pm_parameters;

/*
double gemu_aging_evaluation (void) {
	esweek_pm *p = &pm_parameters.parameters;
	double act_time = (double)vemu_get_act_time(0)/1e9*TIME_SCALAR;
	double clk_gated_time = (double)vemu_get_act_time(1)/1e9*TIME_SCALAR;
	double slp_time = (double)vemu_get_slp_time()/1e9*TIME_SCALAR;
	double omega;
	if (p->aging<0.5) {
		omega = (0.5 * act_time + clk_gated_time) / (slp_time + act_time);
	} else {
		omega = (act_time + clk_gated_time) / (slp_time + act_time);
	}
	if(omega>0.99) { omega=0.99; }
	double t = act_time + slp_time + clk_gated_time;
	double temp = p->T + 273;
	double C = 1e8 * exp(-9.8642e+03 / temp);
	double Kv = 2.9642e-44 * (p->Vdd - 0.65) * pow(C,0.5) * exp((p->Vdd-0.65)/0.1008);
	double bt = (1.1902e-09 + 0.0707* pow((1-omega)*C,0.5)) /(1.3224*1e-9+ pow((C*t),0.5) );
	bt = 1 - bt;
	double delta_vt = 1.9174e+09 * pow((( pow(0.1*omega,0.5) *Kv)/(1- pow(bt,3.6232) )),0.276);
	double nom_delay = p->Vdd/pow((p->Vdd-(0.4+0)),1.1) + p->Vdd/pow((p->Vdd-0.5),1.1);
	#ifdef ADAPTIVE
	if ( nom_delay < (p->Vdd_scalar*p->Vdd)/pow(((p->Vdd_scalar*p->Vdd)-(0.4+delta_vt)),1.1) + (p->Vdd_scalar*p->Vdd)/pow(((p->Vdd_scalar*p->Vdd)-0.5),1.1) )
	p->Vdd_scalar += 0.005;
#endif
	p->delta_vtp = delta_vt;
	return delta_vt;
}
*/

float gemu_pm_slp_power(void)
{
	esweek_pm *p = &pm_parameters.parameters;
	double t = p->T + 273;
	double v = p->Vdd;
#ifdef ADAPTIVE
	v *= p->Vdd_scalar;
#endif
	double power = p->Nl * v * t*t * ( exp(p->Al*p->Vnl/t) + exp(p->Al*(p->Vpl+p->delta_vtp)/t) ) * exp(p->Bl*v/t);
	//double power = p->sA*t*t/exp(p->sB/t) + p->sC;
	return (float)power/1000000;
	
}

uint64_t gemu_pm_slp_energy()
{
	uint64_t slp_time=getSlpTicks(); //In Ps
	uint64_t delta=getSlpTicks()-prev_slp; //In Ps
	double cur_slp_energy=gemu_pm_slp_power()*delta;
	prev_slp=slp_time;
	slp_energy+=cur_slp_energy;
	return (uint64_t)slp_energy;	

}

float gemu_pm_act_power(uint8_t class_num)
{
	esweek_pm *p = &pm_parameters.parameters;
	double v = p->Vdd;
	double f = SimClock::Frequency/1e9;
#ifdef ADAPTIVE
	v *= p->Vdd_scalar;
#endif
	double power = v*v + p->Bd * pow(( v - p->Cd - p->delta_vtp ),3);
	power = power * p->Nd * f;
	return (float)power + gemu_pm_slp_power();
}

uint64_t gemu_pm_act_energy(uint8_t class_num)
{
	uint64_t act_time=getActTicks(); //In Ps
	uint64_t delta=getActTicks()-prev_act; //In Ps
 	double cur_act_energy=gemu_pm_act_power(class_num) * delta;
	prev_act=act_time;
	act_energy+=cur_act_energy;
	return (uint64_t)act_energy; 
}

void gemu_pm_change_parameter(uint8_t class_num, uint8_t parameter, double value)
{
	assert(parameter < 13);
	//vemu_debug("p=%d, v=%f\n",parameter, value);
	pm_parameters.pm_array[parameter] = value;
}


double gemu_pm_get_parameter(uint8_t class_num, uint8_t parameter){
	assert(parameter < 13);
	return pm_parameters.pm_array[parameter];
}


/*
void vemu_pm_print_temp_p_curve(void)
{
	int ttt;
	double oldT = pm_parameters.pm_array[0];
	printf("T, Ps, Pa:\n");
	for (ttt = -20; ttt <= 100; ttt++) {
		pm_parameters.pm_array[0] = (double)ttt;
		printf("%d,%f,%f\n", ttt, vemu_pm_slp_power(), vemu_pm_act_power(0,vemu_frequency));
	}
	pm_parameters.pm_array[0] = oldT;
}
*/





AtomicSimpleCPU::TickEvent::TickEvent(AtomicSimpleCPU *c)
    : Event(CPU_Tick_Pri), cpu(c)
{
}


void
AtomicSimpleCPU::TickEvent::process()
{
    cpu->tick();
}

const char *
AtomicSimpleCPU::TickEvent::description() const
{
    return "AtomicSimpleCPU tick";


}

void
AtomicSimpleCPU::init()
{
    BaseCPU::init();

    // Initialise the ThreadContext's memory proxies
    tcBase()->initMemProxies(tcBase());

    if (FullSystem && !params()->switched_out) {
        ThreadID size = threadContexts.size();
        for (ThreadID i = 0; i < size; ++i) {
            ThreadContext *tc = threadContexts[i];
            // initialize CPU, including PC
            TheISA::initCPU(tc, tc->contextId());
        }
    }

    // Atomic doesn't do MT right now, so contextId == threadId
    ifetch_req.setThreadContext(_cpuId, 0); // Add thread ID if we add MT
    data_read_req.setThreadContext(_cpuId, 0); // Add thread ID here too
    data_write_req.setThreadContext(_cpuId, 0); // Add thread ID here too
}




AtomicSimpleCPU::AtomicSimpleCPU(AtomicSimpleCPUParams *p)
    : BaseSimpleCPU(p), tickEvent(this), width(p->width), locked(false),
      simulate_data_stalls(p->simulate_data_stalls),
      simulate_inst_stalls(p->simulate_inst_stalls),
      drain_manager(NULL),
      icachePort(name() + ".icache_port", this),
      dcachePort(name() + ".dcache_port", this),
      fastmem(p->fastmem),
      simpoint(p->simpoint_profile),
      intervalSize(p->simpoint_interval),
      intervalCount(0),
      intervalDrift(0),
      simpointStream(NULL),
      currentBBV(0, 0),
      currentBBVInstCount(0)
{
    _status = Idle;
    mode=0;

    if (simpoint) {
        simpointStream = simout.create(p->simpoint_profile_file, false);
    }
}


AtomicSimpleCPU::~AtomicSimpleCPU()
{
    if (tickEvent.scheduled()) {
        deschedule(tickEvent);
    }
    if (simpointStream) {
        simout.close(simpointStream);
    }
}

unsigned int
AtomicSimpleCPU::drain(DrainManager *dm)
{
    assert(!drain_manager);
    if (switchedOut())
        return 0;

    if (!isDrained()) {
        DPRINTF(Drain, "Requesting drain: %s\n", pcState());
        drain_manager = dm;
        return 1;
    } else {
        if (tickEvent.scheduled())
            deschedule(tickEvent);

        DPRINTF(Drain, "Not executing microcode, no need to drain.\n");
        return 0;
    }
}

void
AtomicSimpleCPU::drainResume()
{
    assert(!tickEvent.scheduled());
    assert(!drain_manager);
    if (switchedOut())
        return;

    DPRINTF(SimpleCPU, "Resume\n");
    verifyMemoryMode();

    assert(!threadContexts.empty());
    if (threadContexts.size() > 1)
        fatal("The atomic CPU only supports one thread.\n");

    if (thread->status() == ThreadContext::Active) {
        schedule(tickEvent, nextCycle());
	mode=1;
        _status = BaseSimpleCPU::Running;
        notIdleFraction = 1;
    } else {
	mode=0;
        _status = BaseSimpleCPU::Idle;
        notIdleFraction = 0;
    }

    system->totalNumInsts = 0;
}

bool
AtomicSimpleCPU::tryCompleteDrain()
{
    if (!drain_manager)
        return false;

    DPRINTF(Drain, "tryCompleteDrain: %s\n", pcState());
    if (!isDrained())
        return false;

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    drain_manager->signalDrainDone();
    drain_manager = NULL;

    return true;
}


void
AtomicSimpleCPU::switchOut()
{
    BaseSimpleCPU::switchOut();

    assert(!tickEvent.scheduled());
    assert(_status == BaseSimpleCPU::Running || _status == Idle);
    assert(isDrained());
}


void
AtomicSimpleCPU::takeOverFrom(BaseCPU *oldCPU)
{
    BaseSimpleCPU::takeOverFrom(oldCPU);

    // The tick event should have been descheduled by drain()
    assert(!tickEvent.scheduled());

    ifetch_req.setThreadContext(_cpuId, 0); // Add thread ID if we add MT
    data_read_req.setThreadContext(_cpuId, 0); // Add thread ID here too
    data_write_req.setThreadContext(_cpuId, 0); // Add thread ID here too
}

void
AtomicSimpleCPU::verifyMemoryMode() const
{
    if (!system->isAtomicMode()) {
        fatal("The atomic CPU requires the memory system to be in "
              "'atomic' mode.\n");
    }
}

void
AtomicSimpleCPU::activateContext(ThreadID thread_num, Cycles delay)
{

    DPRINTF(SimpleCPU, "ActivateContext %d (%d cycles)\n", thread_num, delay);

    assert(thread_num == 0);
    assert(thread);

    assert(_status == Idle);
    assert(!tickEvent.scheduled());

    notIdleFraction = 1;
    begCycles=numCycles.value();
    numCycles += ticksToCycles(thread->lastActivate - thread->lastSuspend);
    endCycles=numCycles.value();

    slp_tick += (thread->lastActivate - thread->lastSuspend);
    //slp_count += endCycles-begCycles;
    

    


    //printf("Idle cycles: %d\n",idleCycles);
    
    //Make sure ticks are still on multiples of cycles
    schedule(tickEvent, clockEdge(delay));
	
//Call slp energy because up to this point it has been in sleep, so it will accumulate all sleep energy up to this point. 
    gemu_pm_slp_energy();
    //1 means mode is Running
    mode=1;
    _status = BaseSimpleCPU::Running;
   // printf("Waking up at cycle number: %f\n", numCycles.value());
}


void
AtomicSimpleCPU::suspendContext(ThreadID thread_num)
{
    DPRINTF(SimpleCPU, "SuspendContext %d\n", thread_num);

    assert(thread_num == 0);
    assert(thread);

    if (_status == Idle)
        return;

    assert(_status == BaseSimpleCPU::Running);

    // tick event may not be scheduled if this gets called from inside
    // an instruction's execution, e.g. "quiesce"
    if (tickEvent.scheduled())
        deschedule(tickEvent);

    notIdleFraction = 0;
	
   //Call act energy because up to this point it has been active, so it will accumulate all active energy up to this point. 
    gemu_pm_act_energy(1);     

    //0 means mode is Idle
    mode=0;
    _status = Idle;
  //  printf("Going Idle at cycle number: %f\n", numCycles.value());
}


Fault
AtomicSimpleCPU::readMem(Addr addr, uint8_t * data,
                         unsigned size, unsigned flags)
{
    // use the CPU's statically allocated read request and packet objects
    Request *req = &data_read_req;

    if (traceData) {
        traceData->setAddr(addr);
    }

    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    if (secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    while (1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Read);

        // Now do the access.
        if (fault == NoFault && !req->getFlags().isSet(Request::NO_ACCESS)) {
            Packet pkt = Packet(req,
                                req->isLLSC() ? MemCmd::LoadLockedReq :
                                MemCmd::ReadReq);
            pkt.dataStatic(data);

            if (req->isMmappedIpr())
                dcache_latency += TheISA::handleIprRead(thread->getTC(), &pkt);
            else {
                if (fastmem && system->isMemAddr(pkt.getAddr()))
                    system->getPhysMem().access(&pkt);
                else
                    dcache_latency += dcachePort.sendAtomic(&pkt);
            }
            dcache_access = true;

            assert(!pkt.isError());

            if (req->isLLSC()) {
                TheISA::handleLockedRead(thread, req);
            }
        }

        //If there's a fault, return it
        if (fault != NoFault) {
            if (req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        //If we don't need to access a second cache line, stop now.
        if (secondAddr <= addr)
        {
            if (req->isLocked() && fault == NoFault) {
                assert(!locked);
                locked = true;
            }
            return fault;
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}


Fault
AtomicSimpleCPU::writeMem(uint8_t *data, unsigned size,
                          Addr addr, unsigned flags, uint64_t *res)
{
    // use the CPU's statically allocated write request and packet objects
    Request *req = &data_write_req;

    if (traceData) {
        traceData->setAddr(addr);
    }

    //The size of the data we're trying to read.
    int fullSize = size;

    //The address of the second part of this access if it needs to be split
    //across a cache line boundary.
    Addr secondAddr = roundDown(addr + size - 1, cacheLineSize());

    if(secondAddr > addr)
        size = secondAddr - addr;

    dcache_latency = 0;

    while(1) {
        req->setVirt(0, addr, size, flags, dataMasterId(), thread->pcState().instAddr());

        // translate to physical address
        Fault fault = thread->dtb->translateAtomic(req, tc, BaseTLB::Write);

        // Now do the access.
        if (fault == NoFault) {
            MemCmd cmd = MemCmd::WriteReq; // default
            bool do_access = true;  // flag to suppress cache access

            if (req->isLLSC()) {
                cmd = MemCmd::StoreCondReq;
                do_access = TheISA::handleLockedWrite(thread, req);
            } else if (req->isSwap()) {
                cmd = MemCmd::SwapReq;
                if (req->isCondSwap()) {
                    assert(res);
                    req->setExtraData(*res);
                }
            }

            if (do_access && !req->getFlags().isSet(Request::NO_ACCESS)) {
                Packet pkt = Packet(req, cmd);
                pkt.dataStatic(data);

                if (req->isMmappedIpr()) {
                    dcache_latency +=
                        TheISA::handleIprWrite(thread->getTC(), &pkt);
                } else {
                    if (fastmem && system->isMemAddr(pkt.getAddr()))
                        system->getPhysMem().access(&pkt);
                    else
                        dcache_latency += dcachePort.sendAtomic(&pkt);
                }
                dcache_access = true;
                assert(!pkt.isError());

                if (req->isSwap()) {
                    assert(res);
                    memcpy(res, pkt.getPtr<uint8_t>(), fullSize);
                }
            }

            if (res && !req->isSwap()) {
                *res = req->getExtraData();
            }
        }

        //If there's a fault or we don't need to access a second cache line,
        //stop now.
        if (fault != NoFault || secondAddr <= addr)
        {
            if (req->isLocked() && fault == NoFault) {
                assert(locked);
                locked = false;
            }
            if (fault != NoFault && req->isPrefetch()) {
                return NoFault;
            } else {
                return fault;
            }
        }

        /*
         * Set up for accessing the second cache line.
         */

        //Move the pointer we're reading into to the correct location.
        data += size;
        //Adjust the size to get the remaining bytes.
        size = addr + fullSize - secondAddr;
        //And access the right address.
        addr = secondAddr;
    }
}

uint64_t getCycleCount()
{
	return cycleCount;
}

uint64_t getSlpTicks()
{
	return slp_tick;
}

uint64_t getActTicks()
{
	return curTick()-slp_tick;
}


void
AtomicSimpleCPU::tick()
{   

    DPRINTF(SimpleCPU, "Tick\n");
    Tick latency = 0;
       
    for (int i = 0; i < width || locked; ++i) {
	if ((cycleCount % 1000000) == 0){
		parse();
	}        
	numCycles++;
	cycleCount=numCycles.value();
	
	/*if (cycleCount == 1){
		slp_tick = 100000000;
		setClockFrequency(slp_tick);
	}*/
	

        if (!curStaticInst || !curStaticInst->isDelayedCommit())
            checkForInterrupts();

        checkPcEventQueue();
        // We must have just got suspended by a PC event
        if (_status == Idle) {
            tryCompleteDrain();
            return;
        }

        Fault fault = NoFault;

        TheISA::PCState pcState = thread->pcState();

        bool needToFetch = !isRomMicroPC(pcState.microPC()) &&
                           !curMacroStaticInst;
	needToFetch=true;
        if (needToFetch) {
            setupFetchRequest(&ifetch_req);
            fault = thread->itb->translateAtomic(&ifetch_req, tc,
                                                 BaseTLB::Execute);
        }

        if (fault == NoFault) {
            Tick icache_latency = 0;
            bool icache_access = false;
            dcache_access = false; // assume no dcache access

            if (needToFetch) {
                // This is commented out because the decoder would act like
                // a tiny cache otherwise. It wouldn't be flushed when needed
                // like the I cache. It should be flushed, and when that works
                // this code should be uncommented.
                //Fetch more instruction memory if necessary
                //if(decoder.needMoreBytes())
                //{
                    icache_access = true;
                    Packet ifetch_pkt = Packet(&ifetch_req, MemCmd::ReadReq);
                    ifetch_pkt.dataStatic(&inst);

                    if (fastmem && system->isMemAddr(ifetch_pkt.getAddr()))
                        system->getPhysMem().access(&ifetch_pkt);
                    else
                        icache_latency = icachePort.sendAtomic(&ifetch_pkt);

                    assert(!ifetch_pkt.isError());

                    // ifetch_req is initialized to read the instruction directly
                    // into the CPU object's inst field.
                //}
            }

            preExecute();

            if (curStaticInst) {
                fault = curStaticInst->execute(this, traceData);

                // keep an instruction count
                if (fault == NoFault)
                    countInst();
                else if (traceData && !DTRACE(ExecFaulting)) {
                    delete traceData;
                    traceData = NULL;
                }

                postExecute();
            }

            // @todo remove me after debugging with legion done
            if (curStaticInst && (!curStaticInst->isMicroop() ||
                        curStaticInst->isFirstMicroop()))
                instCnt++;

            // profile for SimPoints if enabled and macro inst is finished
            if (simpoint && curStaticInst && (fault == NoFault) &&
                    (!curStaticInst->isMicroop() ||
                     curStaticInst->isLastMicroop())) {
                profileSimPoint();
            }

            Tick stall_ticks = 0;
            if (simulate_inst_stalls && icache_access)
                stall_ticks += icache_latency;

            if (simulate_data_stalls && dcache_access)
                stall_ticks += dcache_latency;

            if (stall_ticks) {
                // the atomic cpu does its accounting in ticks, so
                // keep counting in ticks but round to the clock
                // period
                latency += divCeil(stall_ticks, clockPeriod()) *
                    clockPeriod();
            }

        }
        if(fault != NoFault || !stayAtPC)
            advancePC(fault);
    }

    if (tryCompleteDrain())
        return;

    // instruction takes at least one cycle
    if (latency < clockPeriod())
        latency = clockPeriod();

    if (_status != Idle)
        schedule(tickEvent, curTick() + latency);
}


void
AtomicSimpleCPU::printAddr(Addr a)
{
    dcachePort.printAddr(a);
}

void
AtomicSimpleCPU::profileSimPoint()
{
    if (!currentBBVInstCount)
        currentBBV.first = thread->pcState().instAddr();

    ++intervalCount;
    ++currentBBVInstCount;

    // If inst is control inst, assume end of basic block.
    if (curStaticInst->isControl()) {
        currentBBV.second = thread->pcState().instAddr();

        auto map_itr = bbMap.find(currentBBV);
        if (map_itr == bbMap.end()){
            // If a new (previously unseen) basic block is found,
            // add a new unique id, record num of insts and insert into bbMap.
            BBInfo info;
            info.id = bbMap.size() + 1;
            info.insts = currentBBVInstCount;
            info.count = currentBBVInstCount;
            bbMap.insert(std::make_pair(currentBBV, info));
        } else {
            // If basic block is seen before, just increment the count by the
            // number of insts in basic block.
            BBInfo& info = map_itr->second;
            assert(info.insts == currentBBVInstCount);
            info.count += currentBBVInstCount;
        }
        currentBBVInstCount = 0;

        // Reached end of interval if the sum of the current inst count
        // (intervalCount) and the excessive inst count from the previous
        // interval (intervalDrift) is greater than/equal to the interval size.
        if (intervalCount + intervalDrift >= intervalSize) {
            // summarize interval and display BBV info
            std::vector<pair<uint64_t, uint64_t> > counts;
            for (auto map_itr = bbMap.begin(); map_itr != bbMap.end();
                    ++map_itr) {
                BBInfo& info = map_itr->second;
                if (info.count != 0) {
                    counts.push_back(std::make_pair(info.id, info.count));
                    info.count = 0;
                }
            }
            std::sort(counts.begin(), counts.end());

            // Print output BBV info
            *simpointStream << "T";
            for (auto cnt_itr = counts.begin(); cnt_itr != counts.end();
                    ++cnt_itr) {
                *simpointStream << ":" << cnt_itr->first
                                << ":" << cnt_itr->second << " ";
            }
            *simpointStream << "\n";

            intervalDrift = (intervalCount + intervalDrift) - intervalSize;
            intervalCount = 0;
        }
    }
}

////////////////////////////////////////////////////////////////////////
//
//  AtomicSimpleCPU Simulation Object
//
AtomicSimpleCPU *
AtomicSimpleCPUParams::create()
{
    numThreads = 1;
    if (!FullSystem && workload.size() != 1)
        panic("only one workload allowed");
    return new AtomicSimpleCPU(this);
}
