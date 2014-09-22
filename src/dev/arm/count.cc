/*
 * Author: Ramanan Sivasundaram
 */

#include "base/trace.hh"
#include "debug/Checkpoint.hh"
#include "debug/myDevice.hh"
#include "dev/arm/amba_device.hh"
#include "dev/arm/base_gic.hh"
#include "dev/arm/count.hh"
#include "dev/terminal.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/sim_exit.hh"
#include "cpu/simple/atomic.hh"

uint64_t data = 4;
uint64_t energy_slp = 0, energy_act = 0;
float power_slp=0, power_act;
uint32_t low=0;
uint32_t hi=1;
count::count(const Params *p)
    : myDevice(p, 0xfff)
{

}



Tick
count::read(PacketPtr pkt)
{

	union {
		uint32_t int_power;
		float power;
	};

 Addr daddr = pkt->getAddr() - pioAddr;
 assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
 pkt->allocate();
 
 if (daddr==0){
	power_act=gemu_pm_slp_power();
	int_power=power_act*1000000;
	printf("The sleep power output is: %g\n",power_act);
	pkt->set<uint32_t>(int_power);
 }else if (daddr==4){
	energy_slp=gemu_pm_slp_energy();
	hi=(uint32_t)((energy_slp & 0xFFFFFFFF0000000) >> 32);
 	low = (uint32_t) (energy_slp & 0xFFFFFFFF);
	pkt->set<uint32_t>(low);
	//pkt->set<uint32_t>(hi);
 }else if (daddr==8){
	pkt->set<uint32_t>(hi);
 }else if (daddr==12){
	pkt->set<uint32_t>(getCycleCount());
 }else if (daddr==20){
	power_slp=gemu_pm_act_power(1);
	int_power=power_slp*1000000;
	printf("The active power output is: %g\n",power_slp);
	pkt->set<uint32_t>(int_power);
 }else if (daddr==24){
	energy_act=gemu_pm_act_energy(1);
	hi=(uint32_t)((energy_act & 0xFFFFFFFF0000000) >> 32);
 	low = (uint32_t) (energy_act & 0xFFFFFFFF);
	pkt->set<uint32_t>(low);
	//pkt->set<uint32_t>(hi);
 }else if (daddr==28){
	pkt->set<uint32_t>(hi);
 }

 


//pkt->set<uint64_t>(data);
/*
switch(pkt->getSize()) {
      case 1:
        pkt->set<uint8_t>(data);
        break;
      case 2:
        pkt->set<uint16_t>(data);
        break;
      case 4:
        pkt->set<uint32_t>(data);
        break;
      default:
        panic("Data retrieved from myDevice is too large (more than 32-bits)\n");
        break;
    }
*/

 pkt->makeAtomicResponse();
 warn("Device %s: Read request at offset %#X",devname, daddr);
 return pioDelay;
}

Tick
count::write(PacketPtr pkt)
{
	float x;
	assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);
 	Addr daddr = pkt->getAddr() - pioAddr;
	hi=pkt->get<uint32_t>();
	if (daddr == 16){
		if (hi>512){
			disableFaults(hi);
		}else{	
			enableFaults(hi);		
		}
 	}else{
		x=*(float *)(&hi);
		warn("The power value is: %g\n", x);
	}
	
//Used to set endianess
/*
 switch(pkt->getSize()) {
      case 1:
        data = pkt->get<uint8_t>();
        break;
      case 2:
        data = pkt->get<uint16_t>();
        break;
      case 4:
        data = pkt->get<uint32_t>();
        break;
      default:
        panic("Data written to myDevice is too large (more than 32-bits)\n");
        break;
    }
*/
	
 //*value=data;
	
	
 pkt->makeAtomicResponse();
 warn("Device %s: Write request at offset %#X, value:%#X",devname, daddr, data);
 return pioDelay;
}

AddrRangeList
count::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

count *
countParams::create()
{
    return new count(this);
}
