GemU
====

An edited version of Gem5 with DVS, Fault-injection, and Power modeling

**Currently only supports ARM with the Atomic CPU**

To build:```scons build/ARM/gem5.opt```

For more information about the different build versions, visit: http://www.m5sim.org/Build_System

Either create your own disk image or download the one provided by Gem5. 

Information for creating your own: http://www.m5sim.org/Disk_images

To download: http://www.m5sim.org/Download

**Important:**
Once you have a disk image, mount it (see https://www.youtube.com/watch?v=OXH1oxQbuHA) and create a directory called gem5 in the home directory (/home/gem5). Then, copy the device driver located at kernel/myDevice.ko into this directory. 

  If you would like to specify your own location for this device driver, please go to configs/boot/temp.rcS and edit line 10.
  
  Also, if you would like to utilize fault injection, copy the files located in the faults directory into whichever directory the program you are running is in. Make sure to ```#include "gemu_faults.hh"``` and compile the program with ```gemu_faults.cc```.

To run:```build/ARM/gem5.opt configs/example/fs.py --disk-image=/path/to/image --kernel=$(pwd)/kernel/vmlinux --mem-size=256MB --script=configs/boot/temp.rcS```

To connect to the simualtion, see: http://www.m5sim.org/M5term

**Power Modeling:**
Once the simulation is up and running, echo any of the following commands to /dev/chdev (case-sensitive):

"sleep power" (gives resting power)

"active power" (gives power consumption)

"sleep energy" (gives resting energy)

"active energy" (gives energy consumption) 

"cycles" (gives the number of CPU cycles completed)

Ex: ```echo "sleep power" > /dev/chdev```

To recieve the value of the command you input, enter ```cat /dev/chdev``` 

**Fault Injection:**
To see an example of a program utilizing the fault injection, look at faults/test.cc

**Port Functionality**
To run the Gem5 process as a server, specify a port number using the --port-number parameter. If no parameter is specified, it will run normally. To connect the sample client program to it, run client.py and specify the same port-number as a parameter. Here in the client.py file, you can specify how long you want gem5 to take before it processes the next cpu cycle (how much time it sleeps before sending the next byte).

For additional information about GemU, check out: 
  https://drive.google.com/file/d/0B1fHBX8DZ3PZV0pmbXVmMjQzUVk/view?usp=sharing
