GemU
====

An edited version of Gem5 with DVS, Fault-injection, and Power modeling

To build:```scons build/ARM/gem5.opt```

For more information about the different build versions, visit: http://www.m5sim.org/Build_System

Either create your own disk image or download the one provided by Gem5. 

Information for creating your own: http://www.m5sim.org/Disk_images
To download: http://www.m5sim.org/Download

**Important:**
Once you have a disk image, mount it (see https://www.youtube.com/watch?v=OXH1oxQbuHA) and create a directory called gem5 in the home directory (/home/gem5). Then, move the device driver located at kernel/myDevice.ko into this directory. 

  If you would like to specify your own location for this device driver, please go to configs/boot/temp.rcS and edit line 10.

To run:```build/ARM/gem5.opt configs/example/fs.py --disk-image=/path/to/image --kernel=kernel/vmlinux --mem-size=256MB --script=configs/boot/temp.rcS```

To connect to the simualtion, see: http://www.m5sim.org/M5term

