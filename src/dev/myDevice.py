from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice

class myDevice(BasicPioDevice):
	type= 'myDevice'
	cxx_header = "dev/myDevice.hh"
	devicename=Param.String("My device is being accessed")
	platform = Param.Platform(Parent.any, "Platform this device is part of.")
        terminal = Param.Terminal(Parent.any, "The terminal")
