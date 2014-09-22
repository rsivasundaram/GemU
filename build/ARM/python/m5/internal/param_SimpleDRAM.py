# This file was automatically generated by SWIG (http://www.swig.org).
# Version 2.0.11
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2,6,0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_param_SimpleDRAM', [dirname(__file__)])
        except ImportError:
            import _param_SimpleDRAM
            return _param_SimpleDRAM
        if fp is not None:
            try:
                _mod = imp.load_module('_param_SimpleDRAM', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _param_SimpleDRAM = swig_import_helper()
    del swig_import_helper
else:
    import _param_SimpleDRAM
del version_info
try:
    _swig_property = property
except NameError:
    pass # Python < 2.2 doesn't have 'property'.
def _swig_setattr_nondynamic(self,class_type,name,value,static=1):
    if (name == "thisown"): return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name,None)
    if method: return method(self,value)
    if (not static):
        self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)

def _swig_setattr(self,class_type,name,value):
    return _swig_setattr_nondynamic(self,class_type,name,value,0)

def _swig_getattr(self,class_type,name):
    if (name == "thisown"): return self.this.own()
    method = class_type.__swig_getmethods__.get(name,None)
    if method: return method(self)
    raise AttributeError(name)

def _swig_repr(self):
    try: strthis = "proxy of " + self.this.__repr__()
    except: strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object : pass
    _newclass = 0


def _swig_setattr_nondynamic_method(set):
    def set_attr(self,name,value):
        if (name == "thisown"): return self.this.own(value)
        if hasattr(self,name) or (name == "this"):
            set(self,name,value)
        else:
            raise AttributeError("You cannot add attributes to %s" % self)
    return set_attr


import m5.internal.enum_AddrMap
import m5.internal.enum_PageManage
import m5.internal.enum_MemSched
import m5.internal.param_AbstractMemory
import m5.internal.param_MemObject
import m5.internal.param_ClockedObject
import m5.internal.param_ClockDomain
import m5.internal.param_SimObject
import m5.internal.drain
import m5.internal.serialize
class SimpleDRAM(m5.internal.param_AbstractMemory.AbstractMemory):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    def __init__(self, *args, **kwargs): raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
SimpleDRAM_swigregister = _param_SimpleDRAM.SimpleDRAM_swigregister
SimpleDRAM_swigregister(SimpleDRAM)

class SimpleDRAMParams(m5.internal.param_AbstractMemory.AbstractMemoryParams):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def create(self): return _param_SimpleDRAM.SimpleDRAMParams_create(self)
    tRFC = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tRFC_get, _param_SimpleDRAM.SimpleDRAMParams_tRFC_set)
    activation_limit = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_activation_limit_get, _param_SimpleDRAM.SimpleDRAMParams_activation_limit_set)
    tWTR = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tWTR_get, _param_SimpleDRAM.SimpleDRAMParams_tWTR_set)
    write_low_thresh_perc = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_write_low_thresh_perc_get, _param_SimpleDRAM.SimpleDRAMParams_write_low_thresh_perc_set)
    channels = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_channels_get, _param_SimpleDRAM.SimpleDRAMParams_channels_set)
    device_bus_width = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_device_bus_width_get, _param_SimpleDRAM.SimpleDRAMParams_device_bus_width_set)
    write_high_thresh_perc = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_write_high_thresh_perc_get, _param_SimpleDRAM.SimpleDRAMParams_write_high_thresh_perc_set)
    tRRD = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tRRD_get, _param_SimpleDRAM.SimpleDRAMParams_tRRD_set)
    burst_length = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_burst_length_get, _param_SimpleDRAM.SimpleDRAMParams_burst_length_set)
    banks_per_rank = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_banks_per_rank_get, _param_SimpleDRAM.SimpleDRAMParams_banks_per_rank_set)
    static_backend_latency = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_static_backend_latency_get, _param_SimpleDRAM.SimpleDRAMParams_static_backend_latency_set)
    tXAW = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tXAW_get, _param_SimpleDRAM.SimpleDRAMParams_tXAW_set)
    addr_mapping = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_addr_mapping_get, _param_SimpleDRAM.SimpleDRAMParams_addr_mapping_set)
    tRCD = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tRCD_get, _param_SimpleDRAM.SimpleDRAMParams_tRCD_set)
    write_buffer_size = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_write_buffer_size_get, _param_SimpleDRAM.SimpleDRAMParams_write_buffer_size_set)
    ranks_per_channel = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_ranks_per_channel_get, _param_SimpleDRAM.SimpleDRAMParams_ranks_per_channel_set)
    page_policy = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_page_policy_get, _param_SimpleDRAM.SimpleDRAMParams_page_policy_set)
    tCL = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tCL_get, _param_SimpleDRAM.SimpleDRAMParams_tCL_set)
    read_buffer_size = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_read_buffer_size_get, _param_SimpleDRAM.SimpleDRAMParams_read_buffer_size_set)
    tRAS = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tRAS_get, _param_SimpleDRAM.SimpleDRAMParams_tRAS_set)
    tBURST = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tBURST_get, _param_SimpleDRAM.SimpleDRAMParams_tBURST_set)
    static_frontend_latency = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_static_frontend_latency_get, _param_SimpleDRAM.SimpleDRAMParams_static_frontend_latency_set)
    devices_per_rank = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_devices_per_rank_get, _param_SimpleDRAM.SimpleDRAMParams_devices_per_rank_set)
    tREFI = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tREFI_get, _param_SimpleDRAM.SimpleDRAMParams_tREFI_set)
    mem_sched_policy = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_mem_sched_policy_get, _param_SimpleDRAM.SimpleDRAMParams_mem_sched_policy_set)
    tRP = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_tRP_get, _param_SimpleDRAM.SimpleDRAMParams_tRP_set)
    device_rowbuffer_size = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_device_rowbuffer_size_get, _param_SimpleDRAM.SimpleDRAMParams_device_rowbuffer_size_set)
    port_port_connection_count = _swig_property(_param_SimpleDRAM.SimpleDRAMParams_port_port_connection_count_get, _param_SimpleDRAM.SimpleDRAMParams_port_port_connection_count_set)
    def __init__(self): 
        this = _param_SimpleDRAM.new_SimpleDRAMParams()
        try: self.this.append(this)
        except: self.this = this
    __swig_destroy__ = _param_SimpleDRAM.delete_SimpleDRAMParams
    __del__ = lambda self : None;
SimpleDRAMParams_swigregister = _param_SimpleDRAM.SimpleDRAMParams_swigregister
SimpleDRAMParams_swigregister(SimpleDRAMParams)


