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
            fp, pathname, description = imp.find_module('_core', [dirname(__file__)])
        except ImportError:
            import _core
            return _core
        if fp is not None:
            try:
                _mod = imp.load_module('_core', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _core = swig_import_helper()
    del swig_import_helper
else:
    import _core
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


class Cycles(object):
    thisown = _swig_property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc='The membership flag')
    __repr__ = _swig_repr
    def __init__(self, *args): 
        this = _core.new_Cycles(*args)
        try: self.this.append(this)
        except: self.this = this
    __swig_destroy__ = _core.delete_Cycles
    __del__ = lambda self : None;
Cycles_swigregister = _core.Cycles_swigregister
Cycles_swigregister(Cycles)
cvar = _core.cvar
MaxTick = cvar.MaxTick


def romMicroPC(*args):
  return _core.romMicroPC(*args)
romMicroPC = _core.romMicroPC

def normalMicroPC(*args):
  return _core.normalMicroPC(*args)
normalMicroPC = _core.normalMicroPC

def isRomMicroPC(*args):
  return _core.isRomMicroPC(*args)
isRomMicroPC = _core.isRomMicroPC

def setOutputDir(*args):
  return _core.setOutputDir(*args)
setOutputDir = _core.setOutputDir

def doExitCleanup():
  return _core.doExitCleanup()
doExitCleanup = _core.doExitCleanup

def disableAllListeners():
  return _core.disableAllListeners()
disableAllListeners = _core.disableAllListeners

def seedRandom(*args):
  return _core.seedRandom(*args)
seedRandom = _core.seedRandom

def setClockFrequency(*args):
  return _core.setClockFrequency(*args)
setClockFrequency = _core.setClockFrequency

def curTick():
  return _core.curTick()
curTick = _core.curTick

def serializeAll(*args):
  return _core.serializeAll(*args)
serializeAll = _core.serializeAll

def getCheckpoint(*args):
  return _core.getCheckpoint(*args)
getCheckpoint = _core.getCheckpoint

def unserializeGlobals(*args):
  return _core.unserializeGlobals(*args)
unserializeGlobals = _core.unserializeGlobals

MicroPCRomBit = cvar.MicroPCRomBit
MaxAddr = cvar.MaxAddr
InvalidThreadID = cvar.InvalidThreadID
InvalidPortID = cvar.InvalidPortID
compileDate = cvar.compileDate
flag_DEBUG = cvar.flag_DEBUG
flag_NDEBUG = cvar.flag_NDEBUG
flag_TRACING_ON = cvar.flag_TRACING_ON
