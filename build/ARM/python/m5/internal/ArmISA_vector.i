%module(package="m5.internal") ArmISA_vector
%{
#include "params/ArmISA.hh"
%}

%include "std_container.i"

%import "python/m5/internal/param_ArmISA.i"

%include "std_vector.i"

%typemap(in) std::vector< ArmISA::ISA * >::value_type {
    if (SWIG_ConvertPtr($input, (void **)&$1, $1_descriptor, 0) == -1) {
        if (SWIG_ConvertPtr($input, (void **)&$1,
                            $descriptor(ArmISA::ISA *), 0) == -1) {
            return NULL;
        }
    }
}

%typemap(in) std::vector< ArmISA::ISA * >::value_type * {
    if (SWIG_ConvertPtr($input, (void **)&$1, $1_descriptor, 0) == -1) {
        if (SWIG_ConvertPtr($input, (void **)&$1,
                            $descriptor(ArmISA::ISA * *), 0) == -1) {
            return NULL;
        }
    }
}
%template(vector_ArmISA) std::vector< ArmISA::ISA * >;
