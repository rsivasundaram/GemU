%module(package="m5.internal") Counter_vector
%{
#include "base/types.hh"
%}

%include "std_container.i"

%import "stdint.i"
%import "base/types.hh"

%include "std_vector.i"

%typemap(in) std::vector< Counter >::value_type {
    if (SWIG_ConvertPtr($input, (void **)&$1, $1_descriptor, 0) == -1) {
        if (SWIG_ConvertPtr($input, (void **)&$1,
                            $descriptor(Counter), 0) == -1) {
            return NULL;
        }
    }
}

%typemap(in) std::vector< Counter >::value_type * {
    if (SWIG_ConvertPtr($input, (void **)&$1, $1_descriptor, 0) == -1) {
        if (SWIG_ConvertPtr($input, (void **)&$1,
                            $descriptor(Counter *), 0) == -1) {
            return NULL;
        }
    }
}
%template(vector_Counter) std::vector< Counter >;
