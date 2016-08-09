%module(package="m5.internal") param_TrafficGen

%{
#include "sim/sim_object.hh"
#include "params/TrafficGen.hh"
#include <cstddef>
#include <string>
#include <cstddef>
#include <cstddef>
#include "params/System.hh"
#include "cpu/testers/traffic_gen/traffic_gen.hh"
/**
  * This is a workaround for bug in swig. Prior to gcc 4.6.1 the STL
  * headers like vector, string, etc. used to automatically pull in
  * the cstddef header but starting with gcc 4.6.1 they no longer do.
  * This leads to swig generated a file that does not compile so we
  * explicitly include cstddef. Additionally, including version 2.0.4,
  * swig uses ptrdiff_t without the std:: namespace prefix which is
  * required with gcc 4.6.1. We explicitly provide access to it.
  */
#include <cstddef>
using std::ptrdiff_t;
%}

%include "std_string.i"
%import "python/m5/internal/param_System.i"

%import "python/m5/internal/param_MemObject.i"


// stop swig from creating/wrapping default ctor/dtor
%nodefault TrafficGen;
class TrafficGen
    : public MemObject
{
  public:
};

%include "params/TrafficGen.hh"
