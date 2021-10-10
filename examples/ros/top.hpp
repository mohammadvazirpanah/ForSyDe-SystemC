#include <forsyde.hpp>
#include "forsyde/ros_wrapper.hpp"
#include <iostream>

using namespace sc_core;
using namespace ForSyDe;

SC_MODULE(top)
{
   //SY::signal<double> srca, srcb, outa, outb, outc, outd;
    SY::signal<float> srca, srcb, outa, outb, outc, outd;
    SC_CTOR(top)
    {
        SY::make_roswrap("roswrap","ForsydeToRos","RosToForsyde", outa, srca); 
        SY::make_delay("delay1", abst_ext<float>(0.0), srca, outa);
    }

};


