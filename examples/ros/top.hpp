

#include "report.hpp"
#include "forsyde/ros_wrapper.hpp"
#include <iostream>
#include <forsyde.hpp>
#include "func.hpp"
#include "controller.hpp"


using namespace sc_core;
using namespace ForSyDe;

SC_MODULE(top)
{
    SY::signal<double> srca, result;
    
    SC_CTOR(top)
    {
    
        std::cout<<"SC_CTOR Start \n";

        
        //SY::make_ssource("source1", func, 1, 2, srca);
        //SY::roswrap<std::string,std::string> *ros1 = new SY::roswrap<std::string,std::string>("ros1","sender",result, srca);

        SY::make_roswrap("roswrap","ForsydeToRos","RosToForsyde", result, srca); // (Module Name, Sender Topic, Recevier Topic, out , in)
        
        SY::make_smealy("controller1",
                controller_ns_func,
                controller_od_func,
                std::make_tuple(0.0, 0.0, 0.0),
                srca,
                result
            );


        //SY::make_ssink("report1", report_func, result);
        
        

    }

};



