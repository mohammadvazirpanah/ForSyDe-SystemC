#include <forsyde.hpp>
// #include "comb_func.hpp"
#include <controller.hpp>

using namespace sc_core;
using namespace ForSyDe;

SC_MODULE(top)

{

// SY::signal <abst_ext<int>> to_mealy;

SY::signal <abst_ext<int>> from_mealy;
SY::SY2SY <int> to_mealy; 

    SC_CTOR(top)
    {   
        SY::make_constant ("constant",
                            abst_ext<int>(1),
                            1, 
                            to_mealy);
                            
        SY::make_smealy ("mealy",
                        next_state_func,
                        output_decode_func,
                        1,
                        from_mealy,
                        to_mealy
                        );

        SY::make_ssink ("sink", [=](const abst_ext<int>& a){std::cout<< from_abst_ext(a,0)<< std::endl;}, from_mealy);
    }
};