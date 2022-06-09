#include <forsyde.hpp>
#include "comb_func.hpp"

using namespace sc_core;
using namespace ForSyDe;

SC_MODULE(top)

{

    UT::signal <float> src; 
    UT::signal <float> out;   

    std::vector<float> s1 = {36.7, 36.8, 36.7, 36.8, 36.9, 36.9, 37.0, 37.0, 37.1, 37.2, 37.3, 37.2, 37.3, 37.3, 37.4, 37.5, 37.6, 36.6};


    SC_CTOR(top)
    {
        std::cout<<s1;
        UT::make_vsource ("source",s1, src);
        UT::make_comb ("comb1", comb_func, 3, out, src);
        UT::make_sink ("sink", [=](const float& a){std::cout<<a<<std::endl;}, out);
    }
};