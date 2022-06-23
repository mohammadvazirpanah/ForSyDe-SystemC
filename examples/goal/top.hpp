#include <forsyde.hpp>
#include <controller.hpp>

using namespace sc_core;
using namespace ForSyDe;

bool func_goal(void*)
{
    std::cout << "func_goal" << std::endl;
    return true;
}


SC_MODULE(top)

{

    SY::signal <int> from_mealy;
    SY::SY2SY <int> to_mealy; 

    SC_CTOR(top)
    {
        SY::make_sconstant ("constant",
                            1,
                            10, 
                            to_mealy);

        SY::make_smealy ("mealy",
                        next_state_func,
                        output_decode_func,
                        1,
                        func_goal,
                        from_mealy,
                        to_mealy
                        );

        /* Without Helper Function 
        auto mealy = new SY::smealy<int,int,int>
                        ("mealy",
                        next_state_func,
                        output_decode_func,
                        1,
                        func_goal);

        mealy->iport1(to_mealy);
        mealy->oport1(from_mealy);
        */

        SY::make_ssink ("sink", [=](const int& a){std::cout<< a << std::endl;}, from_mealy);
    }
};