#ifndef CONTROLLE_HPP
#define CONTROLLE_HPP

#include <forsyde.hpp>

using namespace sc_core;
using namespace ForSyDe;




void next_state_func(int& ns,
                    const int& cs,
                    const abst_ext<int>& inp)
{
    int current_state = cs;
    int input = from_abst_ext(inp,0);
    if (current_state == 1)
    {
        if (input<10)
        {
            ns = 1;

        }
        else if (input>=10)
        {
            ns = 2;
        }
    }
    else if (current_state == 2)
    {
        if (input<10)
        {
            ns = 2;

        }
        else if (input>=11)
        {
            ns = 3;
        }
    }

}

void output_decode_func(abst_ext<int>& out,
                        const int& cs,
                        const abst_ext<int>& inp)
{

    int current_state = cs;
    int input = from_abst_ext(inp,0);
    int rate = from_abst_ext(inp,0); 

    if (current_state == 1)
    {
        if (input<10)
        {
            rate++;
            set_val(out, rate);

        }
        if (input>=10)
        {
            rate--;
            set_val(out, rate);

        }
    }
    else if (current_state == 2)
    {
        if (input<10)
        {
            rate--;
            set_val(out, rate);


        }
    }

}



#endif