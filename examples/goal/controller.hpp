#ifndef CONTROLLE_HPP
#define CONTROLLE_HPP

#include <forsyde.hpp>

using namespace sc_core;
using namespace ForSyDe;




void next_state_func(int& ns,
                    const int& cs,
                    const int& inp)
{
    int current_state = cs;
    int input = inp;
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

void output_decode_func(int& out,
                        const int& cs,
                        const int& inp)
{

    int current_state = cs;
    int input = inp;
    int rate = inp;

    if (current_state == 1)
    {
        if (input<10)
        {
            rate++;
            // set_val(out, rate);
            out = rate;

        }
        if (input>=10)
        {
            rate--;
            // set_val(out, rate);
            out = rate;


        }
    }
    else if (current_state == 2)
    {
        if (input<10)
        {
            rate--;
            // set_val(out, rate);
            out = rate;



        }
    }

}



#endif