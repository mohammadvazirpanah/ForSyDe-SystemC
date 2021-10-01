#ifndef FUNC_HPP
#define FUNC_HPP

#include <forsyde.hpp>
#include <math.h>
#define pi 3.14159265359
using namespace ForSyDe;


float func(float inp1, float inp2) 
{
// inp1 = destination_x & inp2 = destination_y & out1 = destination_angle 


    float raw_angle = atan (inp1/inp2);
    float out1 = raw_angle;

    if (raw_angle >=0 && (inp1 <0 || inp2 < 0)) 
        out1 = raw_angle + pi;

    else if (raw_angle < 0 && (inp2 < 0 || inp1 > 0))
        out1 = raw_angle + pi;
    
    out1 = fmod(pi + out1, 2*pi);

    if(abs(out1 - 2*pi) < abs(out1))
        out1 = abs(out1 - 2*pi);

    if(abs(out1 + 2*pi) < abs(out1))
        out1 = abs(out1 + 2*pi);

    return out1;
}

#endif

