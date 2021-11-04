#ifndef GLOBAL_HPP
#define GLOBAL_HPP

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


const double rot_pos_margin = 0.01;
const double rot_vel_gain = 2;
const double lin_vel_gain = 4;
const double sonar_margin = 0.7;
const double max_sonar = 1.4;
const double desired_angle = func(-10,5.7735026919) ; // destination_x , destination_y 
const double destination_x = -10;
const double destination_y = 5.7735026919;
const double current_angle = 5;



#endif
