#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <forsyde.hpp>
#include <math.h>
#define pi 3.14159265359
using namespace ForSyDe;


enum MealyState {Initial_Angle_Changer, 
                 Move_Toward, 
                 Obstacle_Avoid, 
                 Move_Far, 
                 Init_Move_Near, 
                 Move_Near};



float func(float inp1, float inp2) 
{

/*  inp1 = destination_x  
    inp2 = destination_y  
    out = destination_angle
*/ 

    float raw_angle = atan (inp1/inp2);
    float out = raw_angle;

    if (raw_angle >=0 && (inp1 <0 || inp2 < 0)) 
        out = raw_angle + pi;

    else if (raw_angle < 0 && (inp2 < 0 || inp1 > 0))
        out = raw_angle + pi;
    
    out = fmod(pi + out, 2*pi);

    if(abs(out - 2*pi) < abs(out))
        out = abs(out - 2*pi);

    if(abs(out + 2*pi) < abs(out))
        out = abs(out + 2*pi);

    return out;
}
/*
Mealy Controller Type 
*/
typedef std::tuple<double,double,double,int,MealyState> next_state;
typedef std::tuple<double,double,double,int,MealyState> current_state;
typedef std::array<abst_ext<double>,6> input;
typedef std::array<abst_ext<double>,3> output;

/*
Mealy Transform Type 
*/

typedef std::array<abst_ext<double>,4> next_state_transform;
typedef std::array<abst_ext<double>,4> current_state_transform, initial_state_transform;
typedef std::array<abst_ext<double>,3> input_transform;
typedef std::array<abst_ext<double>,3> output_transform;
typedef std::numeric_limits<double> NaN;

const double rot_pos_margin = 0.01;
const double rot_vel_gain = 2.0;
const double lin_vel_gain = 4.0;
const double sonar_margin = 0.7;
const double max_sonar = 1.4;
const double destination_x = -10.0;
const double destination_y = 5.7735026919;
const double desired_angle = func (destination_x, destination_y); 

 




#endif
