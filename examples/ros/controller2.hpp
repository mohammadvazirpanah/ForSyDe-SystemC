#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>
#include <math.h>
#include <global.hpp>

using namespace sc_core;
using namespace ForSyDe;


void controller_ns_func(int& ns,
    const int& cs,
    const double& inp)
{

int move_around;
int state = cs; 
double destination_grad = 0;
double current_grad = 0;


/*
Initial_Angle_Changer : state = 1 
Move_Toward : state = 2 
Obstacle_Avoid : state = 3 
Move_Far : state = 4
Init_Move_Near : state = 5 
Move_Near : state = 6 
*/ 

switch (state)
{
    case 1:  
        if (fmod(abs(desired_angle-current_angle),2*pi) < rot_pos_margin)
        {
            state = 2;
        }   
        break;
        
    case 2:
        if (right_sonar < sonar_margin)
        {
            last_right_sonar = right_sonar;
            move_around = 0;
            double destination_grad = destination_y/destination_x;
            double current_grad = current_y/current_x;
            state = 3;
        }
        break;

    case 3:  
        destination_grad = destination_y/destination_x;
        current_grad = current_y/current_x;

        if (move_around==1 && abs(current_grad-destination_grad)<0.01)
            state = 1;
        if (0.2*sonar_margin<sonar_margin-right_sonar)
        {   
            last_right_sonar = right_sonar;
            state = 4;
        }

        if (right_sonar-sonar_margin>0.1*sonar_margin)
        {
            last_right_sonar = right_sonar;
            state = 5;
        }
        break;

    case 4:
        move_around = 1;
        last_right_sonar = right_sonar;
        if (right_sonar<last_right_sonar || right_sonar > 0.8* sonar_margin)
        {
            state = 3;
        }
        break;

    case 5:  
        move_around = 1;
        // after 600 Ms 
        state = 6;
        break;

    case 6:
        move_around = 1;
        last_right_sonar = right_sonar;
        if (right_sonar<last_right_sonar || right_sonar < sonar_margin || right_sonar == max_sonar)
        {
            state = 3;
        }  
        break;   

} 

ns = state;

}

void controller_od_func(std::tuple<double,double, double>& out,
    const int& cs,
    std::tuple<double,double, double>& inp)
{

double w_out = 0, x_out = 0, y_out = 0 ;
int state = cs;
switch (state)
{
    case 1:  
        w_out = rot_vel_gain*(desired_angle-current_angle)/abs(desired_angle-current_angle);
        x_out = 0;
        y_out = 0; 
        out = std::make_tuple(x_out, y_out, w_out);
        break;
    case 2:
        w_out = 0;
        x_out = -1*lin_vel_gain*cos(pi/3);
        y_out = -1*lin_vel_gain*cos(pi/6); 
        out = std::make_tuple(x_out, y_out, w_out);
        break;
    case 3:  
        w_out = 0;
        x_out = -1*lin_vel_gain*sin(pi/3);
        y_out = -1*lin_vel_gain*sin(pi/6);
        out = std::make_tuple(x_out, y_out, w_out);
        break;
    case 4:  
        w_out = rot_vel_gain*(-1);
        x_out = 0;
        y_out = 0; 
        out = std::make_tuple(x_out, y_out, w_out);
        break;
    case 5:  
        w_out = 0;
        x_out = -0.7*lin_vel_gain*cos(pi/6);
        y_out = -0.7*lin_vel_gain*cos(pi/3); 
        out = std::make_tuple(x_out, y_out, w_out);
        break;
    case 6:  
        w_out = rot_vel_gain*(1);
        x_out = 0;
        y_out = 0; 
        out = std::make_tuple(x_out, y_out, w_out);
        break;
    default:
        w_out = 0;
        x_out = 0;
        y_out = 0; 
        out = std::make_tuple(x_out, y_out, w_out);
} 

}

#endif