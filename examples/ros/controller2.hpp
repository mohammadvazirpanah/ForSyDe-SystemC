#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>
#include <math.h>
#include <func.hpp>

using namespace sc_core;
using namespace ForSyDe;

double rot_pos_margin = 0.01;
double rot_vel_gain = 2;
double lin_vel_gain = 4;
double w_out = 0 , x_out=0 , y_out=0 ;
double sonar_margin = 0.7;
double max_sonar = 1.4;
double desired_angle = func(-10,5.7735026919) ; // destination_x , destination_y 
double current_angle = 5;
int state = 1; 
int move_around;
void controller_ns_func(std::tuple<double,double, double>& ns,
    const std::tuple<double,double, double>& cs,
    const double& inp)
{

 

//-------------------- Initial_Angle_Changer -------------------- state = 1 

// w_out = rot_vel_gain*(desired_angle-current_angle)/abs(desired_angle-current_angle);
// x_out = 0;
// y_out = 0; 

if (fmod(abs(desired_angle-current_angle),2*pi) < rot_pos_margin)
{
    state = 2; 
}


//-------------------- Move_Toward -------------------- state = 2 
if (state == 2 )
{
    w_out = 0;
    x_out = -1*lin_vel_gain*cos(pi/3);
    y_out = -1*lin_vel_gain*cos(pi/6); 

    if (right_sonar < sonar_margin)
    {
        last_right_sonar = right_sonar;
        move_around = 0;
        double destination_grad = destination_y/destination_x;
        double current_grad = current_y/current_x;
        state = 3;
    }
}

//-------------------- Obstacle_Avoid --------------------state = 3 
else if (state == 3 )
{
    w_out = 0;
    x_out = -1*lin_vel_gain*sin(pi/3);
    y_out = -1*lin_vel_gain*sin(pi/6);
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
}


//-------------------- Move_Far -------------------- state = 4
else if (state == 4)
{
    w_out = rot_vel_gain*(-1);
    x_out = 0;
    y_out = 0; 
    move_around = 1;
    last_right_sonar = right_sonar;

    if (right_sonar<last_right_sonar || right_sonar > 0.8* sonar_margin)
        state = 3;
    
}

//-------------------- Init_Move_Near -------------------- state = 5 
else if (state == 5)
{
    w_out = 0;
    x_out = -0.7*lin_vel_gain*cos(pi/6);
    y_out = -0.7*lin_vel_gain*cos(pi/3); 
    move_around = 1;

    // after 600 Ms 
    state = 6;

}


//-------------------- Move_Near -------------------- state = 6 
else if (state == 6)
{
    w_out = rot_vel_gain*(1);
    x_out = 0;
    y_out = 0; 
    move_around = 1;
    last_right_sonar = right_sonar;

    if (right_sonar<last_right_sonar || right_sonar < sonar_margin || right_sonar == max_sonar)
        state = 3;

}



}


void controller_od_func(std::tuple<double,double, double>& out,
    const std::tuple<double,double, double>& cs,
    const double& inp)
{
   
switch (state)
{
    case 1:  
        w_out = rot_vel_gain*(desired_angle-current_angle)/abs(desired_angle-current_angle);
        x_out = 0;
        y_out = 0; 
        out = std::make_tuple(x_out, y_out, w_out);
    case 2:
        w_out = 0;
        x_out = -1*lin_vel_gain*cos(pi/3);
        y_out = -1*lin_vel_gain*cos(pi/6); 
        out = std::make_tuple(x_out, y_out, w_out);
    case 3:  
        w_out = 0;
        x_out = -1*lin_vel_gain*sin(pi/3);
        y_out = -1*lin_vel_gain*sin(pi/6);
        out = std::make_tuple(x_out, y_out, w_out);
    case 4:  
        w_out = rot_vel_gain*(-1);
        x_out = 0;
        y_out = 0; 
        out = std::make_tuple(x_out, y_out, w_out);
    case 5:  
        w_out = 0;
        x_out = -0.7*lin_vel_gain*cos(pi/6);
        y_out = -0.7*lin_vel_gain*cos(pi/3); 
        out = std::make_tuple(x_out, y_out, w_out);
    case 6:  
        w_out = rot_vel_gain*(1);
        x_out = 0;
        y_out = 0; 
        out = std::make_tuple(x_out, y_out, w_out);
    default:
        w_out = 0;
        x_out = 0;
        y_out = 0; 
        out = std::make_tuple(x_out, y_out, w_out);
} 

}

#endif