#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>
#include <math.h>
#include <global.hpp>

using namespace sc_core;
using namespace ForSyDe;



void controller_ns_func(std::tuple<double,double,double,int,int>& ns,
                        const std::tuple<double,double,double,int,int>& cs,
                        const std::array<abst_ext<double>,6>& inp)
{

    
    double destination_grad, current_grad, last_right_sonar; 
    int move_around, state; 
    std::tie(destination_grad, current_grad, last_right_sonar, move_around, state) = cs;

    double current_y     = from_abst_ext(inp[0],0.0);
    double current_x     = from_abst_ext(inp[1],0.0);
    double current_angle = from_abst_ext(inp[2],0.0);
    double left2_sonar   = from_abst_ext(inp[3],0.0);
    double right_sonar   = from_abst_ext(inp[4],0.0);
    double left_sonar    = from_abst_ext(inp[5],0.0);
    // destination_grad = destination_y/destination_x;
    // current_grad = current_y/current_x;
    // std::cout<<"current_y="<<current_y<<"\t";
    // std::cout<<"current_x="<<current_x<<"\t";
    // std::cout<<"current_angle="<<current_angle<<"\n";
    // std::cout<<"left2_sonar="<<left2_sonar<<"\t";
    // std::cout<<"left_sonar="<<left_sonar<<"\t";
    // std::cout<<"right_sonar="<<right_sonar<<"\n";
    




/*
    Initial_Angle_Changer : state = 0 
    Move_Toward : state = 1
    Obstacle_Avoid : state = 2
    Move_Far : state = 3
    Init_Move_Near : state = 4
    Move_Near : state = 5
    */ 

    switch (state)
    {   

        case 0:  
            
            if (remainderf(abs(desired_angle-current_angle),2*pi) < rot_pos_margin)
                state = 1;
            ns = std::make_tuple(destination_grad, current_grad, last_right_sonar, move_around, state);
            break;
        case 1:
            if (right_sonar < sonar_margin)
            {
                last_right_sonar = right_sonar;
                move_around = 0;
                destination_grad = destination_y/destination_x;
                current_grad = current_y/current_x;
                state = 2;
            }
            ns = std::make_tuple(destination_grad, current_grad, last_right_sonar, move_around, state);
            break;

        case 2:  
            if ((move_around==1) && (abs(current_grad-destination_grad)<0.01))
                state = 0;
            
            else if (0.2*sonar_margin<sonar_margin-right_sonar)
            {   
                last_right_sonar = right_sonar;
                move_around = 1;
                state = 3;
            }

            else if (right_sonar-sonar_margin>0.1*sonar_margin)
            {
                last_right_sonar = right_sonar;
                move_around = 1;
                state = 4;
            }
            else 
            {
                destination_grad = destination_y/destination_x;
                current_grad = current_y/current_x;
                state = 2;
            }
            ns = std::make_tuple(destination_grad, current_grad, last_right_sonar, move_around, state);
            break;

        case 3:

            if (right_sonar<last_right_sonar || right_sonar > 0.8* sonar_margin)
                state = 2;
            else 
            {
                last_right_sonar = right_sonar;
                move_around = 1;
                state = 3;
            }
            ns = std::make_tuple(destination_grad, current_grad, last_right_sonar, move_around, state);
            break;

        case 4:  
            move_around = 1;
            state = 5;
            ns = std::make_tuple(destination_grad, current_grad, last_right_sonar, move_around, state);
            break;

        case 5:
            if (right_sonar<last_right_sonar || right_sonar < sonar_margin || right_sonar == max_sonar)
                state = 2;
            else
            {
                last_right_sonar = right_sonar;
                move_around = 1;
                state = 5;
            }
            ns = std::make_tuple(destination_grad, current_grad, last_right_sonar, move_around, state);
            break;   
        default:
            ns = std::make_tuple(destination_grad, current_grad, last_right_sonar, move_around, state);

    } 



}

void controller_od_func(std::array<abst_ext<double>,3> &out,
                        const std::tuple<double,double,double,int,int>& cs,
                        const std::array<abst_ext<double>,6>& inp)
{

    double w_out = 0, x_out = 0, y_out = 0 ;
    double destination_grad, current_grad, last_right_sonar; 
    int move_around, state; 
    std::tie(destination_grad, current_grad, last_right_sonar, move_around, state) = cs;
    double current_y     = from_abst_ext(inp[0],0.0);
    double current_x     = from_abst_ext(inp[1],0.0);
    double current_angle = from_abst_ext(inp[2],0.0);
    switch (state)
    {
        case 0:
            w_out = rot_vel_gain*(desired_angle-current_angle)/abs(desired_angle-current_angle);
            x_out = 0;
            y_out = 0; 
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;
        case 1:
            w_out = 0;
            x_out = -1*lin_vel_gain*cos(pi/6);
            y_out = -1*lin_vel_gain*cos(pi/3); 
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;
           
        case 2: 
            w_out = 0;
            x_out = 1*lin_vel_gain*sin(pi/6);
            y_out = -1*lin_vel_gain*sin(pi/3); 
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;
            
        case 3: 
            w_out = rot_vel_gain * (-1);
            y_out = 0;
            x_out = 0;
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;

        case 4:
            y_out = -0.7*lin_vel_gain* cos(pi/3); 
            x_out = -0.7*lin_vel_gain* cos(pi/6);
            w_out = 0;
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;

        case 5:  
            w_out = rot_vel_gain;
            y_out = 0;
            x_out = 0;
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;

        case 6:  
            w_out = rot_vel_gain;
            y_out = 0;
            x_out = 0;
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;
        default:
            w_out = 0;
            y_out = 0;
            x_out = 0;
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);

    } //end of switch    

}

#endif