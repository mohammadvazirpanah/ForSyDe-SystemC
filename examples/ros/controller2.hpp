#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>
#include <math.h>
#include <global.hpp>

using namespace sc_core;
using namespace ForSyDe;


void controller_ns_func(next_state& ns,
                        const current_state& cs,
                        const input& inp)
{

    double destination_grad;
    double current_grad;
    double last_right_sonar; 
    int move_around;
    MealyState state; 
    std::tie(destination_grad,
             current_grad,
             last_right_sonar, 
             move_around, 
             state) = cs;

    double current_y     = from_abst_ext(inp[0],0.0);
    double current_x     = from_abst_ext(inp[1],0.0);
    double current_angle = from_abst_ext(inp[2],0.0);
    double left_sonar    = from_abst_ext(inp[3],0.0);
    double right_sonar   = from_abst_ext(inp[4],0.0);
    double left2_sonar   = from_abst_ext(inp[5],0.0);

    switch (state)
    {   

        case Angle_Changer:  
            if (remainderf(abs(desired_angle-current_angle),2*pi) < rot_pos_margin)
            {
                state = Move_Toward;
            }
            ns = std::make_tuple(destination_grad, 
                                 current_grad, 
                                 last_right_sonar, 
                                 move_around, 
                                 state);
            break;

        case Move_Toward:
            if (right_sonar < sonar_margin)
            {
                last_right_sonar = right_sonar;
                move_around = 0;
                destination_grad = destination_y/destination_x;
                current_grad = current_y/current_x;
                state = Obstacle_Avoid;
            }
            ns = std::make_tuple(destination_grad, 
                        current_grad, 
                        last_right_sonar, 
                        move_around, 
                        state);
            break;

        case Obstacle_Avoid:  
            if ((move_around == 1) && (abs(current_grad-destination_grad)<0.01))
                state = Angle_Changer;
            
            else if (0.2*sonar_margin<sonar_margin-right_sonar)
            {   
                last_right_sonar = right_sonar;
                move_around = 1;
                state = Move_Far;
            }

            else if (right_sonar-sonar_margin>0.1*sonar_margin)
            {
                last_right_sonar = right_sonar;
                move_around = 1;
                state = Init_Move_Near;
            }
            else 
            {
                destination_grad = destination_y/destination_x;
                current_grad = current_y/current_x;
                state = Obstacle_Avoid;
            }
            ns = std::make_tuple(destination_grad, 
                                 current_grad, 
                                 last_right_sonar, 
                                 move_around, 
                                 state);
            break;

        case Move_Far:

            if (right_sonar<last_right_sonar || right_sonar > 0.8* sonar_margin)
                state = Obstacle_Avoid;
            else 
            {
                last_right_sonar = right_sonar;
                move_around = 1;
                state = Move_Far;
            }
            ns = std::make_tuple(destination_grad, 
                                 current_grad, 
                                 last_right_sonar, 
                                 move_around, 
                                 state);
            break;

        case Init_Move_Near:  
            move_around = 1;
            state = Move_Near;
            ns = std::make_tuple(destination_grad, 
                                 current_grad, 
                                 last_right_sonar, 
                                 move_around, 
                                 state);
            break;

        case Move_Near:
            if (right_sonar>last_right_sonar || right_sonar < sonar_margin || right_sonar == max_sonar)
                state = Obstacle_Avoid;
            else
            {
                last_right_sonar = right_sonar;
                move_around = 1;
                state = Move_Near;
            }
            ns = std::make_tuple(destination_grad, 
                                 current_grad, 
                                 last_right_sonar, 
                                 move_around, 
                                 state);
            break;   
        default:
            ns = std::make_tuple(destination_grad, 
                                 current_grad, 
                                 last_right_sonar, 
                                 move_around, 
                                 state);
    } 

}

void controller_od_func(output& out,
                        const current_state& cs,
                        const input& inp)
{

    double w_out = 0, x_out = 0, y_out = 0 ;
    double destination_grad, current_grad, last_right_sonar;
    int move_around; 
    MealyState state;
    double current_y     = from_abst_ext(inp[0],0.0);
    double current_x     = from_abst_ext(inp[1],0.0);
    double current_angle = from_abst_ext(inp[2],0.0);

    std::tie(destination_grad, 
             current_grad, 
             last_right_sonar, 
             move_around, 
             state) = cs;

    switch (state)
    {
        case Angle_Changer:
            w_out = rot_vel_gain*(desired_angle-current_angle)/abs(desired_angle-current_angle);
            x_out = 0;
            y_out = 0; 
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;

        case Move_Toward:
            w_out = 0;
            x_out = -1*lin_vel_gain*cos(pi/6);
            y_out = -1*lin_vel_gain*cos(pi/3); 
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;
           
        case Obstacle_Avoid: 
            w_out = 0;
            x_out = 1*lin_vel_gain*sin(pi/6);
            y_out = -1*lin_vel_gain*sin(pi/3); 
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;
            
        case Move_Far: 
            w_out = rot_vel_gain * (-1);
            y_out = 0;
            x_out = 0;
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;

        case Init_Move_Near:
            y_out = -0.7*lin_vel_gain* cos(pi/3); 
            x_out = -0.7*lin_vel_gain* cos(pi/6);
            w_out = 0;
            set_val (out [0],w_out);
            set_val (out [1],x_out);
            set_val (out [2],y_out);
            break;

        case Move_Near:  
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