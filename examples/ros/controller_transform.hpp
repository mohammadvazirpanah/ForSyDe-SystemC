#ifndef CONTROLLER_TRANSFORM_HPP
#define CONTROLLER_TRANSFORM_HPP

#include <forsyde.hpp>
#include <math.h>
#include <global.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>  

using namespace sc_core;
using namespace ForSyDe;


void transform_ns_func(next_state_transform& ns,
                       const current_state_transform& cs,
                       const input_transform& inp)
{
    double in_x, in_y, in_angular_pos;
    double added_y, added_x;
    double cur_x, cur_y, last_in_x, last_in_y; 
    
    in_x           = from_abst_ext(inp[0],0.0);
    in_y           = from_abst_ext(inp[1],0.0);
    in_angular_pos = from_abst_ext(inp[2],0.0);
    cur_x          = from_abst_ext(cs[0],0.0);
    cur_y          = from_abst_ext(cs[1],0.0);
    last_in_x      = from_abst_ext(cs[2],0.0);
    last_in_y      = from_abst_ext(cs[3],0.0);

    added_y = (in_y - last_in_y) * cos((-1.0)*in_angular_pos) + (in_x - last_in_x) * sin((-1.0)*in_angular_pos);
    added_x = (-1.0)*(in_y - last_in_y) * sin((-1.0)*in_angular_pos) + (in_x - last_in_x) * cos((-1.0)*in_angular_pos);
    cur_x = cur_x + added_x;
    cur_y = cur_y + added_y;
    last_in_x = in_x;
    last_in_y = in_y;
    
    set_val (ns[0],cur_x);
    set_val (ns[1],cur_y);
    set_val (ns[2],last_in_x);
    set_val (ns[3],last_in_y);
}   

void transform_od_func(output_transform& out,
                       const current_state_transform& cs,
                       const input_transform& inp)
{

    double in_x, in_y, in_angular_pos, curr_angle;
    double added_y, added_x;
    double cur_x, cur_y, last_in_x, last_in_y; 
    
    in_x           = from_abst_ext(inp[0],0.0);
    in_y           = from_abst_ext(inp[1],0.0);
    in_angular_pos = from_abst_ext(inp[2],0.0);
    curr_angle = in_angular_pos + (pi/3.0);

    cur_x          = from_abst_ext(cs[0],0.0);
    cur_y          = from_abst_ext(cs[1],0.0);
    last_in_x      = from_abst_ext(cs[2],0.0);
    last_in_y      = from_abst_ext(cs[3],0.0);

    added_y = (in_y - last_in_y) * cos((-1.0)*in_angular_pos) + (in_x - last_in_x) * sin((-1.0)*in_angular_pos);
    added_x = (-1.0)*(in_y - last_in_y) * sin((-1.0)*in_angular_pos) + (in_x - last_in_x) * cos((-1.0)*in_angular_pos);
    cur_x = cur_x + added_x;
    cur_y = cur_y + added_y;
    last_in_x = in_x;
    last_in_y = in_y;

    set_val (out[0],cur_y);
    set_val (out[1],cur_x);
    set_val (out[2],curr_angle);

    // std::cout<<"cur_x="<<cur_x<<"\tcur_y="<<cur_y<<"\tcurr_angle="<<curr_angle<<std::endl;
}

#endif