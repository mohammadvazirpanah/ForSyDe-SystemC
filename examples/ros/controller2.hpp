#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>

using namespace sc_core;
using namespace ForSyDe;

double rot_pos_margin = 0.01;
double rot_vel_gain = 2;
double lin_vel_gain = 4;
double w_out = 0 , x_out=0 , y_out=0 ;
double sonar_margin = 0.7;
double max_sonar = 1.4;

void controller_ns_func(std::tuple<double,double, double>& ns,
    const std::tuple<double,double, double>& cs,
    const double& inp)
{




}

void controller_od_func(double& out,
    const std::tuple<double,double, double>& cs,
    const double& inp)
{
   

   
}

#endif