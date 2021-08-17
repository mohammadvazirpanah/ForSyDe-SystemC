#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>

using namespace sc_core;
using namespace ForSyDe;


void controller_ns_func(std::tuple<double,double, double>& ns,
    const std::tuple<double,double, double>& cs,
    const double& inp)
{
    double right_sonar, left_sonar, left2_sonar, vx, vy;
    std::tie(right_sonar, left_sonar, left2_sonar) = cs;

    if (left2_sonar < 1 && left_sonar >= 1 && right_sonar >= 1)
    {
        vy = -0.57735026919 * inp; 
        vx = -1 * inp;

    }
    else if (left2_sonar >= 1 && left_sonar < 1 && right_sonar >= 1)
    {
        vy = inp; 
        vx = 0;
    }
    else if (left2_sonar < 1 && left_sonar >= 1 && right_sonar < 1)
    {
        vy = -1 * inp;
        vx = 0;
    }
    else if (left2_sonar < 1 && left_sonar < 1 && right_sonar > 1)
    {
        vy = 0.57735026919 * inp; 
        vx = -1 * inp;
    }
    else if (left2_sonar >= 1 && left_sonar >= 1 && right_sonar < 1)
    {
        vy = -0.57735026919 * inp; 
        vx = inp;
    }
    else if (left2_sonar >= 1 && left_sonar < 1 && right_sonar < 1)
    {
        vy = -0.57735026919 * inp; 
        vx = -1 * inp;
    }

}

void controller_od_func(double& out,
    const std::tuple<double,double, double>& cs,
    const double& inp)
{
   
}

#endif