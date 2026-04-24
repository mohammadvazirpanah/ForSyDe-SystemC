/**********************************************************************
    * main.cpp -- entry point for the D*-Lite/SADF simulation        *
    *                                                                 *
    * Author:  Mohammad Vazirpanah (mohammad.vazirpanah@yahoo.com)    *
    *                                                                 *
    * Purpose: Instantiates the top-level ForSyDe-SystemC module and  *
    *          starts the SystemC simulation kernel.                  *
    *                                                                 *
    * Usage:   D*-Lite path planning on TurtleBot3 via ROS/Gazebo     *
    *                                                                 *
    * License: BSD3                                                   *
    *******************************************************************/
#include <forsyde.hpp>
#include "top.hpp"

#ifdef FORSYDE_SELF_REPORTING
#include <fcntl.h>
#endif

int sc_main(int argc, char* argv[])
{
    top top1("top1");
    sc_core::sc_start();
    return 0;
}
