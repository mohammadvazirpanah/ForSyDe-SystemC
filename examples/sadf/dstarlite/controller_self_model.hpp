
#ifndef CONTROLLER_SELF_MODEL_HPP
#define CONTROLLER_SELF_MODEL_HPP

#include <forsyde.hpp>
#include <tuple>
#include <vector>
#include "globals.hpp"


using namespace std;

typedef map<
            sa_state_type,
            tuple<array<size_t,2>,array<size_t,1>>
        > controller_self_model_table_type;

controller_self_model_table_type controller_self_model_table = 
{  
    {NORMAL,           make_tuple(array<size_t,2>{0,0},array<size_t,1>{0})},
    {CONTROLLER_CHECK, make_tuple(array<size_t,2>{1,1},array<size_t,1>{1})}
};

void controller_self_model_func(tuple<vector<cmd_type>>& out, 
                    const sa_state_type& _scenarios_state, 
                    const tuple<vector<odom_type>,vector<laser_type>>& inp)
{
    if (_scenarios_state == CONTROLLER_CHECK)
    {
        keymodifier += H(lastcell);
        lastcell = mazegoal;
        for (tmpcell=mazegoal; tmpcell; tmpcell=tmpcell->trace)
            updatemaze(tmpcell);
        if (computeshortestpath())
		        SC_REPORT_ERROR("Controller","Cannot compute the shortest path!");
        mazegoal->trace = NULL;
    }
}

#endif
