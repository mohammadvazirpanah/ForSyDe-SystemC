
#ifndef CONTROLLER_MONITOR_HPP
#define CONTROLLER_MONITOR_HPP

#include <forsyde.hpp>
#include <tuple>
#include <vector>
#include "globals.hpp"


using namespace std;


void controller_monitor_func(tuple<vector<controller_monitor_state>>& outs,
                    const tuple<vector<odom_type>,vector<cmd_type>,vector<controller_monitor_state>>& inps)
{
    auto cur_odom = get<0>(inps)[0];
    auto cur_cmd = get<1>(inps)[0];
    auto controller_monitor_st = get<2>(inps)[0];
    auto prev_cmd = get<0>(controller_monitor_st);
    auto prev_odom = get<1>(controller_monitor_st);
    auto stuck_factor = get<2>(controller_monitor_st);

    if ((prev_cmd == cur_cmd) && (prev_odom == cur_odom))
        stuck_factor++;
    else
        stuck_factor = 0;
    if (stuck_factor > STUCK_REPLAN)
    {
        // cout << "[Controller monitor] Stuck! Replanning..." << endl;
        // keymodifier += H(lastcell);
        // lastcell = mazegoal;
        // for (tmpcell=mazegoal; tmpcell; tmpcell=tmpcell->trace)
        //     updatemaze(tmpcell);
        // if (computeshortestpath())
		//         SC_REPORT_ERROR("Controller","Cannot compute the shortest path!");
        // mazegoal->trace = NULL;
        
        stuck_factor = 0;
    }
    get<0>(outs)[0] = controller_monitor_state(cur_cmd, cur_odom, stuck_factor);
}

#endif
