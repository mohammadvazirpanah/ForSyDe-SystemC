/**********************************************************************
    * abstract_sys.hpp -- SADF kernel: abstract system model         *
    *                                                                 *
    * Author:  Mohammad Vazirpanah (mohammad.vazirpanah@yahoo.com)    *
    *                                                                 *
    * Purpose: Implements the abstract self-model that fires only on  *
    *          REPLAN scenarios to re-run the D*-Lite shortest-path   *
    *          computation after new obstacles are detected.          *
    *                                                                 *
    * Usage:   D*-Lite path planning on TurtleBot3 via ROS/Gazebo     *
    *                                                                 *
    * License: BSD3                                                   *
    *******************************************************************/
#ifndef ABSTRACT_SYS_HPP
#define ABSTRACT_SYS_HPP

#include "globals.hpp"
#include "maze.hpp"         // for postprocessmaze, dstar_initialize
#include "controller.hpp"   // for updatemaze, computeshortestpath

// ---------------------------------------------------------------------------
// Abstract System Model (Ā^L_A^L) — SADF::kernelMN
//
// Scenario rates:
//   NORMAL / START: consume 0 from both inputs, produce 0  (inactive)
//   REPLAN:         consume 1 from both inputs, produce 1 to sink
//
// On REPLAN fires: updates D*-Lite graph for any new obstacles and
// re-computes the shortest path.  Uses global maze state already
// updated by the controller (maze[][].obstacle flags).
// ---------------------------------------------------------------------------

typedef map<scenario_type,
            tuple<std::array<size_t,1>, std::array<size_t,1>>>
        abstract_sys_table_type;

abstract_sys_table_type abstract_sys_table = {
    { START,  make_tuple(std::array<size_t,1>{0}, std::array<size_t,1>{0}) },
    { NORMAL, make_tuple(std::array<size_t,1>{0}, std::array<size_t,1>{0}) },
    { REPLAN, make_tuple(std::array<size_t,1>{1}, std::array<size_t,1>{1}) },
};

void abstract_sys_func(tuple<vector<int>>& outs,
                       const scenario_type& scenario,
                       const tuple<vector<monitor_state>>& inps)
{
    if (scenario != REPLAN) return;   // only active in REPLAN

    // Full D*-Lite replan from current mazegoal.
    // Incremental updatemaze only fixes cells in the robot's trace chain;
    // cells outside it (e.g. (4,2)) keep stale g-values that may still
    // route through the newly blocked cell, causing infinite re-REPLAN.
    // A full reset is always correct and fast enough for a 6x6 grid.
    postprocessmaze();      // restore all move[] = succ[] (clear old blockings)
    dstar_initialize();     // reset g/rhs/heap, re-seed mazestart, keymod=0

    // Re-apply every known obstacle: mask move[] edges in both directions
    for (int y = 0; y < MAZEHEIGHT; y++)
        for (int x = 0; x < MAZEWIDTH; x++)
            if (maze[y][x].obstacle)
                for (int d = 0; d < DIRECTIONS; d++)
                    if (maze[y][x].succ[d]) {
                        maze[y][x].move[d]                  = nullptr;
                        maze[y][x].succ[d]->move[revers[d]] = nullptr;
                    }

    if (computeshortestpath())
        SC_REPORT_ERROR("AbstractSys", "D*-Lite: no alternative path found!");

    mazegoal->trace = nullptr;

    cout << "[AbstractSys] Replanned from ("
         << mazegoal->x << "," << mazegoal->y << ")" << endl;
#ifdef DISPLAY
    printknownmaze(stdout);
#endif

    get<0>(outs)[0] = 0;   // dummy output, goes to sink
}

#endif // ABSTRACT_SYS_HPP
