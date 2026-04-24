/**********************************************************************
    * controller.hpp -- SADF kernel: D*-Lite controller actor        *
    *                                                                 *
    * Author:  Mohammad Vazirpanah (mohammad.vazirpanah@yahoo.com)    *
    *                                                                 *
    * Purpose: Implements the D*-Lite algorithm functions and the     *
    *          SADF kernel process that drives robot movement,        *
    *          updates the obstacle map, and issues velocity commands. *
    *                                                                 *
    * Usage:   D*-Lite path planning on TurtleBot3 via ROS/Gazebo     *
    *                                                                 *
    * License: BSD3                                                   *
    *******************************************************************/
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "globals.hpp"
#include "maze.hpp"
#include "heap.hpp"

// ---------------------------------------------------------------------------
// D*-Lite algorithm functions (ported from dstarlite/controller.hpp)
// ---------------------------------------------------------------------------

static void initializecell(cell *c)
{
    if (c->generated != mazeiteration) {
        c->g = c->rhs = LARGE;
        c->searchtree = nullptr;
        c->generated  = mazeiteration;
    }
}

static void updatecell(cell *c)
{
    if (c->g < c->rhs) {
#ifdef TIEBREAKING
        c->key[0] = c->g   + H(c) + keymodifier;
        c->key[1] = c->g   + H(c) + keymodifier;
        c->key[2] = c->g;
#else
        c->key[0] = c->g   + H(c) + keymodifier;
        c->key[1] = c->g;
#endif
        insertheap(c);
    } else if (c->g > c->rhs) {
#ifdef TIEBREAKING
        c->key[0] = c->rhs + H(c) + keymodifier;
        c->key[1] = c->rhs + H(c) + keymodifier + 1;
        c->key[2] = H(c) + keymodifier;
#else
        c->key[0] = c->rhs + H(c) + keymodifier;
        c->key[1] = c->rhs;
#endif
        insertheap(c);
    } else {
        deleteheap(c);
    }
}

static void updatekey(cell *c)
{
    if (c->g < c->rhs) {
#ifdef TIEBREAKING
        c->key[0] = c->g   + H(c) + keymodifier;
        c->key[1] = c->g   + H(c) + keymodifier;
        c->key[2] = c->g;
#else
        c->key[0] = c->g   + H(c) + keymodifier;
        c->key[1] = c->g;
#endif
    } else {
#ifdef TIEBREAKING
        c->key[0] = c->rhs + H(c) + keymodifier;
        c->key[1] = c->rhs + H(c) + keymodifier + 1;
        c->key[2] = H(c) + keymodifier;
#else
        c->key[0] = c->rhs + H(c) + keymodifier;
        c->key[1] = c->rhs;
#endif
    }
}

static void updaterhs(cell *c)
{
    c->rhs = LARGE;
    c->searchtree = nullptr;
    for (int d = 0; d < DIRECTIONS; ++d) {
        if (c->move[d] && c->move[d]->generated == mazeiteration &&
            c->rhs > c->move[d]->g + 1)
        {
            c->rhs        = c->move[d]->g + 1;
            c->searchtree = c->move[d];
        }
    }
    updatecell(c);
}

static int computeshortestpath()
{
    cell goal_tmp, old_tmp;
    cell *c1, *c2;

#ifdef TIEBREAKING
    auto fill_key = [&](cell &dst, cell *src) {
        if (src->g < src->rhs) {
            dst.key[0] = src->g   + keymodifier;
            dst.key[1] = src->g   + keymodifier;
            dst.key[2] = src->g;
        } else {
            dst.key[0] = src->rhs + keymodifier;
            dst.key[1] = src->rhs + keymodifier + 1;
            dst.key[2] = keymodifier;
        }
    };
#else
    auto fill_key = [&](cell &dst, cell *src) {
        if (src->g < src->rhs) {
            dst.key[0] = src->g   + keymodifier;
            dst.key[1] = src->g;
        } else {
            dst.key[0] = src->rhs + keymodifier;
            dst.key[1] = src->rhs;
        }
    };
#endif

    fill_key(goal_tmp, mazegoal);

    while (topheap() &&
           (mazegoal->rhs > mazegoal->g || keyless(topheap(), &goal_tmp)))
    {
        c1 = topheap();
        old_tmp.key[0] = c1->key[0];
        old_tmp.key[1] = c1->key[1];
#ifdef TIEBREAKING
        old_tmp.key[2] = c1->key[2];
#endif
        updatekey(c1);
        if (keyless(&old_tmp, c1)) {
            updatecell(c1);
        } else if (c1->g > c1->rhs) {
            c1->g = c1->rhs;
            deleteheap(c1);
            for (int d = 0; d < DIRECTIONS; ++d) {
                if (c1->move[d]) {
                    c2 = c1->move[d];
                    initializecell(c2);
                    if (c2 != mazestart && c2->rhs > c1->g + 1) {
                        c2->rhs        = c1->g + 1;
                        c2->searchtree = c1;
                        updatecell(c2);
                    }
                }
            }
        } else {
            c1->g = LARGE;
            updatecell(c1);
            for (int d = 0; d < DIRECTIONS; ++d) {
                if (c1->move[d]) {
                    c2 = c1->move[d];
                    initializecell(c2);
                    if (c2 != mazestart && c2->searchtree == c1)
                        updaterhs(c2);
                }
            }
        }
        fill_key(goal_tmp, mazegoal);
    }
    return (mazegoal->rhs == LARGE);
}

// ---------------------------------------------------------------------------
// updatemaze: called after marking new obstacles in maze[][].obstacle
// Removes move[] edges for obstacle cells and updates D*-Lite priorities.
// ---------------------------------------------------------------------------
void updatemaze(cell *robot)
{
    for (int d1 = 0; d1 < DIRECTIONS; ++d1) {
        if (robot->move[d1] && robot->move[d1]->obstacle) {
            cell *obs = robot->move[d1];
            initializecell(obs);
            for (int d2 = 0; d2 < DIRECTIONS; ++d2) {
                if (obs->move[d2]) {
                    cell *nb = obs->succ[d2];
                    obs->move[d2]          = nullptr;
                    nb->move[revers[d2]]   = nullptr;
                    initializecell(nb);
                    if (nb != mazestart && nb->searchtree == obs)
                        updaterhs(nb);
                }
            }
            if (obs != mazestart) {
                obs->rhs = LARGE;
                updatecell(obs);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// controller_table: scenario → consumption/production rates
//  inputs : (odom_type, monitor_state, controller_state)
//  outputs: (cmd_type_robot, controller_state)
// ---------------------------------------------------------------------------
typedef map<scenario_type,
            tuple<std::array<size_t,2>, std::array<size_t,2>>>
        controller_table_type;

controller_table_type controller_table = {
    { START,  make_tuple(std::array<size_t,2>{1,1}, std::array<size_t,2>{1,1}) },
    { NORMAL, make_tuple(std::array<size_t,2>{1,1}, std::array<size_t,2>{1,1}) },
    { REPLAN, make_tuple(std::array<size_t,2>{1,1}, std::array<size_t,2>{1,1}) },
};

// ---------------------------------------------------------------------------
// controller_func — A^L kernel
// ---------------------------------------------------------------------------
void controller_func(tuple<vector<cmd_type_robot>,
                           vector<controller_state>>& outs,
                     const scenario_type& scenario,
                     const tuple<vector<monitor_state>,
                                 vector<controller_state>>& inps)
{
    const auto& mon     = get<0>(inps)[0];
    const auto& cs_prev = get<1>(inps)[0];
    const auto& odom    = get<2>(mon);    // odom is carried inside monitor_state

    cmd_type_robot cmd_out = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    controller_state cs = cs_prev;   // start from previous state

    float cur_x = odom[0];
    float cur_y = odom[1];
    float cur_yaw = get_yaw(odom);

    // ----- REPLAN scenario: mark obstacle, stop robot, wait for abstract_sys -----
    // REPLAN fires because monitor detected STOCK (obstacle ahead while M_DRIVING).
    // The M_DRIVING STOCK block is never reached (SAF switches scenario first),
    // so obstacle marking must happen here using the trusted CS_GX/CS_GY/CS_CMD.
    if (scenario == REPLAN) {
        int obs_gx = CS_GX(cs) + dx[(int)CS_CMD(cs)];
        int obs_gy = CS_GY(cs) + dy[(int)CS_CMD(cs)];
        if (obs_gx >= 0 && obs_gx < MAZEWIDTH &&
            obs_gy >= 0 && obs_gy < MAZEHEIGHT &&
            !maze[obs_gy][obs_gx].obstacle)
        {
            maze[obs_gy][obs_gx].obstacle = true;
            cout << "[Controller] Obstacle detected at grid ("
                 << obs_gx << "," << obs_gy << ")" << endl;
        }
        CS_PHASE(cs) = M_STOPPED;
        CS_OBS(cs)   = false;
        get<0>(outs)[0] = cmd_out;
        get<1>(outs)[0] = cs;
        return;
    }

    // ----- Normal / Start: run state machine -----
    switch (CS_PROG(cs)) {

    case CTRL_INIT: {
        init_maze();
        CS_GX(cs) = world_to_grid_x(cur_x);
        CS_GY(cs) = world_to_grid_y(cur_y);
        mazegoal  = &maze[CS_GY(cs)][CS_GX(cs)];
        lastcell  = mazegoal;
        mazegoal->searchtree = nullptr;
        CS_PROG(cs)  = CTRL_PLAN;
        CS_PHASE(cs) = M_STOPPED;
        break;
    }

    case CTRL_PLAN: {
        if (computeshortestpath())
            SC_REPORT_ERROR("Controller", "D*-Lite: no path to goal!");
        mazegoal->trace = nullptr;
#ifdef DISPLAY
        printknownmaze(stdout);
#endif
        CS_PROG(cs)  = CTRL_MOVE;
        CS_PHASE(cs) = M_STOPPED;
        break;
    }

    case CTRL_MOVE: {
        // Reached the actual goal?
        if (mazegoal == mazestart) {
            CS_PROG(cs)  = CTRL_IDLE;
            CS_PHASE(cs) = M_STOPPED;
            CS_OBS(cs)   = false;   // reused as idle-cycle counter
            break;
        }

        // After a replan we come back here with obstacle_detected cleared.
        // The searchtree has been updated by abstract_sys — just restart motion.
        if (CS_OBS(cs)) {
            CS_OBS(cs)   = false;
            CS_PHASE(cs) = M_STOPPED;
        }

        move_phase ph = CS_PHASE(cs);

        // ---- M_STOPPED: decide next cell, start rotating ----
        if (ph == M_STOPPED) {
            if (!mazegoal->searchtree) {
                // No path — shouldn't happen, but be safe
                break;
            }
            // Find direction to searchtree
            int d = -1;
            for (int i = 0; i < DIRECTIONS; ++i) {
                int nx = mazegoal->x + dx[i];
                int ny = mazegoal->y + dy[i];
                if (nx >= 0 && nx < MAZEWIDTH && ny >= 0 && ny < MAZEHEIGHT &&
                    &maze[ny][nx] == mazegoal->searchtree)
                {
                    d = i;
                    break;
                }
            }
            if (d < 0) break;   // no valid direction

            CS_CMD(cs)  = (cmd_type)d;
            CS_TYAW(cs) = direction_yaw((cmd_type)d);
            CS_TX(cs)   = cell_center_x(mazegoal->x + dx[d]);
            CS_TY(cs)   = cell_center_y(mazegoal->y + dy[d]);
            CS_PHASE(cs)= M_ROTATING;
            // fall through to ROTATING this step
            ph = M_ROTATING;
        }

        // ---- M_ROTATING ----
        if (ph == M_ROTATING) {
            float err = normalize_angle(CS_TYAW(cs) - cur_yaw);
            if (fabsf(err) < YAW_TOL) {
                CS_PHASE(cs) = M_DRIVING;
                // zero velocity this step (brief stop before driving)
            } else {
                float av = (err > 0) ? ANGULAR_VEL : -ANGULAR_VEL;
                cmd_out = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, av};
            }
            break;
        }

        // ---- M_DRIVING ----
        if (ph == M_DRIVING) {
            // Check obstacle ahead (monitor sets STOCK when obstacle < threshold)
            status_monitor mon_status = get<0>(mon);
            if (mon_status == STOCK) {
                // Use CS_GX/CS_GY (trusted grid position, drift-free) to mark obstacle.
                // world_to_grid_x(cur_x) is unreliable: robot can drift across cell
                // boundaries during rotation, causing the wrong cell to be flagged.
                int obs_gx = CS_GX(cs) + dx[(int)CS_CMD(cs)];
                int obs_gy = CS_GY(cs) + dy[(int)CS_CMD(cs)];
                if (obs_gx >= 0 && obs_gx < MAZEWIDTH &&
                    obs_gy >= 0 && obs_gy < MAZEHEIGHT)
                {
                    maze[obs_gy][obs_gx].obstacle = true;
                    cout << "[Controller] Obstacle detected at grid ("
                         << obs_gx << "," << obs_gy << ")" << endl;
                }
                CS_OBS(cs)   = true;
                CS_PHASE(cs) = M_STOPPED;
                // cmd_out stays zero — SAF will trigger REPLAN next cycle
                break;
            }

            // Arrival check: entered target grid cell OR close to center
            float dx_to_tgt = CS_TX(cs) - cur_x;
            float dy_to_tgt = CS_TY(cs) - cur_y;
            float dist = sqrtf(dx_to_tgt * dx_to_tgt + dy_to_tgt * dy_to_tgt);
            int tgt_gx = world_to_grid_x(CS_TX(cs));
            int tgt_gy = world_to_grid_y(CS_TY(cs));
            bool in_target_cell = (world_to_grid_x(cur_x) == tgt_gx &&
                                   world_to_grid_y(cur_y) == tgt_gy);
            if (dist < POS_TOL || in_target_cell) {
                // Move D*-Lite position to next cell
                mazegoal->searchtree->trace = mazegoal;
                mazegoal  = mazegoal->searchtree;
                CS_GX(cs) = mazegoal->x;
                CS_GY(cs) = mazegoal->y;
                CS_PHASE(cs) = M_STOPPED;
#ifdef DISPLAY
                printknownmaze(stdout);
#endif
            } else {
                // Proportional steering toward target cell center (corrects lateral drift)
                float target_heading = atan2f(dy_to_tgt, dx_to_tgt);
                float heading_err = normalize_angle(target_heading - cur_yaw);
                float angular = 2.0f * heading_err;
                cmd_out = {LINEAR_VEL, 0.0f, 0.0f, 0.0f, 0.0f, angular};
            }
        }
        break;
    }

    case CTRL_IDLE: {
        // Send zero velocity for two cycles so the wrapper publishes it
        // before sc_stop() halts simulation (last cmd_vel must be zero,
        // otherwise the robot keeps driving after SystemC stops).
        if (!CS_OBS(cs)) {
            CS_OBS(cs) = true;   // first cycle: zero cmd published this cycle
        } else {
            cout << "[Controller] Goal reached! Simulation complete." << endl;
            sc_core::sc_stop();  // second cycle: zero already in channel
        }
        // cmd_out stays {0,0,0,0,0,0}
        break;
    }

    } // end switch

    get<0>(outs)[0] = cmd_out;
    get<1>(outs)[0] = cs;
}

#endif // CONTROLLER_HPP
