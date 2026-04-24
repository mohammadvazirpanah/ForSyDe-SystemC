#ifndef MONITOR_HPP
#define MONITOR_HPP

#include "globals.hpp"

// ---------------------------------------------------------------------------
// Monitor (J_A^L) — SDF::combMN, always active
//
// Inputs:  scan_type        (360 LiDAR readings, body frame)
//          odom_type        (real-world position + orientation)
//          controller_state (previous step, from delay)
// Output:  monitor_state    = (status, body_dirs[4], odom)
//
// Sets STOCK only when phase == M_DRIVING AND obstacle in front sector.
// body_dirs = { front, left, back, right } — all in robot body frame.
// ---------------------------------------------------------------------------

static bool check_sector(const scan_type& scan, int from, int to)
{
    for (int i = from; i <= to; ++i) {
        if (i < (int)scan.size() && scan[i] > 0.01f && scan[i] < OBSTACLE_THRESH)
            return true;
    }
    return false;
}

void monitor_func(tuple<vector<monitor_state>>& outs,
                  const tuple<vector<scan_type>,
                              vector<odom_type>,
                              vector<controller_state>>& inps)
{
    const auto& scan = get<0>(inps)[0];
    const auto& odom = get<1>(inps)[0];
    const auto& cs   = get<2>(inps)[0];

    std::array<bool,4> body_dirs = {false, false, false, false};

    if (scan.size() >= 360) {
        // Front: indices 0-9 and 350-359  (±10° around robot nose)
        body_dirs[0] = check_sector(scan,   0,   9) ||
                       check_sector(scan, 350, 359);
        // Left:  80-100
        body_dirs[1] = check_sector(scan,  80, 100);
        // Back:  170-190
        body_dirs[2] = check_sector(scan, 170, 190);
        // Right: 260-280
        body_dirs[3] = check_sector(scan, 260, 280);
    }

    // STOCK only when actively driving AND obstacle directly ahead
    move_phase phase = CS_PHASE(cs);
    status_monitor status =
        (phase == M_DRIVING && body_dirs[0]) ? STOCK : NOSTOCK;

    get<0>(outs)[0] = make_tuple(status, body_dirs, odom);
}

#endif // MONITOR_HPP
