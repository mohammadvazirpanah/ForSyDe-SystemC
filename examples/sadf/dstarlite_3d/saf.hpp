#ifndef SAF_HPP
#define SAF_HPP

#include "globals.hpp"

// ---------------------------------------------------------------------------
// SAF Detector (A^L_SAF) — SADF::detectorMN
//
// Input:  monitor_state (status, body_dirs, odom)
// Output: scenario_type → to controller and abstract_sys
//
// Logic:
//   - If previous scenario was REPLAN → return NORMAL (one-shot replan)
//   - If monitor reports STOCK         → switch to REPLAN
//   - Otherwise                        → NORMAL
// ---------------------------------------------------------------------------

typedef map<scenario_type, std::array<size_t,1>> saf_table_type;

saf_table_type saf_table = {
    { START,  {1} },
    { NORMAL, {1} },
    { REPLAN, {1} },
};

void saf_cds_func(scenario_type& new_scenario,
                  const scenario_type& prev_scenario,
                  const tuple<vector<monitor_state>>& inp)
{
    if (prev_scenario == REPLAN) {
        // Always return to NORMAL after one replan step
        new_scenario = NORMAL;
        return;
    }
    status_monitor status = get<0>(get<0>(inp)[0]);
    new_scenario = (status == STOCK) ? REPLAN : NORMAL;
}

void saf_kss_func(tuple<scenario_type>& out,
                  const scenario_type& current_scenario,
                  const tuple<vector<monitor_state>>& inp)
{
    get<0>(out) = current_scenario;
}

#endif // SAF_HPP
