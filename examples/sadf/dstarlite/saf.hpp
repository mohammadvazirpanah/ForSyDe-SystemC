
#ifndef SAF_HPP
#define SAF_HPP

#include <forsyde.hpp>
#include <tuple>
#include <vector>
#include "globals.hpp"


using namespace std;

typedef map<
            sa_state_type,
            array<size_t,1>
        > saf_table_type;


saf_table_type saf_table = {
    {NORMAL, {1}},
    {CONTROLLER_CHECK, {1}}
};

void saf_cds_func(sa_state_type& new_scenario, 
                const sa_state_type& previous_scenario, 
                const tuple<vector<controller_monitor_state>>& inp)
{
    if (get<2>(get<0>(inp)[0]) == 2)
        new_scenario = CONTROLLER_CHECK;
    else
        new_scenario = NORMAL;
}


void saf_kss_func(tuple<sa_state_type>& out,
                const sa_state_type& current_scenario,
                const tuple<vector<controller_monitor_state>>& inp)
{
    if (current_scenario == CONTROLLER_CHECK)
        out = make_tuple(CONTROLLER_CHECK);
    else
        out = make_tuple(NORMAL);
}


#endif
