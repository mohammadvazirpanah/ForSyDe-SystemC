#ifndef TOP_HPP
#define TOP_HPP

#include <forsyde.hpp>
#include "forsyde/ros_hsdf_wrapper.hpp"
#include "globals.hpp"
#include "monitor.hpp"
#include "controller.hpp"
#include "abstract_sys.hpp"
#include "saf.hpp"

using namespace sc_core;
using namespace ForSyDe;
using namespace std;

SC_MODULE(top)
{
    // ----- ROS topic names -----
    vector<string> topics_pub = { "/cmd_vel" };
    vector<string> topics_sub = { "/odom", "/scan" };

    // ----- Signals: ROS Wrapper -----
    SDF::signal<odom_type>       from_wrapper_odom;
    SDF::signal<scan_type>       from_wrapper_scan;
    SDF::signal<cmd_type_robot>  to_wrapper_cmd;

    // ----- Signals: Monitor -----
    SDF::signal<monitor_state>   monitor_to_saf;
    SDF::signal<monitor_state>   monitor_to_ctrl;

    // ----- Signals: Controller state (delay loop) -----
    SDF::signal<controller_state> ctrl_state_out;
    SDF::signal<controller_state> ctrl_state_to_ctrl;
    SDF::signal<controller_state> ctrl_state_to_monitor;

    // ----- Signals: SAF scenario -----
    SADF::signal<scenario_type>  scenario_to_ctrl;
    SADF::signal<scenario_type>  scenario_to_abssys;

    // ----- Signal: abstract_sys (large buffer, accumulates during NORMAL) -----
    SDF::signal<monitor_state>*  monitor_to_abssys;

#ifdef FORSYDE_SELF_REPORTING
    FILE* report_pipe    = nullptr;
    int   report_pipe_fd = -1;
#endif

    SC_CTOR(top)
    {
        // Large-buffer signal for abstract_sys (tokens accumulate during NORMAL scenario)
        monitor_to_abssys = new SDF::signal<monitor_state>("monitor_to_abs", 10000);

        // ---- ROS Wrapper ----
        SDF::make_sdf_roswrap("SdfRosWrapper",
                              topics_pub, topics_sub,
                              to_wrapper_cmd,
                              from_wrapper_odom,
                              from_wrapper_scan);

        // ---- Monitor (SDF::combMN) ----
        // inputs:  scan, odom, ctrl_state_delayed
        // outputs: monitor_state (to SAF, controller, abstract_sys)
        auto monitor = new SDF::combMN<
            tuple<monitor_state>,
            tuple<scan_type, odom_type, controller_state>>(
            "monitor",
            monitor_func,
            {1},
            {1, 1, 1}
        );
        get<0>(monitor->iport)(from_wrapper_scan);
        get<1>(monitor->iport)(from_wrapper_odom);
        get<2>(monitor->iport)(ctrl_state_to_monitor);
        get<0>(monitor->oport)(monitor_to_saf);
        get<0>(monitor->oport)(monitor_to_ctrl);
        get<0>(monitor->oport)(*monitor_to_abssys);   // large-buffer copy

        // ---- SAF Detector (SADF::detectorMN) ----
        auto saf = new SADF::detectorMN<
            tuple<scenario_type>,
            tuple<monitor_state>,
            scenario_type>(
            "saf",
            saf_cds_func,
            saf_kss_func,
            saf_table,
            NORMAL,
            std::array<size_t,1>{1}
#ifdef FORSYDE_SELF_REPORTING
            , &report_pipe
#endif
        );
        get<0>(saf->iport)(monitor_to_saf);
        get<0>(saf->oport)(scenario_to_ctrl);
        get<0>(saf->oport)(scenario_to_abssys);

        // ---- Controller / A^L (SADF::kernelMN) ----
        // inputs:  odom, monitor_state, ctrl_state_delayed
        // outputs: cmd_type_robot, ctrl_state
        auto controller = new SADF::kernelMN<
            tuple<cmd_type_robot, controller_state>,
            scenario_type,
            tuple<monitor_state, controller_state>>(
            "controller",
            controller_func,
            controller_table
#ifdef FORSYDE_SELF_REPORTING
            , &report_pipe
#endif
        );
        controller->cport1(scenario_to_ctrl);
        get<0>(controller->iport)(monitor_to_ctrl);
        get<1>(controller->iport)(ctrl_state_to_ctrl);
        get<0>(controller->oport)(to_wrapper_cmd);
        get<1>(controller->oport)(ctrl_state_out);

        // ---- Controller state delay (self-feedback) ----
        auto ctrl_delay = SDF::make_delay("ctrl_delay",
                                          INIT_CTRL_STATE,
                                          ctrl_state_to_ctrl,
                                          ctrl_state_out);
        // Fan-out delay output to monitor as well
        ctrl_delay->oport1(ctrl_state_to_monitor);

        // ---- Abstract System Model / Ā^L_A^L (SADF::kernelMN) ----
        // Inactive in NORMAL/START (rate 0), fires once per REPLAN.
        auto abstract_sys = new SADF::kernelMN<
            tuple<int>,
            scenario_type,
            tuple<monitor_state>>(
            "abstract_sys",
            abstract_sys_func,
            abstract_sys_table
#ifdef FORSYDE_SELF_REPORTING
            , &report_pipe
#endif
        );
        abstract_sys->cport1(scenario_to_abssys);
        get<0>(abstract_sys->iport)(*monitor_to_abssys);
        // abstract_sys output is a dummy int → goes to a dedicated sink signal
        SDF::signal<int>* abssys_out_sig = new SDF::signal<int>("abssys_out", 10000);
        get<0>(abstract_sys->oport)(*abssys_out_sig);
        SDF::make_sink("abssys_sink", [](const int&){}, *abssys_out_sig);

    }

#ifdef FORSYDE_INTROSPECTION
    void start_of_simulation()
    {
        ForSyDe::XMLExport dumper("gen/");
        dumper.traverse(this);
#ifdef FORSYDE_SELF_REPORTING
        while (report_pipe_fd <= 0) {
            report_pipe_fd = open("gen/self_report", O_WRONLY | O_NONBLOCK);
            if (report_pipe_fd > 0)
                report_pipe = fdopen(report_pipe_fd, "w");
            else
                cout << "Waiting for report pipe to open..." << endl;
        }
#endif
    }
#endif

#ifdef FORSYDE_SELF_REPORTING
    void end_of_simulation() { fclose(report_pipe); }
#endif
};

#endif // TOP_HPP
