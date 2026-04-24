
#include <forsyde.hpp>
#include "globals.hpp"
#include "maze.hpp"
#include "controller.hpp"
#include "controller_monitor.hpp"
#include "controller_self_model.hpp"
#include "saf.hpp"

// #include <chrono>

using namespace sc_core;
using namespace ForSyDe;
using namespace std;
// using namespace std::chrono;

SC_MODULE(top)
{
    // Signals
    SDF::signal<maze_state> maze_init_out, maze_init_inp;
    SDF::signal<odom_type> maze_odom_out, maze_odom_out2;
    SDF::signal<laser_type> maze_laser_out;
    SDF::signal<cmd_type> controller_cmd_out, controller_cmd_out2, maze_cmd_inp, controller_self_cmd_out;
    SDF::signal<controller_state> controller_state_out, controller_state_inp;
    SDF::signal<controller_monitor_state> controller_monitor_state_out, controller_monitor_state_out2, controller_monitor_state_inp;
    SADF::signal<sa_state_type> controller_self_control_inp;
#ifdef FORSYDE_SELF_REPORTING
    // Communication pipes
    FILE* report_pipe;      // Report pipe
    int report_pipe_fd;     // Report pipe file descriptor
#endif

    SC_CTOR(top)
    {
        auto maze_odom_out_self = new SDF::signal<odom_type>("maze_odom_out_self",1000);
        auto maze_laser_out_self = new SDF::signal<laser_type>("maze_laser_out_self",1000);

        // Instantiate the maze
        auto maze1 = new SDF::combMN<tuple<odom_type,laser_type,maze_state>,tuple<cmd_type,maze_state>>(
            "maze1",
            maze_func,
            {1,1,1},
            {1,1}
        );
        get<0>(maze1->oport)(maze_odom_out);
        get<0>(maze1->oport)(maze_odom_out2);
        get<0>(maze1->oport)(*maze_odom_out_self);
        get<1>(maze1->oport)(maze_laser_out);
        get<1>(maze1->oport)(*maze_laser_out_self);
        get<2>(maze1->oport)(maze_init_out);
        get<0>(maze1->iport)(maze_cmd_inp);
        get<1>(maze1->iport)(maze_init_inp);

        SDF::make_delay("maze_init", make_tuple(false,0,0), maze_init_inp, maze_init_out);

        SDF::make_delay("cmd_init", NOP, maze_cmd_inp, controller_cmd_out);

        // Instantiate the D* Lite controller
        auto controller = new SDF::combMN<tuple<cmd_type,controller_state>,tuple<odom_type,laser_type,controller_state>>(
            "controller",
            controller_func,
            {1,1},
            {1,1,1}
        );
        get<0>(controller->iport)(maze_odom_out);
        get<1>(controller->iport)(maze_laser_out);
        get<2>(controller->iport)(controller_state_inp);
        get<0>(controller->oport)(controller_cmd_out);
        get<0>(controller->oport)(controller_cmd_out2);
        get<1>(controller->oport)(controller_state_out);

        SDF::make_delay("controller_state", INIT, controller_state_inp, controller_state_out);

        // Instantiate the controller monitor
        auto controller_monitor = new SDF::combMN<tuple<controller_monitor_state>,tuple<odom_type,cmd_type,controller_monitor_state>>(
            "controller_monitor",
            controller_monitor_func,
            {1},
            {1,1,1}
        );
        get<0>(controller_monitor->iport)(maze_odom_out2);
        get<1>(controller_monitor->iport)(controller_cmd_out2);
        get<2>(controller_monitor->iport)(controller_monitor_state_inp);
        get<0>(controller_monitor->oport)(controller_monitor_state_out);
        get<0>(controller_monitor->oport)(controller_monitor_state_out2);

        SDF::make_delay("controller_monitor_state", {NOP,odom_type(0,0),0}, controller_monitor_state_inp, controller_monitor_state_out);

        auto controller_self_model = new SADF::kernelMN<tuple<cmd_type>,sa_state_type,tuple<odom_type,laser_type>>(
            "controller_self_model",
            controller_self_model_func,
            controller_self_model_table
            #ifdef FORSYDE_SELF_REPORTING
            ,&report_pipe
            #endif
        );
        controller_self_model->cport1(controller_self_control_inp);
        get<0>(controller_self_model->iport)(*maze_odom_out_self);
        get<1>(controller_self_model->iport)(*maze_laser_out_self);
        get<0>(controller_self_model->oport)(controller_self_cmd_out);

        auto saf = new SADF::detectorMN<tuple<sa_state_type>,tuple<controller_monitor_state>,sa_state_type>(
            "saf",
            saf_cds_func,
            saf_kss_func,
            saf_table,
            NORMAL,
            array<size_t,1>{1}
            #ifdef FORSYDE_SELF_REPORTING
            ,&report_pipe
            #endif
        );
        get<0>(saf->iport)(controller_monitor_state_out2);
        get<0>(saf->oport)(controller_self_control_inp);

        SDF::make_sink("sink", [](const cmd_type&){}, controller_self_cmd_out);
    }
#ifdef FORSYDE_INTROSPECTION
    void start_of_simulation()
    {
        ForSyDe::XMLExport dumper("gen/");
        dumper.traverse(this);
#ifdef FORSYDE_SELF_REPORTING
        while (report_pipe_fd<=0) // pipe is not open
        {
            report_pipe_fd = open("gen/self_report", O_WRONLY|O_NONBLOCK);
            if (report_pipe_fd > 0)
                report_pipe = fdopen(report_pipe_fd, "w");
            else
                cout<<"Waiting for report pipe to open..."<<endl;
        }
#endif
    }
#endif
#ifdef FORSYDE_SELF_REPORTING
    void end_of_simulation()
    {
        fclose(report_pipe);
    }
#endif

};

