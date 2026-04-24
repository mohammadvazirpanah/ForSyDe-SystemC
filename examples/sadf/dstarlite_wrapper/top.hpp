
#include <forsyde.hpp>
#include "forsyde/ros_hsdf_wrapper.hpp"
#include "monitor.hpp"
#include "controller.hpp"
#include "globals.hpp"
#include "saf.hpp"
// #include "controller_self_model.hpp"
#include <cmath>
#include <limits>


using namespace sc_core;
using namespace ForSyDe;
using namespace std;




SC_MODULE(top)
{
    

    std::vector <std::string> topics_publisher=  
    {
        "/cmd_vel"
    }; 

    std::vector <std::string> topics_subscriber= 
    {
        "/odom",
        "/scan"
    };

    SDF::signal<odom_type> from_wrapper_odom;
    SDF::signal<scan_type> from_wrapper_scan;
    SDF::signal<cmd_type> to_controller_cmd_out, to_controller_cmd_in;
    SDF::signal<cmd_type_robot> to_wrapper_cmd;
    SDF::signal<monitor_state> from_monitor_out, from_monitor_out2;
    SDF::signal<controller_state> controller_state_out, controller_state_inp;
    // SADF::signal<sa_state_type> controller_self_control_inp;
    SDF::signal<path> controller_inp_path;
    SADF::signal<contorller_scenario_type> controller_control_inp;

    SC_CTOR(top)
    {
        SDF::make_sdf_roswrap("SdfRosWrapper", 
                            topics_publisher,
                            topics_subscriber, 
                            to_wrapper_cmd,
                            from_wrapper_odom,
                            from_wrapper_scan
                            );

        auto monitor = new SDF::combMN<tuple<monitor_state>,tuple<scan_type,odom_type>>(
        "monitor_checking",
        monitor_func,
        {1},
        {1,1}
        );
        get<0>(monitor->iport)(from_wrapper_scan);                //from Wrapper (for getting scan data)
        get<1>(monitor->iport)(from_wrapper_odom);               //from Wrapper (for getting odom data)
        get<0>(monitor->oport)(from_monitor_out);               //to controller (for getting online data)
        get<0>(monitor->oport)(from_monitor_out2);             //to SAF detector (for checking status)




        auto controller = new SADF::kernelMN<tuple<cmd_type_robot, controller_state>, 
                                            contorller_scenario_type,                     
                                            tuple<path, monitor_state, controller_state>>
                                            (
                                            "controller",
                                            controller_func,
                                            controller_table
                                            #ifdef FORSYDE_SELF_REPORTING
                                            ,&report_pipe
                                            #endif
                                            );
        controller->cport1(controller_control_inp);                //scenario from self-aware detector
        get<0>(controller->iport)(controller_inp_path);           //path from controller self-model
        get<1>(controller->iport)(from_monitor_out);             //monitor state from monitor
        get<2>(controller->iport)(controller_state_inp);        //from Delay (for getting self-state) 
        get<0>(controller->oport)(to_wrapper_cmd);             //to Wrapper  (for sending cmd robot)
        get<1>(controller->oport)(controller_state_out);      //to Delay (for saving self-state)


        SDF::make_delay("controller_state", {INITCONTROLL, odom_type{0.0,0.0,0.0,0.0,0.0}}, controller_state_inp, controller_state_out);


        auto saf = new SADF::detectorMN<tuple<contorller_scenario_type>,tuple<monitor_state>,contorller_scenario_type>(
            "saf",
            saf_cds_func,
            saf_kss_func,
            saf_table,
            START,
            array<size_t,1>{1}
            #ifdef FORSYDE_SELF_REPORTING
            ,&report_pipe
            #endif
        );
        get<0>(saf->iport)(from_monitor_out2);            //from monitor (for getting status)
        get<0>(saf->oport)(controller_control_inp);  //to controller (for setting scenario)


        // auto controller = new SDF::combMN<tuple<cmd_type_robot,controller_state>,
        //                                 tuple<monitor_state,controller_state>(
        // "controller",
        // controller_func,
        // {1,1},
        // {1,1}
        // );

        // get<0>(controller->iport)(from_monitor_out);
        // get<1>(controller->iport)(controller_state_inp);
        // // get<2>(controller->iport)(controller_inp_path);

        // get<0>(controller->oport)(to_wrapper_cmd);
        // get<1>(controller->oport)(controller_state_out);

        // auto controller_self_model = new SADF::kernelMN<tuple<cmd_type>,sa_state_type,tuple<odom_type,laser_type>>(
        // "controller_self_model",
        // controller_self_model_func,
        // controller_self_model_table
        // #ifdef FORSYDE_SELF_REPORTING
        // ,&report_pipe
        // #endif
        // );
        // controller_self_model->cport1(controller_self_control_inp);
        // get<0>(controller_self_model->iport)(*maze_odom_out_self);
        // get<1>(controller_self_model->iport)(*maze_laser_out_self);
        // get<0>(controller_self_model->oport)(controller_self_cmd_out);


        
        
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

