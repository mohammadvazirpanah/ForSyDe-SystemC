#include <forsyde.hpp>
#include "forsyde/ros_wrapper.hpp"
#include <iostream>
#include <controller2.hpp>
#include <global.hpp>
#include <controller_transform.hpp>
#include "sensor_msgs/Range.h"
#include <math.h>
#include <limits.h>

using namespace sc_core;
using namespace ForSyDe;

void inv_kinematics_func (std::array<abst_ext<double>,3>& in_transform_mealy, const double& x, const double& y, const double& z)
{
    double x_pos, y_pos, angular_pos;

    angular_pos   =  ((-1.0/3.0) * (y)) + ((-1.0/3.0) * (z)) + ((-1.0/3.0) * (x));
    x_pos         =  ((2.0/3.0) * (y)) + ((-1.0/3.0) * (z)) + ((-1.0/3.0) * (x)); 
    y_pos         =  ( 0.0 * y) + ((-0.57735026919) * (z)) + (0.57735026919 * (x));

    angular_pos = ((2 * pi * angular_pos) / (50.84)) ; 

    set_val (in_transform_mealy [0],x_pos);
    set_val (in_transform_mealy [1],y_pos);
    set_val (in_transform_mealy [2],angular_pos);

}

void kinematics_func(std::array<abst_ext<double>,3> &out, const std::array<abst_ext<double>,3> &in)
{
    double w, vx, vy;
    double right_speed, left_speed, left2_speed; 

    w  = from_abst_ext(in[0],0.0);
    vx = from_abst_ext(in[1],0.0);
    vy = from_abst_ext(in[2],0.0);

    left_speed  = (w * (-1) + vx ) * (-1);
    right_speed = (w * (-1)) + (vx * (-0.5)) + (vy * (-0.86602)); 
    left2_speed = (w * (-1)) + (vx * (-0.5)) + (vy * (0.86602));

    set_val (out [0],left_speed);
    set_val (out [1],right_speed);
    set_val (out [2],left2_speed);
}

SC_MODULE(top)
{

    SY::signal<double> from_wrapper1, from_wrapper2, from_wrapper3, from_wrapper4, from_wrapper5, from_wrapper6;
    SY::signal<double> to_wrapper1, to_wrapper2, to_wrapper3; 
    SY::signal<double> from_unzip1, from_unzip2, from_unzip3; 

    SY::signal<std::array<abst_ext<double>,3>> joint_state;
    SY::signal<std::array<abst_ext<double>,3>> in_transform_mealy;
    SY::signal<std::array<abst_ext<double>,3>> out_transform_mealy;
    SY::signal<std::array<abst_ext<double>,6>> to_mealy;
    SY::signal<std::array<abst_ext<double>,3>> from_mealy;
    SY::signal<std::array<abst_ext<double>,3>> from_kinematics;

    std::vector <std::string> topics_publisher=  
    {
        "/robot/left_joint_velocity_controller/command",
        "/robot/right_joint_velocity_controller/command",
        "/robot/left2_joint_velocity_controller/command"
        
    }; 

    std::vector <std::string> topics_subscriber= 
    {
        "/robot/joint_states",
        "/sonar_sonar_link_1",
        "/sonar_sonar_link_2", //right
        "/sonar_sonar_link_3"
    };

    SC_CTOR(top)
    {

        SY::make_roswrap("roswrap", topics_publisher,
                        topics_subscriber, 
                        from_wrapper1,
                        from_wrapper2, 
                        from_wrapper3, 
                        from_wrapper4, 
                        from_wrapper5, 
                        from_wrapper6,
                        to_wrapper1,
                        to_wrapper2,
                        to_wrapper3
                        );
        
        SY::make_scomb3("inverse_kinematics",
                        inv_kinematics_func, 
                        in_transform_mealy, 
                        from_wrapper1, 
                        from_wrapper2, 
                        from_wrapper3
                        );

        SY::make_smealy("controller_transform",
                transform_ns_func,
                transform_od_func,
                initial_state_transform {0.0, 0.0, 0.0, 0.0},
                out_transform_mealy,
                in_transform_mealy
                );

        auto joint_unzip = SY::make_unzipX("joint_unzip",out_transform_mealy);
        joint_unzip -> oport[0] (from_unzip1);
        joint_unzip -> oport[1] (from_unzip2);
        joint_unzip -> oport[2] (from_unzip3);
        
        auto zip_mealy = SY::make_zipX ("zip_mealy",to_mealy); 
        zip_mealy -> iport[0] (from_unzip1);
        zip_mealy -> iport[1] (from_unzip2);
        zip_mealy -> iport[2] (from_unzip3);
        zip_mealy -> iport[3] (from_wrapper4);
        zip_mealy -> iport[4] (from_wrapper5);
        zip_mealy -> iport[5] (from_wrapper6);

        SY::make_smealy("controller2",
                        controller_ns_func,
                        controller_od_func,
                        std::make_tuple(0.0, 0.0, 0.0, 0, Angle_Changer),
                        from_mealy,
                        to_mealy
                        );
        
        SY::make_scomb("kinematics",
                        kinematics_func, 
                        from_kinematics, 
                        from_mealy
                        );

        auto wrapper_unzip = SY::make_unzipX("wrapper_unzip",from_kinematics);
        wrapper_unzip -> oport[0](to_wrapper1);
        wrapper_unzip -> oport[1](to_wrapper2);
        wrapper_unzip -> oport[2](to_wrapper3);
    }

    #ifdef FORSYDE_INTROSPECTION
        void start_of_simulation()
        {
            ForSyDe::XMLExport dumper("gen/");
            dumper.traverse(this);
        }
    #endif
};

