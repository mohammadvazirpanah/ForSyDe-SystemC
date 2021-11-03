#include <forsyde.hpp>
#include "forsyde/ros_wrapper.hpp"
#include <iostream>
#include <controller2.hpp>
#include "sensor_msgs/Range.h"
#include <math.h>

using namespace sc_core;
using namespace ForSyDe;

void inv_kinematics_func (std::array<abst_ext<double>,3>& joint_state, const double& x, const double& y, const double& angular)
{
    double x_pos, y_pos, angular_pos, in_angular_pos;
    double cur_x, cur_y, last_in_y, last_in_x, added_y, added_x, angle_radian;
    
    // double left_pos, left2_pos, right_pos;
    // left_pos  = x;
    // left2_pos = y;
    // right_pos = angular;
    
    angular_pos   =  ((-1.0)*(1.0/3.0) * (y)) + ((-1.0)*(1.0/3.0) * (angular)) + ((-1.0)*(1.0/3.0) * (x));
    x_pos         =  ((2.0/3.0) * (y)) + ((-1.0)*(1.0/3.0) * (angular)) + ((-1.0)*(1.0/3.0) * (x)); 
    y_pos         =  ( 0.0 * y) + ((-1.0)*(0.57735026919) * (angular)) + (0.57735026919 * (x));

    in_angular_pos = ((2 *3.14159265359 * angular_pos) / (50.84)) ; 

    if(cur_x == 0)
    {
        cur_x = 0.0;
        cur_y = 0.0;
        last_in_x = 0.0;
        last_in_y = 0.0;
    }

    added_y = (y_pos - last_in_y) * cos((-1)*in_angular_pos) + (x_pos - last_in_x) * sin((-1)*in_angular_pos);
    added_x = -1*(y_pos - last_in_y) * sin(-in_angular_pos) + (x_pos - last_in_x) * cos((-1)*in_angular_pos);
    cur_x = cur_x + added_x;
    cur_y = cur_y + added_y;
    last_in_x = x_pos;
    last_in_y = y_pos;
    angle_radian = ((2 *3.14159265359 * angular_pos) / (50.84)) + (3.14159265359/3.0) ;      

    joint_state [0] = cur_y; //transformed_y
    joint_state [1] = cur_x; //transformed_x
    joint_state [2] = angle_radian; 

}

void kinematics_func(std::array<abst_ext<double>,3> &out, const std::array<abst_ext<double>,3> &in)
{
    double w, vx, vy;
    double right_speed, left_speed, left2_speed; 

    w = in [0];
    vx = in [1];
    vy = in [2];

    right_speed = w * (-1) + vx * (-0.5) + vy * (-0.86602); 
    left_speed  = (w * (-1) + vx ) * (-1);
    left2_speed = w * (-1) + vx * (-0.5) + vy * (0.86602);

    out [0] = right_speed; 
    out [1] = left_speed;    
    out [2] = left2_speed;    

}

SC_MODULE(top)
{

    SY::signal<double> from_wrapper1, from_wrapper2, from_wrapper3, from_wrapper4, from_wrapper5, from_wrapper6;
    SY::signal<double> to_wrapper1, to_wrapper2, to_wrapper3; 
    SY::signal<double> from_unzip1, from_unzip2, from_unzip3; 

    SY::signal<std::array<abst_ext<double>,3>> joint_state;
    SY::signal<std::array<abst_ext<double>,6>> to_mealy;
    SY::signal<std::array<abst_ext<double>,3>> from_mealy;
    SY::signal<std::array<abst_ext<double>,3>> from_kinematics;

    std::vector <std::string> topics_publisher=  
    {
        "robot/left_joint_velocity_controller/command",
        "robot/right_joint_velocity_controller/command",
        "robot/left2_joint_velocity_controller/command"
    }; 

    std::vector <std::string> topics_subscriber= 
    {
        "/robot/joint_states",
        "/sonar_sonar_link_1",
        "/sonar_sonar_link_2",
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
                        joint_state, 
                        from_wrapper1, 
                        from_wrapper2, 
                        from_wrapper3
                        );

        auto joint_unzip = SY::make_unzipX("joint_unzip",joint_state);
        joint_unzip -> oport[0](from_unzip1);
        joint_unzip -> oport[1](from_unzip2);
        joint_unzip -> oport[2](from_unzip3);
        
        auto zip_mealy = SY::make_zipX ("zip_mealy",to_mealy); 
        zip_mealy -> iport[0] (from_unzip1);
        zip_mealy -> iport[1] (from_unzip2);
        zip_mealy -> iport[2] (from_unzip3);
        zip_mealy -> iport[3] (from_wrapper4);
        zip_mealy -> iport[4] (from_wrapper5);
        zip_mealy -> iport[5] (from_wrapper6);

        SY::make_smealy("controller1",
                        controller_ns_func,
                        controller_od_func,
                        std::make_tuple(0.0, 0.0, 0.0),  
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


};

