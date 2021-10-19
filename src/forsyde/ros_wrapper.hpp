#ifndef ROS_WRAPPERS_HPP
#define ROS_WRAPPERS_HPP

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/JointState.h"
#include "sy_process.hpp"
#include "abst_ext.hpp"
#include <stdio.h>
#include <math.h>



namespace ForSyDe
{

namespace SY
{

using namespace sc_core;

template <typename T0, typename T1>
class roswrap : public sy_process

{
public:
    SY_in<T1>  iport1;       
    SY_out<T0> oport1; 
    int argc;
    char **argv; 
    
    //typedef std::function<void(abst_ext<T0>&,const abst_ext<T1>&)> functype;

    roswrap(const sc_module_name& _name,                   // Process name 
            const std::vector <std::string>& topic_publisher,          
            const std::vector <std::string>& topic_subscriber
            //const functype& _func_callback
            ): 
            sy_process(_name),
            iport1("iport1"),
            oport1("oport1"),
            topic_publisher(topic_publisher),
            topic_subscriber(topic_subscriber)
            //_func_callback(_func_callback)                 //Function to be passed for Callback function in ROS 

    {
#ifdef FORSYDE_INTROSPECTION
        arg_vec.push_back(std::make_tuple("topic_name",topic_publisher, topic_subscriber));
#endif
    }

    std::string forsyde_kind() const {return "SY::roswrap";}


private:

std::vector <std::string> topic_publisher;
std::vector <std::string> topic_subscriber;
std::string _name;
abst_ext<T1>* ival1;
abst_ext<T0> ovals;

abst_ext<T0>* rosval1;
abst_ext<T0>* rosval2;
abst_ext<T0>* rosval3;
abst_ext<T0>* transformed_y;
abst_ext<T0>* transformed_x;
abst_ext<T0>* angle_radian;

//functype _func_callback;
std::istringstream ival1_str;
std::istringstream ival2_str;

ros::Publisher pub;          // Publish in Prep Step
ros::Subscriber sub1,sub2,sub3,sub4;        // Subscribe in Prod Step 
ros::NodeHandle *n;         // Node Handle For Ros Environment 
ros::Rate *rate;            // Ros Rate 


    void Callback_Jonit(const sensor_msgs::JointState::ConstPtr& msg)
    {
        ROS_INFO("Joint State left_pos: [%f]", -msg->position[0]);
        ROS_INFO("Joint State left2_pos: [%f]", msg->position[1]);
        ROS_INFO("Joint State right_pos: [%f]", msg->position[2]);

        double x_pos, y_pos, angular_pos;

        double left_pos =(-1.0) * msg->position[0];
        double left2_pos = msg->position[1];
        double right_pos = msg->position[2];

        angular_pos   =  ((-1.0)*(1.0/3.0) * (left2_pos)) + ((-1.0)*(1.0/3.0) * (right_pos)) + ((-1.0)*(1.0/3.0) * (left_pos));
        x_pos         =  ((2.0/3.0) * (left2_pos)) + ((-1.0)*(1.0/3.0) * (right_pos)) + ((-1.0)*(1.0/3.0) * (left_pos)); 
        y_pos         =  ( 0.0 * left2_pos) + ((-1.0)*(0.57735026919) * (right_pos)) + (0.57735026919 * (left_pos));

        double cur_x, cur_y, last_in_y, last_in_x, added_y, added_x;
        if(cur_x == 0)
        {
          cur_x = 0.0;
          cur_y = 0.0;
          last_in_x = 0.0;
          last_in_y = 0.0;
        }

        added_y = (y_pos - last_in_y)*cos((-1)*angular_pos) + (x_pos - last_in_x)*sin((-1)*angular_pos);
        added_x = -1*(y_pos - last_in_y)*sin(-angular_pos) + (x_pos - last_in_x)*cos((-1)*angular_pos);
        cur_x = cur_x + added_x;
        cur_y = cur_y + added_y;
        last_in_x = x_pos;
        last_in_y = y_pos;
        transformed_y ->set_val (cur_y);
        transformed_x -> set_val (cur_x);
        double temp = ((2 *3.14159265359 * angular_pos) / (50.84)) + (3.14159265359/3.0) ; 
        angle_radian ->set_val (temp);

        // ROS_INFO("Joint State x_pos: [%f]", x_pos);
        // ROS_INFO("Joint State y_pos: [%f]", y_pos);
        // ROS_INFO("Joint State angular_pos: [%f]",angular_pos);
        
    }

    void Callback_Sonar_1(const sensor_msgs::Range::ConstPtr& msg)
    {
      rosval1->set_val(msg->range);
      // ROS_INFO("Sonar Range 1: [%f]", rosval1->from_abst_ext(0.0));
    }

    void Callback_Sonar_2(const sensor_msgs::Range::ConstPtr& msg)
    {
      rosval2->set_val(msg->range);
      // ROS_INFO("Sonar Range 2: [%f]", rosval2->from_abst_ext(0.0));
    }

    void Callback_Sonar_3(const sensor_msgs::Range::ConstPtr& msg)
    {
      rosval3->set_val(msg->range);
      // ROS_INFO("Sonar Range 3: [%f]", rosval3->from_abst_ext(0.0));
    }
  
  
  
    void init()
    {

      //oval = new abst_ext<T0>;
      //oval = new std::vector<T0>;
      ival1 = new abst_ext<T1>;
      rosval1 = new abst_ext<T0>;
      rosval2 = new abst_ext<T0>;
      rosval3 = new abst_ext<T0>;
      transformed_y = new abst_ext<T0>; 
      transformed_x = new abst_ext<T0>; 
      angle_radian = new abst_ext<T0>; 

		  ros::init(argc, argv, "forsyde");
      ros::start();
      n = new ros::NodeHandle;
      rate = new ros::Rate(1);
      
      // ival1_str.str(topic_name_sender);
      // ival2_str.str(topic_name_receiver);
      


      sub1 = n->subscribe ("/sonar_sonar_link_1", 1000, &roswrap::Callback_Sonar_1, this);
      sub2 = n->subscribe ("/sonar_sonar_link_2", 1000, &roswrap::Callback_Sonar_2, this);
      sub3 = n->subscribe ("/sonar_sonar_link_3", 1000, &roswrap::Callback_Sonar_3, this);
      sub4 = n->subscribe ("/robot/joint_states", 1000, &roswrap::Callback_Jonit, this);


      //sub = n.subscribe ("/sonar_sonar_link_2", 1000, Callback_Sonar);
      //sub = n.subscribe ("/sonar_sonar_link_3", 1000, Callback_Sonar);

      // for (int i=0 ; i<topic_subscriber.size(); i++)
      
        //sub = n->subscribe (topic_subscriber[0], 1000, &roswrap::_func_callback, this);
        
      
      
      // for (int j=0 ; j<topic_subscriber.size(); j++)
      
        //pub = n->advertise <std_msgs::Float64> (topic_publisher[0], 1000);
      

  


    }
    
    void prep()
    {

      
      *ival1 = iport1.read();
      std_msgs::Float64 msg ;
      msg.data = ival1->from_abst_ext(0.0);
      //pub.publish(msg);
      ros::spinOnce(); 
  
      
    }
    
    void exec() {}
    
    void prod()
    {
      
      
      while (rosval1->is_absent() || rosval2->is_absent() || rosval3->is_absent() || transformed_y->is_absent() || transformed_x->is_absent()  || angle_radian->is_absent())

      {
        ros::spinOnce(); 
        rate->sleep();
        wait(SC_ZERO_TIME);
      }

      WRITE_MULTIPORT(oport1, ovals)
      rosval1->set_abst();
      rosval2->set_abst();
      rosval3->set_abst();
      transformed_y ->set_abst(); 
      transformed_x ->set_abst();
      angle_radian-> set_abst();
    }
    
    void clean()
    {
      delete ival1;
      //delete oval;
      delete n;
      delete rate;
      ros::shutdown();
    }


    
};



template <class T0, template <class> class OIf,
          class T1, template <class> class I1If>
inline roswrap<T0,T1>* make_roswrap(const std::string& pName,
    const std::vector <std::string>& sName,
    const std::vector <std::string>& rName,
    //const typename roswrap<T0,T1>::functype& _func_callback,
    OIf<T0>& outS,
    I1If<T1>& inp1S
    )
{
    auto p = new roswrap<T0,T1>(pName.c_str(), sName, rName);
    
    (*p).iport1(inp1S);
    (*p).oport1(outS);
    
    return p;
}




}
}

#endif


    // void Callback_Sonar(const sensor_msgs::Range::ConstPtr& msg)
    // {
    //   float max_range = msg-> max_range ;
    //   float min_range = msg-> min_range ;
    //   float range = msg->range ; 
    //   oval->set_val(msg->range);
    //   ROS_INFO("Sonar Range: [%f]", oval->from_abst_ext(0.0));
    // }