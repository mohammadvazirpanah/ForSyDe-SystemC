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

roswrap(const sc_module_name& _name, const std::string& topic_name_sender, const std::string& topic_name_receiver) : 
sy_process(_name), iport1("iport1"), oport1("oport1"), topic_name_sender(topic_name_sender), topic_name_receiver(topic_name_receiver)

    {
#ifdef FORSYDE_INTROSPECTION
        arg_vec.push_back(std::make_tuple("topic_name",topic_name_sender, topic_name_receiver));
#endif
    }

    std::string forsyde_kind() const {return "SY::roswrap";}


private:

std::string topic_name_sender;
std::string topic_name_receiver;
std::string _name;
abst_ext<T1>* ival1;
abst_ext<T0>* oval;
//T0* oval;
std::istringstream ival1_str;
std::istringstream ival2_str;

ros::Publisher pub;          // Publish in Prep Step
ros::Subscriber sub;        // Subscribe in Prod Step 
ros::NodeHandle *n;         //  Node Handle For Ros Environment 


    void Callback_Sonar(const sensor_msgs::Range::ConstPtr& msg)
    {
      float max_range = msg-> max_range ;
      float min_range = msg-> min_range ;
      float range = msg->range ; 
      oval->set_val(msg->min_range);
      ROS_INFO("Sonar Range: [%f]", oval->from_abst_ext(0.0));
    }
  
    void init()
    {

      oval = new abst_ext<T0>;
      ival1 = new abst_ext<T1>;
		  ros::init(argc, argv, "forsyde");
      ros::start();
      n = new ros::NodeHandle;
      // ival1_str.str(topic_name_sender);
      // ival2_str.str(topic_name_receiver);
      


      sub = n->subscribe ("/sonar_sonar_link_1", 1000, &roswrap::Callback_Sonar, this);
      //sub = n.subscribe ("/sonar_sonar_link_2", 1000, Callback_Sonar);
      //sub = n.subscribe ("/sonar_sonar_link_3", 1000, Callback_Sonar);

      pub = n->advertise <std_msgs::Float64> ("robot/left_joint_velocity_controller/command", 1000);
      //pub = n.advertise <std_msgs::Float64> ("robot/right_joint_velocity_controller/command", 1000); 
      //pub = n.advertise <std_msgs::Float64> ("robot/left2_joint_velocity_controller/command", 1000); 

    }
    
    void prep()
    {

      
      *ival1 = iport1.read();
      std_msgs::Float64 msg ;
      msg.data = ival1->from_abst_ext(0.0);
      pub.publish(msg);
      ros::spinOnce(); 
  
      
    }
    
    void exec() {}
    
    void prod()
    {
      
      ros::Rate rate(1);
      while (oval->is_absent())

      {
        ros::spinOnce(); 
        rate.sleep();
        wait(SC_ZERO_TIME);
      }

      WRITE_MULTIPORT(oport1, abst_ext<T0>(*oval))
      oval->set_abst();

    }
    
    void clean()
    {
      delete ival1;
      delete oval;
      delete n;
      ros::shutdown();
    }


    
};



template <class T0, template <class> class OIf,
          class T1, template <class> class I1If>
inline roswrap<T0,T1>* make_roswrap(const std::string& pName,
    const std::string& sName,
    const std::string& rName,
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
