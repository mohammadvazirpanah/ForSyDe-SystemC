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
    SY_in<T1>  iport1, iport2, iport3;       
    SY_out<T0> oport1, oport2, oport3, oport4, oport5, oport6; 

    
    //typedef std::function<void(abst_ext<T0>&,const abst_ext<T1>&)> functype;

    roswrap(const sc_module_name& _name,                    
            const std::vector <std::string>& topics_publisher,          
            const std::vector <std::string>& topics_subscriber
            //const functype& _func_callback
            ): 
            sy_process(_name),
            iport1("iport1"), iport2("iport2"), iport3("iport3"),
            oport1("oport1"), oport2("oport2"), oport3("oport3"), oport4("oport4"), oport5("oport5"), oport6("oport6"),
            topics_publisher(topics_publisher),
            topics_subscriber(topics_subscriber),
            first_run(true)
            //_func_callback(_func_callback)                 //Function to be passed for Callback function in ROS 

    {
#ifdef FORSYDE_INTROSPECTION
        //arg_vec.push_back(std::make_tuple("topic_name",topics_publisher, topics_subscriber));
#endif
    }

    std::string forsyde_kind() const {return "SY::roswrap";}


private:

std::vector <std::string> topics_publisher;
std::vector <std::string> topics_subscriber;
std::string _name;
abst_ext<T1> *ival1;
abst_ext<T1> *ival2;
abst_ext<T1> *ival3;
abst_ext<T0> *oval1; //y
abst_ext<T0> *oval2; //x
abst_ext<T0> *oval3; //angle
abst_ext<T0> *oval4; //sonar range_1
abst_ext<T0> *oval5; //sonar range_2
abst_ext<T0> *oval6; //sonar range_3
bool first_run;
//functype _func_callback;
ros::Publisher pub1, pub2, pub3;          // Publish in Prep Step
ros::Subscriber sub1,sub2,sub3,sub4;      // Subscribe in Prod Step 
ros::NodeHandle *n;                       // Node Handle For Ros Environment 
ros::Rate *rate;                          // Ros Rate 


    void Callback_Jonit(const sensor_msgs::JointState::ConstPtr& msg)
    {
        oval1->set_val (msg->position[0]);         //left2 
        oval2->set_val ((-1)*msg->position[1]);   // left   
        oval3->set_val (msg->position[2]);       //  right
        
    }

    void Callback_Sonar_1(const sensor_msgs::Range::ConstPtr& msg)
    {
      oval4->set_val(msg->range); 
    }

    void Callback_Sonar_2(const sensor_msgs::Range::ConstPtr& msg)
    {
      oval5->set_val(msg->range);
    }

    void Callback_Sonar_3(const sensor_msgs::Range::ConstPtr& msg)
    {
      oval6->set_val(msg->range);
    }
  
  
  
    void init()
    {

      ival1 = new abst_ext<T1>;
      ival2 = new abst_ext<T1>;
      ival3 = new abst_ext<T1>;
      oval1 = new abst_ext<T0>;
      oval2 = new abst_ext<T0>;
      oval3 = new abst_ext<T0>;
      oval4 = new abst_ext<T0>; 
      oval5 = new abst_ext<T0>; 
      oval6 = new abst_ext<T0>; 
      int argc;
      char **argv; 
		  ros::init(argc, argv, "forsyde");
      ros::start();
      n = new ros::NodeHandle;

      // rate = new ros::Rate(100);

      pub1 = n->advertise<std_msgs::Float64>(topics_publisher[0], 1000);
      pub2 = n->advertise<std_msgs::Float64>(topics_publisher[1], 1000);
      pub3 = n->advertise<std_msgs::Float64>(topics_publisher[2], 1000);

      sub1 = n->subscribe (topics_subscriber[0], 1000, &roswrap::Callback_Jonit, this);
      sub2 = n->subscribe (topics_subscriber[1], 1000, &roswrap::Callback_Sonar_1, this);
      sub3 = n->subscribe (topics_subscriber[2], 1000, &roswrap::Callback_Sonar_2, this);
      sub4 = n->subscribe (topics_subscriber[3], 1000, &roswrap::Callback_Sonar_3, this);
  
    }
    
    void prep()
    {

      if (first_run)
      {
        first_run = false;
        return;
      }
      
      *ival1 = iport1.read();
      *ival2 = iport2.read();
      *ival3 = iport3.read();

      std_msgs::Float64 msg1;
      std_msgs::Float64 msg2;
      std_msgs::Float64 msg3;

      msg1.data = ival1->from_abst_ext(0.0);
      msg2.data = ival2->from_abst_ext(0.0);
      msg3.data = ival3->from_abst_ext(0.0);
      
      pub1.publish(msg1);
      pub2.publish(msg2);
      pub3.publish(msg3);
      
      ros::spinOnce(); 
      
    }
    
    void exec() 
    {

    }
    
    void prod()
    {
      
      while (oval1->is_absent() || oval2->is_absent() || oval3->is_absent() || oval4->is_absent() || oval5->is_absent()  || oval6->is_absent())

      {
        ros::spinOnce(); 
        // rate->sleep();
        wait(SC_ZERO_TIME);
      }

      WRITE_MULTIPORT(oport1, *oval1)
      WRITE_MULTIPORT(oport2, *oval2)
      WRITE_MULTIPORT(oport3, *oval3)
      WRITE_MULTIPORT(oport4, *oval4)
      WRITE_MULTIPORT(oport5, *oval5)
      WRITE_MULTIPORT(oport6, *oval6)

      oval1->set_abst();
      oval2->set_abst();
      oval3->set_abst();
      oval4->set_abst(); 
      oval5->set_abst();
      oval6->set_abst();
    }
    
    void clean()
    {
      delete ival1; 
      delete ival2; 
      delete ival3;
      delete oval1;
      delete oval2; 
      delete oval3;
      delete oval4;
      delete oval5;
      delete oval6;
      delete n;
      delete rate;
      ros::shutdown();
    }

    #ifdef FORSYDE_INTROSPECTION
    void bindInfo()
    {
        boundInChans.resize(3);     
        boundInChans[0].port = &iport1;
        boundInChans[1].port = &iport2;
        boundInChans[2].port = &iport3;
        boundOutChans.resize(6);    
        boundOutChans[0].port = &oport1;
        boundOutChans[1].port = &oport2;
        boundOutChans[2].port = &oport3;
        boundOutChans[3].port = &oport4;
        boundOutChans[4].port = &oport5;
        boundOutChans[5].port = &oport6;

    }
    #endif
};



template <class T0, template <class> class OIf,
          class T1, template <class> class I1If>
inline roswrap<T0,T1>* make_roswrap(const std::string& pName,
    const std::vector <std::string>& pnames,
    const std::vector <std::string>& snames,
    //const typename roswrap<T0,T1>::functype& _func_callback,
    OIf<T0>& out1S, OIf<T0>& out2S, OIf<T0>& out3S, OIf<T0>& out4S, OIf<T0>& out5S, OIf<T0>& out6S,
    I1If<T1>& inp1S, I1If<T1>& inp2S, I1If<T1>& inp3S
    )
{
    auto p = new roswrap<T0,T1>(pName.c_str(), pnames, snames);
    
    (*p).iport1(inp1S);
    (*p).iport2(inp2S);
    (*p).iport3(inp3S);
    (*p).oport1(out1S);
    (*p).oport2(out2S);
    (*p).oport3(out3S);
    (*p).oport4(out4S);
    (*p).oport5(out5S);
    (*p).oport6(out6S);

    return p;
}

}
}

#endif
