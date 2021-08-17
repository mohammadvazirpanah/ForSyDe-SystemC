

#include <forsyde.hpp>

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

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void Callback_Sonar(const sensor_msgs::Range::ConstPtr& msg)
{
 
  double max_range = msg-> max_range ;
  double min_range = msg-> min_range ;
  double rang = msg->range ; 
  
  
  //ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
  ROS_INFO("Sonar Range: [%f]", msg->range);

}

void callback(const sensor_msgs::Range::ConstPtr &in1, const sensor_msgs::Range::ConstPtr &in2)
{
  ROS_INFO("Synchronization successful");
}

void Callback_Jonit(const sensor_msgs::JointState::ConstPtr& msg)
{
    //ROS_INFO("Joint State: [%f]", msg->position[1]);

    double x = msg->position[1];
    double y = msg->position[2];
    double z = msg->position[3];
}

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

T0* oval;
std::istringstream ival1_str;
std::istringstream ival2_str;

ros::Publisher chatter_pub; // Publish in Prep Step
ros::Subscriber sub;        // Subscribe in Prod Step 

//ros::Subscriber sub = n.subscribe("receive", 1000, chatterCallback);
   
    void init()
    {
      
      oval = new T0;
      ival1 = new abst_ext<T1>;
		  ros::init(argc, argv, "forsyde");
      ros::start();
      ival1_str.str(topic_name_sender);
      ival2_str.str(topic_name_receiver);
      ros::NodeHandle n;
      sub = n.subscribe ("/sonar_sonar_link_1", 1000, Callback_Sonar);
  
      //chatter_pub = n.advertise<std_msgs::String>(ival1_str.str(), 1000);

      //chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000); turtle1/cmd_vel
      //chatter_pub = n.advertise<std_msgs::Float64>("robot/left_joint_velocity_controller/command", 1000);   
    
    }
    
    void prep()
    {


      ros::Rate loop_rate(1);
      *ival1 = iport1.read(); // Read input 
      //std_msgs::String msg; 
      std::stringstream ss;
      //ros::NodeHandle n;

      //ss << *ival1;
      //ss << "/cmd_vel geometry_msgs/Twist linear: '[10, 0, 0]' '[0, 0, 5]'"; 

      //msg.data = ss.str(); 
      //ROS_INFO("%s", msg.data.c_str());
    
      //geometry_msgs::Twist msg; 
      //std_msgs::Float64 msg1 ;
      // msg.linear.x = 0.2;
      // msg.linear.y = 0;
	    // msg.linear.z = 0;
      // msg1.data = 5.0;
      //msg.angular.z = 5;
      int flag = 1;
    
      //message_filters::Subscriber<sensor_msgs::Image> sub_1_;
      //message_filters::Subscriber<sensor_msgs::Image> sub_2_;

 
      
      //sub = n.subscribe ("/robot/joint_states", 1000, Callback_Jonit);
      //chatter_pub.publish(msg1);
      ros::spin();

      loop_rate.sleep();
    
      //wait(SC_ZERO_TIME);


     

      
      
      
     
      //std::cout<<"Exit";
      
    
      
    }
    
    void exec() {}
    
    void prod()
    {

      //ros::NodeHandle n;
      //sub = n.subscribe(ival2_str.str(), 1000, chatterCallback);
      //ROS_INFO("%s", msg.data.c_str());
      //std::cout<<ival2_str.str()<<"\n";
      //WRITE_MULTIPORT(oport1, abst_ext<T0>(*oval))
    }
    
    void clean()
    {
      delete ival1;
      delete oval;
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












