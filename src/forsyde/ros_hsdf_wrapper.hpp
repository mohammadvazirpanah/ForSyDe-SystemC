#ifndef ROS_HSDF_WRAPPERS_HPP
#define ROS_HSDF_WRAPPERS_HPP

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sy_process.hpp"
#include "abst_ext.hpp"
#include <stdio.h>
#include <math.h>

namespace ForSyDe
{

namespace SDF
{

using namespace sc_core;

template <typename T0, typename T1, typename T2>
class sdf_roswrap : public sdf_process

{
public:
    SDF_in<T0>  iport1;     
    SDF_out<T1> oport1;
    SDF_out<T2> oport2;


    sdf_roswrap(const sc_module_name& _name,                    
            const std::vector <std::string>& topics_publisher,          
            const std::vector <std::string>& topics_subscriber
            ): 
            sdf_process(_name),
            iport1("iport1"),
            oport1("oport1"),
            oport2("oport2"),
            topics_publisher(topics_publisher),
            topics_subscriber(topics_subscriber),
            first_run(true)

    {
#ifdef FORSYDE_INTROSPECTION
        //arg_vec.push_back(std::make_tuple("topic_name",topics_publisher, topics_subscriber));
#endif
    }

    std::string forsyde_kind() const {return "SDF::roswrap";}


private:

std::vector <std::string> topics_publisher;
std::vector <std::string> topics_subscriber;
std::string _name;
T0 *ival1;                                // Input Value for Cmd_vel
T1 *oval1;                                // Output Value from Odom
T2 *oval2;                                // Output Value from Scan

ros::Publisher pub1;                      // Publisher for Cmd_vel        
ros::Subscriber sub1, sub2;               // Subscribers for Odom and Scan (Odometry and LaserScan)
ros::NodeHandle *n;                       // Node Handle For Ros Environment 
ros::Rate *rate;                          // Ros Rate (Nedded For Timed Callbacks)
bool first_run;                           // First Run Flag


/* CallBack Functions are Declerated Here */
void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{

  float x =msg->pose.pose.position.x;
  float y =msg->pose.pose.position.y;
  float z =msg->pose.pose.position.z;
  float z_orientation =msg->pose.pose.orientation.z;
  float w_orientation =msg->pose.pose.orientation.w;
  *oval1 = std::array<float,5>{x,y,z,z_orientation,w_orientation};
}



void Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{ 
    std::vector<float> scan_vect; // Vector to store the scan data
    std::copy(scan->ranges.begin(), scan->ranges.end(), std::back_inserter(scan_vect)); 
    *oval2 = scan_vect;

}
  
  
    void init()
    {

      ival1 = new T0;            // Input Value for Cmd_vel
      oval1 = new T1;            // Output Value from Odom
      oval2 = new T2;            // Output Value from Scan

      int argc;
      char **argv; 
		  ros::init(argc, argv, "ForSyDe_HSDF_ROS_Wrapper");
      ros::start();
      n = new ros::NodeHandle;
     

      //publisher and subscriber nodes are created here

      pub1 = n->advertise<geometry_msgs::Twist>(topics_publisher[0], 1000);
      sub1 = n->subscribe (topics_subscriber[0], 1000, &sdf_roswrap::Odom_Callback, this);
      sub2 = n->subscribe (topics_subscriber[1], 1000, &sdf_roswrap::Scan_Callback, this);
  
    }
    
    void prep()
    {

      if (first_run)
      {
        first_run = false;
        return;
      }
      
      *ival1 = iport1.read();       // Read Input Value for Cmd_vel from iport1 

      auto ivals = *ival1;
      double x_linear = ivals[0];
      double y_linear = ivals[1];
      double z_linear = ivals[2];

      double x_angular = ivals[3];
      double y_angular = ivals[4];
      double z_angular = ivals[5];
      
   

      //Geometry Message for Cmd_vel is created here and published
      geometry_msgs::Twist msg1;
    
      msg1.linear.x = x_linear;
      msg1.linear.y = y_linear;
      msg1.linear.z = z_linear;

      msg1.angular.x = x_angular;
      msg1.angular.y = y_angular;
      msg1.angular.z = z_angular;

      pub1.publish(msg1);

      ros::spinOnce(); 

 

     
      
      
    }
    
    void exec() 
    {
    }
    
    void prod()
    {
      
      // Wait for the callback functions to be called and the output values to be written
      while (oval1->empty() || oval2->empty())
      {
        ros::spinOnce();
        wait(SC_ZERO_TIME);
      }

      write_multiport(oport1, *oval1);  // Write Output Value from Odom callback function to oport1
      write_multiport(oport2, *oval2);  // Write Output Value from Scan callback function to oport2
      // oval1->clear();
      oval2->clear();
      



    }
    
    void clean()
    {

      delete ival1; 
      delete oval1;
      delete oval2;
      delete n;
      delete rate;
      ros::shutdown();
    }

    #ifdef FORSYDE_INTROSPECTION
    void bindInfo()
    {
        boundInChans.resize(1);     
        boundInChans[0].port = &iport1;

        boundOutChans.resize(2);    
        boundOutChans[0].port = &oport1;
        boundOutChans[1].port = &oport2;
    }
    #endif
};



template <class T0, template <class> class I1If,
          class T1, template <class> class O1If,
          class T2, template <class> class O2IF>
inline sdf_roswrap<T0,T1,T2>* make_sdf_roswrap(const std::string& pName,
    const std::vector <std::string>& pnames,
    const std::vector <std::string>& snames,
    I1If<T0>& inp1S,
    O1If<T1>& out1S,
    O2IF<T2>& out2S
    
    )
{
    auto p = new sdf_roswrap<T0,T1,T2>(pName.c_str(), pnames, snames);
    
    (*p).iport1(inp1S);

    (*p).oport1(out1S);
    (*p).oport2(out2S);



    return p;
}

}
}

#endif
