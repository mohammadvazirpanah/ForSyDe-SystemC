#ifndef ROS_WRAPPERS_HPP
#define ROS_WRAPPERS_HPP

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

template <typename T0, typename T1>
class roswrap : public sdf_process

{
public:
    SDF_in<T1>  iport1;     
    SDF_out<T0> oport1;


    roswrap(const sc_module_name& _name,                    
            const std::vector <std::string>& topics_publisher,          
            const std::vector <std::string>& topics_subscriber
            ): 
            sdf_process(_name),
            iport1("iport1"),
            oport1("oport1"),
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
T1 *ival1; 
T0 *oval1; 

ros::Publisher pub1;         
ros::Subscriber sub1, sub2;      
ros::NodeHandle *n;                       // Node Handle For Ros Environment 
ros::Rate *rate;                          // Ros Rate (Nedded For Timed Callbacks)
bool first_run;                           // First Run Flag

/* CallBack Functions are Declerated Here */


void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  *oval1 = msg->pose.pose.position.x ;
}


void Scan_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    
  //   scan_vect = scan->ranges;
	// 	minRan = scan->range_min;
	// 	maxRan = scan->range_max;
	// 	minAng = scan->angle_min;
	// 	maxAng = scan->angle_max;

	// 	// std::cout << "minRan: " << minRan << " maxRan: " << maxRan << " minAng: " << minAng << " maxAng: " << maxAng << std::endl;

	// 	int midIdx = (int)round(scan_vect.size() / 2.0);
	// 	scan_right = scan_vect.front();
	// 	scan_mid = scan_vect[midIdx];
	// 	scan_left = scan_vect.back();

	// 	if (startup) {
	// 		if (std::isnan(scan_left))
	// 			send_left = MAX_DEPTH;
	// 		if (std::isnan(scan_mid))
	// 			send_mid = MAX_DEPTH;
	// 		if (std::isnan(scan_right))
	// 			send_right = MAX_DEPTH;
	// 		scanHistLeft.push_back(send_left);
	// 		scanHistMid.push_back(send_mid);
	// 		scanHistRight.push_back(send_right);
	// 	}
	// 	else {
	// 		if (std::isnan(scan_left)) {
	// 			if (*scanHistLeft.begin() < *scanHistLeft.rbegin()) //if was previously increasing
	// 				send_left = MAX_DEPTH;
	// 			else if (*scanHistLeft.begin() == *scanHistLeft.rbegin())
	// 				send_left = send_left;
	// 			else
	// 				send_left = MIN_DEPTH;
	// 		} 
	// 		else
	// 			send_left = scan_left;
	// 		if (std::isnan(scan_mid)) {
	// 			if (*scanHistMid.begin() < *scanHistMid.rbegin())
	// 				send_mid = MAX_DEPTH;
	// 			else if (*scanHistMid.begin() == *scanHistMid.rbegin())
	// 				send_mid = send_mid;
	// 			else
	// 				send_mid = MIN_DEPTH;
	// 		} 
	// 		else
	// 			send_mid = scan_mid;
	// 		if (std::isnan(scan_right)) {
	// 			if (*scanHistRight.begin() < *scanHistRight.rbegin())
	// 				send_right = MAX_DEPTH;
	// 			else if (*scanHistRight.begin() == *scanHistRight.rbegin())
	// 				send_right = send_right;
	// 			else
	// 				send_right = MIN_DEPTH;
	// 		} 
	// 		else
	// 			send_right = scan_right;
	// 		scanHistLeft.pop_front();
	// 		scanHistMid.pop_front();
	// 		scanHistRight.pop_front();

	// 		scanHistLeft.push_back(send_left);
	// 		scanHistMid.push_back(send_mid);
	// 		scanHistRight.push_back(send_right);
	// 	}
	// 	startup = false;
	// 	std::cout << "Send Left: " << send_left << " Send Mid: " << send_mid << " Send Right: " << send_right << std::endl;

	// 	scan_msg.data.resize(3);
	// 	scan_msg.data[0] = send_left;
	// 	scan_msg.data[1] = send_mid;
	// 	scan_msg.data[2] = send_right;
	// 	scan_info.publish(scan_msg);
	// }

}
  
  
    void init()
    {

      ival1 = new T1;
      oval1 = new T0;

      int argc;
      char **argv; 
		  ros::init(argc, argv, "ForSyDe_ROS_Wrapper");
      ros::start();
      n = new ros::NodeHandle;

      pub1 = n->advertise<geometry_msgs::Twist>(topics_publisher[0], 1000);

      sub1 = n->subscribe (topics_subscriber[0], 1000, &roswrap::Odom_Callback, this);
      // sub2 = n->subscribe (topics_subscriber[1], 1000, &roswrap::Scan_Callback, this);
  
    }
    
    void prep()
    {

      if (first_run)
      {
        first_run = false;
        return;
      }
      
      *ival1 = iport1.read();


      geometry_msgs::Twist msg1;

      msg1.linear.x = *ival1;
      // msg1.linear.y = ival1->from_abst_ext(0.0);
      //  msg1.angular.z = ival1->from_abst_ext(0.0);

      
      pub1.publish(msg1);

      
      ros::spinOnce(); 
      
    }
    
    void exec() 
    {

    }
    
    void prod()
    {
      
      while (ros::ok())

      {
        ros::spinOnce(); 
        wait(SC_ZERO_TIME);
      }

      write_multiport(oport1, *oval1);


      // oval1->set_abst();

    }
    
    void clean()
    {
      delete ival1; 

      delete oval1;

      delete n;
      delete rate;
      ros::shutdown();
    }

    #ifdef FORSYDE_INTROSPECTION
    void bindInfo()
    {
        boundInChans.resize(3);     
        boundInChans[0].port = &iport1;

        boundOutChans.resize(6);    
        boundOutChans[0].port = &oport1;


    }
    #endif
};



template <class T0, template <class> class OIf,
          class T1, template <class> class I1If>
inline roswrap<T0,T1>* make_roswrap(const std::string& pName,
    const std::vector <std::string>& pnames,
    const std::vector <std::string>& snames,
    OIf<T0>& out1S, 
    I1If<T1>& inp1S
    )
{
    auto p = new roswrap<T0,T1>(pName.c_str(), pnames, snames);
    
    (*p).iport1(inp1S);

    (*p).oport1(out1S);


    return p;
}

}
}

#endif
