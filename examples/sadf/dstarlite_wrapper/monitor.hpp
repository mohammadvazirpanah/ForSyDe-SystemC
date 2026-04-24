
#ifndef MONITOR_HPP
#define MONITOR_HPP

#include <forsyde.hpp>
#include <tuple>
#include <vector>
#include "globals.hpp"


using namespace std;

  
bool checkObjectInRange(const std::vector<float>& scan, int start_idx, int end_idx, float threshold_distance) 
{
      for (int i = start_idx; i <= end_idx; ++i) 
      {
          float range = scan[i];
          if (range > scan_range_min && range < scan_range_max && range < threshold_distance) 
          {
              return true; // Object detected within threshold distance
          }
      }
      return false; // No object detected within threshold distance
}
void monitor_func(tuple<vector<monitor_state>>& outs,
                    const tuple<vector<scan_type>,vector <odom_type>>& inps)
{
    status_monitor status; 
    float threshold_distance = 0.75; 
    
    auto scan_data_vect = std::get<0>(inps)[0];
    auto odom_data_arr  = std::get<1>(inps)[0];



    bool object_front_1 = checkObjectInRange(scan_data_vect, 0, 25, threshold_distance);
    bool object_front_2 = checkObjectInRange(scan_data_vect, 335, 359, threshold_distance);
    bool object_front = object_front_1 || object_front_2;
    bool object_left = checkObjectInRange(scan_data_vect, 25, 135, threshold_distance);
    bool object_back = checkObjectInRange(scan_data_vect, 135, 225, threshold_distance);
    bool object_right = checkObjectInRange(scan_data_vect, 225, 315, threshold_distance);
        
   
    if (object_front)
    {
        std::cout<<"Object detected in front"<<std::endl;
        status = STOCK;
    }
    else 
    {
        std::cout<<"No object detected in front"<<std::endl;
        status = NOSTOCK;
    }
    
    
    // std::cout<<"object_front = "<<object_front<<std::endl;
    // std::cout<<"object_back = "<<object_back<<std::endl;
    // std::cout<<"object_left = "<<object_left<<std::endl;
    // std::cout<<"object_right = "<<object_right<<std::endl;

    // Print object detections status
    std::cout << "Object detected:\n";
    std::cout << "Front: " << (object_front ? "Yes" : "No") << std::endl;
    std::cout << "Back: " << (object_back ? "Yes" : "No") << std::endl;
    std::cout << "Left: " << (object_left ? "Yes" : "No") << std::endl;
    std::cout << "Right: " << (object_right ? "Yes" : "No") << std::endl;
    
    array <bool,4> directions = {object_front, object_back, object_left, object_right};
    tuple <status_monitor,four_directions, odom_type> monitor_output = make_tuple(status, directions, odom_data_arr);
    std::get<0>(outs)[0] = monitor_output;
}

#endif
