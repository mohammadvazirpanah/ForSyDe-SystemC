
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>
#include <tuple>
#include <vector>
#include "globals.hpp"
// #include "maze.hpp"

using namespace std;

path shortestPath =
{
    std::make_tuple(1, 1), 
    std::make_tuple(2, 1), 
    std::make_tuple(2, 2), 
    std::make_tuple(3, 2), 
    std::make_tuple(4, 2), 
    std::make_tuple(4, 3), 
    std::make_tuple(4, 4), 
    std::make_tuple(4, 5), 
    std::make_tuple(5, 5)
};
    
typedef map<
            contorller_scenario_type,
            tuple<array<size_t,3>,array<size_t,2>>
        > controller_table_type;

controller_table_type controller_table = 
{  
    {START,            make_tuple(array<size_t,3>{1,1,1},array<size_t,2>{1,1})},
    {NORMAL,           make_tuple(array<size_t,3>{0,0,0},array<size_t,2>{0,0})},
    {CONTROLLER_CHECK, make_tuple(array<size_t,3>{1,1,1},array<size_t,2>{1,1})}
};

void controller_func(tuple<
                                vector<cmd_type_robot>,
                                vector<controller_state>
                                >& outs,
                                const contorller_scenario_type& contorller_scenario,
                                const tuple<
                                vector<path>,
                                vector<monitor_state>,
                                vector<controller_state>
                                >& inps)
{

    //monitor_state:status_monitor,four_directions,odom_type

//     auto status_monitor         =   get<0>((get<0>(inps)[0]));
//     auto four_directions        =   get<1>((get<0>(inps)[0]));
//     auto odom_data              =   get<2>((get<0>(inps)[0]));
//     auto controller_st          =   get<0>(get<1>(inps)[0]);
//     auto current_x              =   odom_data[0];
//     auto current_y              =   odom_data[1];
//     auto current_z              =   odom_data[2];
//     auto current_z_orientation  =   odom_data[3];
//     auto current_w_orientation  =   odom_data[4];

//     auto curr_z_round = std::round(current_z_orientation * 100) / 100;
//     auto curr_w_round = std::round(current_w_orientation * 100) / 100;

    


//     switch (controller_st)
//     {
//          case INIT:
//         {
           
//         float targetX, targetY;
//         if (shortestPath.empty()) {
//         // No more waypoints or no shortest path, stop moving
//         moveCommands.clear();
    
//         return;
//         std::tie(targetX, targetY) = point;

//         int deltaX = targetX - current_x; // current_x باید مقدار فعلی ربات باشد
//         int deltaY = targetY - current_y; // current_y باید مقدار فعلی ربات باشد

//             get<0>(outs)[0] = NOP;
//             get<1>(outs)[0] = PLAN;
//             break;
//             // if (cmd==UP)
//             // {
//             //     goal = {current_x+1, current_y, current_z};
//             // }
            
//             // else if (cmd==DOWN)
//             // {
//             //     goal = {current_x-1, current_y, current_z};
//             // }
//             // else if (cmd==LEFT)
//             // {
//             //     goal = {current_x, current_y-1, current_z};
//             // }
//             // else if (cmd==RIGHT)
//             // {
//             //     goal = {current_x, current_y+1, current_z};
//             // }

//             get<0>(get<1>(outs)[0]) = MOVE;
//             get<1>(get<1>(outs)[0]) = goal;

//             break;
//         }
//         case MOVE:
//         {
//             if (cmd==UP)
//             {
//                 if (current_x <= goal[1])
//                 {
//                     get<0>(outs)[0] = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
//                     get<0>(get<1>(outs)[0]) = MOVE;
//                 }
//                 else
//                 {
//                     get<0>(outs)[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//                     get<0>(get<1>(outs)[0]) = IDLE;
//                 }
//             }
//             if (cmd==DOWN)
//             {
//                 if (current_x >= goal[1])
//                 {
//                     get<0>(outs)[0] = {-0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
//                     get<0>(get<1>(outs)[0]) = MOVE;
//                 }
//                 else
//                 {
//                     get<0>(outs)[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//                     get<0>(get<1>(outs)[0]) = IDLE;
//                 }
//             }
//             if (cmd==RIGHT)
//             {
//                 if (curr_z_round <= curr_w_round)
//                 {
//                     get<0>(outs)[0] = {0.0, 0.0, 0.0, 0.0, 0.0, -0.2};
//                     get<0>(get<1>(outs)[0]) = MOVE;
//                     break;
//                 }
//                 else if (( std::abs(curr_z_round) == curr_w_round) || std::abs(curr_z_round) == 0.7 || curr_w_round == 0.7)
//                 {
//                     get<0>(outs)[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//                     get<0>(get<1>(outs)[0]) = IDLE;
//                     break; 
//                 }
//             }
//             if (cmd==LEFT)
//             {
//                 if (curr_z_round <= curr_w_round)
//                 {
//                     get<0>(outs)[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.2};
//                     get<0>(get<1>(outs)[0]) = MOVE;
//                     break; 
//                 }
//                 else if ((curr_z_round == curr_w_round) || curr_z_round == 0.7 || curr_w_round == 0.7)
//                 {
//                     get<0>(outs)[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//                     get<0>(get<1>(outs)[0]) = IDLE;
//                     break;
//                 }
//             }

//             break;
//         }
//         case IDLE:
//         {
//             get<0>(outs)[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//             get<0>(get<1>(outs)[0]) = IDLE;
//             break;
//         }
  
//         default:
//         {
//             get<0>(outs)[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//             get<0>(get<1>(outs)[0]) = IDLE;
//             break;
//         }
//     }
//     cout<<"[Controller] command: "<<get<0>(outs)[0]<<endl;
// 

}   
    


#endif
