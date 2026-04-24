#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <forsyde.hpp>
#include <tuple>
#include <array>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

using namespace ForSyDe;
using namespace std;

double scan_range_min = 0.119;
double scan_range_max = 6.0;
typedef array<float,5> odom_type;              
typedef std::vector<float> scan_type;
enum controller_program {INITCONTROLL, MOVE, MOVECHECK, PLAN, IDLE}; 
typedef array <bool,4> four_directions;                                         //Front, Back, Left, Right
enum status_monitor {INIT,STOCK,NOSTOCK};                                      //STOCK, NOSTOCK
typedef tuple<status_monitor,four_directions,odom_type> monitor_state;        //status_monitor, 
typedef tuple<controller_program, odom_type> controller_state;               //controller_state, odom_type 

const int GRID_SIZE = 5;        // Size of the grid map (assuming a 5x5 grid for Gazebosim)
const double CELL_SIZE = 1.0;  // Size of each grid cell in meters

enum cmd_type {RIGHT, UP, LEFT, DOWN, NOP}; 
typedef array<float,6> cmd_type_robot; 
typedef tuple<cmd_type_robot, double> controller_move_state;              //cmd_type, goal

//enum sa_state_type {INITIAL ,NORMAL, CONTROLLER_CHECK};
typedef std::vector<std::tuple<float, float>> path;                       //X,Y 

enum contorller_scenario_type {START, NORMAL, CONTROLLER_CHECK};

struct GridCell {
    bool obstacle;  // Indicates whether the cell is an obstacle or not
    // Other properties of the grid cell can be added here if needed
};







#define LARGE  1000000

#define DISPLAY                   /* display what happens (in ASCII)                                                     */
#define RANDOMIZESUCCS            /* randomize the order in which successors of a node are generated                     */
// #define RANDOMMAZE             /* whether the gridworld has random obstacles or is a maze created with dfs            */
#define WALLSTOREMOVE 4           /* number of walls to remove if RANDOMMAZE is NOT defined - infinite loop if too large */
// #define DEBUG                  /* whether debugging is on - debugging takes time but performs various checks          */
#define MAZEWIDTH 6               /* the width of the maze                                                               */
#define MAZEHEIGHT 6              /* the height of the mze                                                               */
#define MAZEDENSITY 0.25          /* percentage of blocked cells if RANDOMMAZE is defined                                */
// #define STARTCANBEBLOCKED      /* whether the goal cell of the robot can be blocked                                   */
#define RANDOMSTARTGOAL           /* whether the start and goal state are drawn randomly                                 */
#define STARTX 2                  /* x coordinate of the start cell                                                      */
#define STARTY 2                  /* y coordinate of the start cell                                                      */
#define GOALX 2                  /* x coordinate of the goal  cell                                                      */
#define GOALY -4                  /* y coordinate of the goal  cell                                                      */
#define INFORMEDSEARCH            /* use Manhattandistance rather than zero heuristics                                   */
#define RUNS 10                   /* number of different runs                                                            */
#define TIEBREAKING               /* tie breaking towards larger g-values (otherwise: smaller g-values)                  */

#define DIRECTIONS 4
static int dx[DIRECTIONS] = {1, 0, -1,  0};
static int dy[DIRECTIONS] = {0, 1,  0, -1};
static int reverse[DIRECTIONS] = {2, 3, 0, 1};

#ifdef INFORMEDSEARCH
#define H(cell) (abs((cell)->y - mazegoal->y) + abs((cell)->x - mazegoal->x))
#else
#define H(cell) 0
#endif


struct cell;
typedef struct cell cell;

struct cell
{
    cell *move[DIRECTIONS];
    cell *succ[DIRECTIONS];
    cell *searchtree;
    cell *trace;
    short obstacle;
    int x, y;
    int dfsx, dfsy; /* needed only for generating dfs mazes */
    int g;
    int rhs;
    int key[3];
    int generated;
    int heapindex;
};

int mazeiteration = 0; //TODO:remove
int keymodifier; //TODO:remove


#endif




