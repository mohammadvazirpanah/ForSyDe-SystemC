#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <forsyde.hpp>
#include <tuple>
#include <array>

using namespace ForSyDe;
using namespace std;

typedef tuple<int,int> odom_type;               // (x,y)
typedef array<bool,4> laser_type;               // (right,up,left,down)
enum cmd_type {RIGHT, UP, LEFT, DOWN, NOP};     // move command and direction
typedef tuple<bool,int,int> maze_state;         // (initialized, x, y)
enum controller_state {INIT, MOVE, MOVECHECK, PLAN, IDLE};      // controller state
typedef tuple<cmd_type,odom_type,int> controller_monitor_state; // controller state
enum sa_state_type {NORMAL, CONTROLLER_CHECK};


#define STUCK_REPLAN 2


// copie from D* Lite code by Maxim Likhachev (CMU) and Sven Koenig (USC):

#define LARGE  1000000

#define DISPLAY                   /* display what happens (in ASCII)                                                     */
#define RANDOMIZESUCCS            /* randomize the order in which successors of a node are generated                     */
// #define RANDOMMAZE             /* whether the gridworld has random obstacles or is a maze created with dfs            */
#define WALLSTOREMOVE 4           /* number of walls to remove if RANDOMMAZE is NOT defined - infinite loop if too large */
// #define DEBUG                  /* whether debugging is on - debugging takes time but performs various checks          */
#define MAZEWIDTH 21              /* the width of the maze                                                               */
#define MAZEHEIGHT 21             /* the height of the mze                                                               */
#define MAZEDENSITY 0.25          /* percentage of blocked cells if RANDOMMAZE is defined                                */
// #define STARTCANBEBLOCKED      /* whether the goal cell of the robot can be blocked                                   */
#define RANDOMSTARTGOAL           /* whether the start and goal state are drawn randomly                                 */
#define STARTX 0                  /* x coordinate of the start cell                                                      */
#define STARTY 0                  /* y coordinate of the start cell                                                      */
#define GOALX 20                  /* x coordinate of the goal  cell                                                      */
#define GOALY 20                  /* y coordinate of the goal  cell                                                      */
#define INFORMEDSEARCH            /* use Manhattandistance rather than zero heuristics                                   */
#define RUNS 10                   /* number of different runs                                                            */
#define TIEBREAKING               /* tie breaking towards larger g-values (otherwise: smaller g-values)                  */

#define DIRECTIONS 4
static int dx[DIRECTIONS] = {1, 0, -1,  0};
static int dy[DIRECTIONS] = {0, 1,  0, -1};
static int revers[DIRECTIONS] = {2, 3, 0, 1};

#ifdef INFORMEDSEARCH
#define H(cell) (abs((cell)->y - mazegoal->y) + abs((cell)->x - mazegoal->x))
#else
#define H(cell) 0
#endif

// copied from maze.h
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
