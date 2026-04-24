#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <forsyde.hpp>
#include <tuple>
#include <array>
#include <vector>
#include <cmath>

using namespace ForSyDe;
using namespace std;

// ---------------------------------------------------------------------------
// Real-world (ROS/Gazebo) types
// ---------------------------------------------------------------------------
typedef std::array<float,5>    odom_type;        // x, y, z, z_orient, w_orient
typedef vector<float>          scan_type;        // 360 LiDAR readings (meters)
typedef std::array<float,6>    cmd_type_robot;   // linear.xyz, angular.xyz

// ---------------------------------------------------------------------------
// Abstract grid types
// ---------------------------------------------------------------------------
enum cmd_type { RIGHT=0, UP=1, LEFT=2, DOWN=3, NOP=4 };

enum ctrl_prog  { CTRL_INIT, CTRL_PLAN, CTRL_MOVE, CTRL_IDLE };
enum move_phase { M_STOPPED, M_ROTATING, M_DRIVING };

// controller_state tuple layout:
//  0  ctrl_prog        prog
//  1  move_phase       phase
//  2  cmd_type         current_cmd
//  3  float            target_yaw     (radians)
//  4  float            target_x       (meters, next cell center)
//  5  float            target_y       (meters)
//  6  int              current_gx     (current grid col)
//  7  int              current_gy     (current grid row)
//  8  bool             obstacle_detected
typedef tuple<ctrl_prog,move_phase,cmd_type,float,float,float,int,int,bool> controller_state;

#define CS_PROG(cs)    get<0>(cs)
#define CS_PHASE(cs)   get<1>(cs)
#define CS_CMD(cs)     get<2>(cs)
#define CS_TYAW(cs)    get<3>(cs)
#define CS_TX(cs)      get<4>(cs)
#define CS_TY(cs)      get<5>(cs)
#define CS_GX(cs)      get<6>(cs)
#define CS_GY(cs)      get<7>(cs)
#define CS_OBS(cs)     get<8>(cs)

const controller_state INIT_CTRL_STATE =
    make_tuple(CTRL_INIT, M_STOPPED, NOP, 0.0f, 0.5f, 0.5f, 0, 0, false);

// ---------------------------------------------------------------------------
// Monitor state
// ---------------------------------------------------------------------------
enum status_monitor { NOSTOCK, STOCK };
// array<bool,4>: obstacle in body-frame { FRONT, LEFT, BACK, RIGHT }
typedef tuple<status_monitor, std::array<bool,4>, odom_type> monitor_state;

// ---------------------------------------------------------------------------
// SAF scenario
// ---------------------------------------------------------------------------
enum scenario_type { START, NORMAL, REPLAN };

// ---------------------------------------------------------------------------
// Grid configuration
// ---------------------------------------------------------------------------
#define MAZEWIDTH   6
#define MAZEHEIGHT  6
#define STARTX      5      // actual goal column  (D*-Lite calls it "start")
#define STARTY      5      // actual goal row
#define GOALX       0      // robot initial column (D*-Lite calls it "goal")
#define GOALY       0      // robot initial row
#define CELL_SIZE   1.0f   // meters per grid cell

// ---------------------------------------------------------------------------
// D*-Lite internals (ported verbatim from dstarlite/globals.hpp)
// ---------------------------------------------------------------------------
#define LARGE         1000000
#define INFORMEDSEARCH
#define TIEBREAKING
#define DIRECTIONS    4

static int dx[DIRECTIONS] = { 1,  0, -1,  0 };  // RIGHT UP LEFT DOWN
static int dy[DIRECTIONS] = { 0,  1,  0, -1 };
static int revers[DIRECTIONS] = { 2, 3, 0, 1 };

struct cell;
typedef struct cell cell;

struct cell {
    cell *move[DIRECTIONS];
    cell *succ[DIRECTIONS];
    cell *searchtree;
    cell *trace;
    short obstacle;
    int   x, y;
    int   g, rhs;
    int   key[3];
    int   generated;
    int   heapindex;
};

// Global D*-Lite state (defined once, shared by controller + abstract_sys)
cell **maze       = nullptr;
cell  *mazestart  = nullptr;   // actual navigation goal cell (STARTX,STARTY)
cell  *mazegoal   = nullptr;   // robot's current cell in D*-Lite terminology
int    mazeiteration = 0;
int    keymodifier   = 0;
cell  *lastcell   = nullptr;
cell  *tmpcell    = nullptr;

#ifdef INFORMEDSEARCH
#define H(c) (abs((c)->y - mazestart->y) + abs((c)->x - mazestart->x))
#else
#define H(c) 0
#endif

// ---------------------------------------------------------------------------
// Sensor / motion thresholds
// ---------------------------------------------------------------------------
const float OBSTACLE_THRESH = 0.35f;   // LiDAR reading (m) that flags obstacle ahead
const float YAW_TOL         = 0.05f;   // radians — rotation done when |err| < this
const float POS_TOL         = 0.10f;   // meters  — cell arrived when dist < this
const float ANGULAR_VEL     = 0.4f;    // rad/s for rotation
const float LINEAR_VEL      = 0.2f;    // m/s for driving

// ---------------------------------------------------------------------------
// Yaw extraction helper
// ---------------------------------------------------------------------------
inline float get_yaw(const odom_type& odom) {
    return 2.0f * atan2f(odom[3], odom[4]);
}

// Normalize angle to (-pi, pi]
inline float normalize_angle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

// Target yaw for each abstract direction (robot faces East=0 by default)
inline float direction_yaw(cmd_type d) {
    switch (d) {
        case RIGHT: return 0.0f;
        case UP:    return  M_PI / 2.0f;
        case LEFT:  return  M_PI;
        case DOWN:  return -M_PI / 2.0f;
        default:    return  0.0f;
    }
}

// Grid cell center in Gazebo world coords
inline float cell_center_x(int gx) { return gx * CELL_SIZE + CELL_SIZE * 0.5f; }
inline float cell_center_y(int gy) { return gy * CELL_SIZE + CELL_SIZE * 0.5f; }

// Real-world position → grid cell index
inline int world_to_grid_x(float x) { return (int)floorf(x / CELL_SIZE); }
inline int world_to_grid_y(float y) { return (int)floorf(y / CELL_SIZE); }

#ifdef FORSYDE_SELF_REPORTING
FILE* report_pipe    = nullptr;
int   report_pipe_fd = -1;
#endif

#endif // GLOBALS_HPP
