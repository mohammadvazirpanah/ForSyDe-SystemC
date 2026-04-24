/**********************************************************************
    * maze.hpp -- grid maze allocation and D*-Lite graph setup       *
    *                                                                 *
    * Author:  Mohammad Vazirpanah (mohammad.vazirpanah@yahoo.com)    *
    *                                                                 *
    * Purpose: Allocates the 2-D grid, initialises successor edges,   *
    *          and provides the D*-Lite initialisation routine called  *
    *          once at simulation start by the controller.            *
    *                                                                 *
    * Usage:   D*-Lite path planning on TurtleBot3 via ROS/Gazebo     *
    *                                                                 *
    * License: BSD3                                                   *
    *******************************************************************/
#ifndef MAZE_HPP
#define MAZE_HPP

#include "globals.hpp"
#include "heap.hpp"

// ---------------------------------------------------------------------------
// Allocate the 6x6 grid, set up successor edges, no obstacles known yet.
// Call once at simulation start (from controller CTRL_INIT).
// ---------------------------------------------------------------------------
void preprocessmaze()
{
    if (maze != nullptr) return;   // already done

    maze = (cell **)calloc(MAZEHEIGHT, sizeof(cell *));
    for (int y = 0; y < MAZEHEIGHT; ++y)
        maze[y] = (cell *)calloc(MAZEWIDTH, sizeof(cell));

    for (int y = 0; y < MAZEHEIGHT; ++y)
        for (int x = 0; x < MAZEWIDTH; ++x) {
            maze[y][x].x = x;
            maze[y][x].y = y;
            for (int d = 0; d < DIRECTIONS; ++d) {
                int ny = y + dy[d];
                int nx = x + dx[d];
                maze[y][x].succ[d] = (ny >= 0 && ny < MAZEHEIGHT &&
                                      nx >= 0 && nx < MAZEWIDTH)
                                     ? &maze[ny][nx] : nullptr;
            }
        }

    mazestart = &maze[STARTY][STARTX];   // actual goal (5,5)
    mazegoal  = &maze[GOALY][GOALX];     // robot starts at (0,0)
}

// ---------------------------------------------------------------------------
// Copy succ[] → move[] for all cells (robot knows no obstacles initially).
// ---------------------------------------------------------------------------
void postprocessmaze()
{
    for (int y = 0; y < MAZEHEIGHT; ++y)
        for (int x = 0; x < MAZEWIDTH; ++x)
            for (int d = 0; d < DIRECTIONS; ++d)
                maze[y][x].move[d] = maze[y][x].succ[d];
}

// ---------------------------------------------------------------------------
// D*-Lite initialize — sets up the priority queue with mazestart.
// ---------------------------------------------------------------------------
void dstar_initialize()
{
    ++mazeiteration;
    keymodifier = 0;

    for (int y = 0; y < MAZEHEIGHT; ++y)
        for (int x = 0; x < MAZEWIDTH; ++x) {
            maze[y][x].g          = LARGE;
            maze[y][x].rhs        = LARGE;
            maze[y][x].searchtree = nullptr;
            maze[y][x].trace      = nullptr;
            maze[y][x].generated  = 0;
            maze[y][x].heapindex  = 0;
        }

    mazestart->rhs        = 0;
    mazestart->g          = LARGE;
    mazestart->searchtree = nullptr;
    mazestart->generated  = mazeiteration;

#ifdef TIEBREAKING
    emptyheap(3);
    mazestart->key[0] = H(mazestart);
    mazestart->key[1] = H(mazestart) + 1;
    mazestart->key[2] = H(mazestart);
#else
    emptyheap(2);
    mazestart->key[0] = H(mazestart);
    mazestart->key[1] = 0;
#endif
    insertheap(mazestart);

    mazegoal->g          = LARGE;
    mazegoal->rhs        = LARGE;
    mazegoal->generated  = mazeiteration;
    mazegoal->searchtree = nullptr;
}

// ---------------------------------------------------------------------------
// Full maze setup: allocate + postprocess + D*-Lite init.
// ---------------------------------------------------------------------------
void init_maze()
{
    preprocessmaze();
    postprocessmaze();
    dstar_initialize();
}

// ---------------------------------------------------------------------------
// ASCII debug print (optional, guards on DISPLAY)
// ---------------------------------------------------------------------------
#ifdef DISPLAY
void printknownmaze(FILE *output)
{
    for (int x = 0; x < MAZEWIDTH + 2; ++x) fprintf(output, "X");
    fprintf(output, "\n");
    for (int y = MAZEHEIGHT - 1; y >= 0; --y) {
        fprintf(output, "X");
        for (int x = 0; x < MAZEWIDTH; ++x) {
            if      (&maze[y][x] == mazestart) fprintf(output, "G");
            else if (&maze[y][x] == mazegoal)  fprintf(output, "R");
            else if (maze[y][x].obstacle)       fprintf(output, "X");
            else {
                bool on_path = false;
                for (cell *c = mazegoal; c && c != mazestart; c = c->searchtree)
                    if (c == &maze[y][x]) { on_path = true; break; }
                fprintf(output, on_path ? "." : " ");
            }
        }
        fprintf(output, "X\n");
    }
    for (int x = 0; x < MAZEWIDTH + 2; ++x) fprintf(output, "X");
    fprintf(output, "\n\n");
    fflush(output);
}
#endif

#endif // MAZE_HPP
