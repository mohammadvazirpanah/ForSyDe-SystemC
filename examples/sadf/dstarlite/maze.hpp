
#ifndef MAZE_HPP
#define MAZE_HPP

#include <forsyde.hpp>
#include <tuple>
#include <vector>
#include "globals.hpp"
#include "heap.hpp"

using namespace std;

// copied from maze.c by Maxim Likhachev (CMU) and Sven Koenig (USC):

cell **maze = NULL;
cell *mazestart, *mazegoal; //TODO:remove

int goaly = GOALY;
int goalx = GOALX;
int starty = STARTY;
int startx = STARTX;

void printactualmaze(FILE *output)
{
    int x, y;

    for (x = 0; x < MAZEWIDTH+2; ++x)
	fprintf(output, "X");
    fprintf(output, "\n");
    for (y = 0; y < MAZEHEIGHT; ++y)
    {
	fprintf(output, "X");
	for (x = 0; x < MAZEWIDTH; ++x)
	{
	    if (mazegoal == &maze[y][x])
		fprintf(output, "R");
	    else if (mazestart == &maze[y][x])
		fprintf(output, "G");		
	    else if (maze[y][x].obstacle)
		fprintf(output, "X");
	    else
		fprintf(output, " ");
	}
	fprintf(output, "X\n");
    }
    for (x = 0; x < MAZEWIDTH+2; ++x)
	fprintf(output, "X");
    fprintf(output, "\n\n\n");
}

void printknownmaze(FILE *output)
{
    int x, y, d;
    static char **display = NULL;
    cell *tmpcell;

    if (display == NULL)
    {
	display = (char **)calloc(MAZEHEIGHT, sizeof(char *));
	for (y = 0; y < MAZEHEIGHT; ++y)
	    display[y] = (char *)calloc(MAZEWIDTH, sizeof(char));
    }
    for (y = 0; y < MAZEHEIGHT; ++y)
	for (x = 0; x < MAZEWIDTH; ++x)
	    {
		display[y][x] = 'X';
		for (d = 0; d < DIRECTIONS; ++d)
		    if (maze[y][x].move[d])
			display[y][x] = ' ';
	    }
    for (tmpcell = mazegoal; tmpcell != mazestart; tmpcell = tmpcell->searchtree)
	display[tmpcell->y][tmpcell->x] = '.';
    display[mazestart->y][mazestart->x] = 'G';
    display[mazegoal->y][mazegoal->x] = 'R';
    for (x = 0; x < MAZEWIDTH+2; ++x)
	fprintf(output, "X");
    fprintf(output, "\n");
    for (y = 0; y < MAZEHEIGHT; ++y)
    {
	fprintf(output, "X");
	for (x = 0; x < MAZEWIDTH; ++x)
	    fprintf(output, "%c", display[y][x]);
	fprintf(output, "X\n");
    }
    for (x = 0; x < MAZEWIDTH+2; ++x)
	fprintf(output, "X");
    fprintf(output, "\n\n\n");
}

void preprocessmaze()
{
    int x, y, d;
    int newx, newy;

    if (maze == NULL)
    {
	maze = (cell **)calloc(MAZEHEIGHT, sizeof(cell *));
	for (y = 0; y < MAZEHEIGHT; ++y)
	    maze[y] = (cell *)calloc(MAZEWIDTH, sizeof(cell));
	for (y = 0; y < MAZEHEIGHT; ++y)
	    for (x = 0; x < MAZEWIDTH; ++x)
	    {
		maze[y][x].x = x;
		maze[y][x].y = y;
		for (d = 0; d < DIRECTIONS; ++d)
		{
		    newy = y + dy[d];
		    newx = x + dx[d];
		    maze[y][x].succ[d] = (newy >= 0 && newy < MAZEHEIGHT && newx >= 0 && newx < MAZEWIDTH) ? &maze[newy][newx] : NULL;
		}
	    }
    }
#ifdef RANDOMSTARTGOAL
    goaly = (random() % ((MAZEHEIGHT + 1) / 2)) * 2;
    goalx = (random() % ((MAZEWIDTH + 1) / 2)) * 2;
    while (1)
    {
	starty = (random() % ((MAZEHEIGHT + 1) / 2)) * 2;
	startx = (random() % ((MAZEWIDTH + 1) / 2)) * 2;
        if (startx != goalx || starty != goaly)
            break;
    }
    mazestart = &maze[starty][startx];
    mazegoal = &maze[goaly][goalx];
#else
#ifdef DEBUG
    assert(STARTY % 2 == 0);
    assert(STARTX % 2 == 0);
    assert(GOALY % 2 == 0);
    assert(GOALX % 2 == 0);
#endif
    mazestart = &maze[STARTY][STARTX];
    mazegoal = &maze[GOALY][GOALX];
#endif
    mazeiteration = 0;
}

void postprocessmaze()
{
    int x, y;
    int d1, d2;
    cell *tmpcell;

    for (y = 0; y < MAZEHEIGHT; ++y)
	for (x = 0; x < MAZEWIDTH; ++x)
	{
	    maze[y][x].generated = 0;
	    maze[y][x].heapindex = 0; 
	    for (d1 = 0; d1 < DIRECTIONS; ++d1)
		maze[y][x].move[d1] = maze[y][x].succ[d1];
	}
    for (d1 = 0; d1 < DIRECTIONS; ++d1)
	if (mazegoal->move[d1] && mazegoal->move[d1]->obstacle)
	{
	    tmpcell = mazegoal->move[d1];
	    for (d2 = 0; d2 < DIRECTIONS; ++d2)
		if (tmpcell->move[d2])
		{
		    tmpcell->move[d2] = NULL;
		    tmpcell->succ[d2]->move[revers[d2]] = NULL;
		}
	}
}

void newrandommaze()
{
    int d1, d2;
    int x, y;
    int newx, newy;
    cell *tmpcell;

    preprocessmaze();
    for (y = 0; y < MAZEHEIGHT; ++y)
	for (x = 0; x < MAZEWIDTH; ++x)
	    maze[y][x].obstacle = (random() % 10000 < 10000 * MAZEDENSITY);
    mazegoal->obstacle = 0;
#ifndef STARTCANBEBLOCKED
    mazestart->obstacle = 0;
#endif
    postprocessmaze();
}

void newdfsmaze(int wallstoremove)
{
    int d, dtmp;
    int x, y;
    int newx, newy;
    int randomnumber;
    cell *tmpcell;
    int permute[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    int permutetmp;

    preprocessmaze();
  #ifdef DEBUG
    assert(MAZEWIDTH % 2 == 1);
    assert(MAZEHEIGHT % 2 == 1);
  #endif
    for (y = 0; y < MAZEHEIGHT; ++y)
	for (x = 0; x < MAZEWIDTH; ++x)
	{
	    maze[y][x].obstacle = 1;
	    maze[y][x].dfsx = 0;
	    maze[y][x].dfsy = 0;
	}
    x = 0;
    y = 0;
    maze[y][x].dfsx = -1;
    maze[y][x].dfsy = -1;
    while (1)
    {
	if (maze[y][x].obstacle)
	    maze[y][x].obstacle = 0;
	for (d = 0; d < DIRECTIONS-1; ++d)
	{
	    randomnumber = random() % (DIRECTIONS-d);
	    permutetmp = permute[randomnumber];
	    permute[randomnumber] = permute[DIRECTIONS-1-d];
	    permute[DIRECTIONS-1-d] = permutetmp;
	}
	for (dtmp = 0; dtmp < DIRECTIONS; ++dtmp)
	{
	    d = permute[dtmp];
	    newx = x + 2*dx[d];
	    newy = y + 2*dy[d];
	    if (maze[y][x].succ[d] != NULL && maze[newy][newx].obstacle)
	    {
		if (maze[y + dy[d]][x + dx[d]].obstacle)
		    maze[y + dy[d]][x + dx[d]].obstacle = 0;
		maze[newy][newx].dfsx = x;
		maze[newy][newx].dfsy = y;
		x = newx;
		y = newy;
		break;
	    }
	}
	if (dtmp == DIRECTIONS)
	{
	    if (maze[y][x].dfsx == -1)
		break;
	    newx = maze[y][x].dfsx;
	    newy = maze[y][x].dfsy;
	    x = newx;
	    y = newy;
	}
    }
    while (wallstoremove)
    {
	newx = random() % MAZEWIDTH;
	newy = random() % MAZEHEIGHT;
	if (maze[newy][newx].obstacle)
	{
	    maze[newy][newx].obstacle = 0;
	    --wallstoremove;
	}
    }
    mazegoal->obstacle = 0;
#ifndef STARTCANBEBLOCKED
    mazestart->obstacle = 0;
#endif
    postprocessmaze();
}

void initialize() //TODO: move to controller
{
    ++mazeiteration;
    keymodifier = 0;
    mazestart->g = LARGE;
    mazestart->rhs = 0;
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
    mazestart->searchtree = NULL;
    mazestart->generated = mazeiteration;
    insertheap(mazestart);
    mazegoal->g = LARGE;
    mazegoal->rhs = LARGE;
    mazegoal->searchtree = NULL;
    mazegoal->generated = mazeiteration;
}

#ifdef RANDOMIZESUCCS
int permute[DIRECTIONS];
int* permutation[DIRECTIONS];
int permutations;

void swappermutations(int n)
{
    int i;
    int swap;

    if (n)
	for (i = 0; i <= n; ++i)
	{
	    swappermutations(n-1);
	    if (n % 2)
	    {
		swap = permute[n];
		permute[n] = permute[i];
		permute[i] = swap;
	    }
	    else
	    {
		swap = permute[n];
		permute[n] = permute[0];
		permute[0] = swap;
	    }
	}
    else
    {
	for (i = 0; i < DIRECTIONS; ++i)
	    permutation[i][permutations] = permute[i];
	permutations++;
    }
}

void createpermutations()
{
    int i, j;

    permutations = 2;
    for (i = 3; i <= DIRECTIONS; ++i)
	permutations *= i;
    for (i = 0; i < DIRECTIONS; ++i)
    {
	permute[i] = i;
	permutation[i] = (int*)calloc(permutations, sizeof(int));
    }
    permutations = 0;
    swappermutations(DIRECTIONS-1);
}
#endif

// ForSyDe model:

void maze_func(tuple<vector<odom_type>,vector<laser_type>,vector<maze_state>>& outs,
                    const tuple<vector<cmd_type>,vector<maze_state>>& inps)
{cout<<"[Maze] inps"<<inps<<endl;
    int cur_x, cur_y;
    bool initialized;
    tie(initialized, cur_x, cur_y) = get<1>(inps)[0];
    if (!initialized)   // if not initialized
    {
    #ifdef RANDOMIZESUCCS
    createpermutations();
    #endif
    srandom(13);
    #ifdef RANDOMMAZE	
	    newrandommaze();
    #else
	    newdfsmaze(WALLSTOREMOVE);
    #endif
    #ifdef DISPLAY
	    printactualmaze(stdout);
    #endif
	    initialize();
        cur_x = mazegoal->x; cur_y = mazegoal->y;
	    fflush(stdout);
    }
    else
    {
        auto cmd = get<0>(inps)[0];
        switch (cmd)
        {
            case UP:
                if (cur_y < MAZEHEIGHT-1 && !maze[cur_y+1][cur_x].obstacle)
                    ++cur_y;
                break;
            case DOWN:
                if (cur_y > 0 && !maze[cur_y-1][cur_x].obstacle)
                    --cur_y;
                break;
            case LEFT:
                if (cur_x > 0 && !maze[cur_y][cur_x-1].obstacle)
                    --cur_x;
                break;
            case RIGHT:
                if (cur_x < MAZEWIDTH-1 && !maze[cur_y][cur_x+1].obstacle)
                    ++cur_x;
                break;
            default:
                break;
        }
    }
    // send the odometry
    get<0>(outs)[0] = odom_type(cur_x, cur_y);
    // send the laser scan data
    get<1>(outs)[0] = {maze[cur_y][cur_x].succ[RIGHT] ? maze[cur_y][cur_x].succ[RIGHT]->obstacle!=0 : false,
                        maze[cur_y][cur_x].succ[UP]   ? maze[cur_y][cur_x].succ[UP]->obstacle!=0    : false,
                        maze[cur_y][cur_x].succ[LEFT] ? maze[cur_y][cur_x].succ[LEFT]->obstacle!=0  : false,
                        maze[cur_y][cur_x].succ[DOWN] ? maze[cur_y][cur_x].succ[DOWN]->obstacle!=0  : false};
    // the maze (self) state
    get<2>(outs)[0] = maze_state(true, cur_x, cur_y);
    cout<<"[Maze] outs"<<outs<<endl;
    // getchar();
}

#endif
