
#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <forsyde.hpp>
#include <tuple>
#include <vector>
#include "globals.hpp"
// #include "heap.hpp"
#include "maze.hpp"

using namespace std;

// Copied from D* lite original code
///TODO: remove
cell goaltmpcell, oldtmpcell;
cell *tmpcell;
cell *lastcell;

void initializecell(cell *thiscell)
{
    if (thiscell->generated != mazeiteration)
    {
	thiscell->g = LARGE;
	thiscell->rhs = LARGE;
	thiscell->searchtree = NULL;
	thiscell->generated = mazeiteration;
    }
}

void updatecell(cell *thiscell)
{
    if (thiscell->g < thiscell->rhs)
    {
#ifdef TIEBREAKING
	thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
	thiscell->key[1] = thiscell->g + H(thiscell) + keymodifier;
	thiscell->key[2] = thiscell->g;
#else
	thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
	thiscell->key[1] = thiscell->g; 
#endif
	insertheap(thiscell);
    }
    else if (thiscell->g > thiscell->rhs)
    {
#ifdef TIEBREAKING
	thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
	thiscell->key[1] = thiscell->rhs + H(thiscell) + keymodifier + 1;
	thiscell->key[2] = H(thiscell) + keymodifier;
#else
	thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
	thiscell->key[1] = thiscell->rhs;
#endif
	insertheap(thiscell);
    }
    else 
	deleteheap(thiscell);
}

void updatekey(cell *thiscell)
{
    if (thiscell->g < thiscell->rhs)
    {
#ifdef TIEBREAKING
	thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
	thiscell->key[1] = thiscell->g + H(thiscell) + keymodifier;
	thiscell->key[2] = thiscell->g;
#else
        thiscell->key[0] = thiscell->g + H(thiscell) + keymodifier;
	thiscell->key[1] = thiscell->g; 
#endif
    }
    else 
    {
#ifdef TIEBREAKING
	thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
	thiscell->key[1] = thiscell->rhs + H(thiscell) + keymodifier + 1;
	thiscell->key[2] = H(thiscell) + keymodifier;
#else
        thiscell->key[0] = thiscell->rhs + H(thiscell) + keymodifier;
        thiscell->key[1] = thiscell->rhs;
#endif
    }
}

void updaterhs(cell *thiscell)
{
    int d;
#ifdef RANDOMIZESUCCS
    int dcase, dtemp;
#endif

    thiscell->rhs = LARGE;
    thiscell->searchtree = NULL;
#ifdef RANDOMIZESUCCS
      dcase = random() % permutations;
      for (dtemp = 0; dtemp < DIRECTIONS; ++dtemp)
      {
	  d = permutation[dtemp][dcase];
#else
    for (d = 0; d < DIRECTIONS; ++d)
    {
#endif
	if (thiscell->move[d] && thiscell->move[d]->generated == mazeiteration && thiscell->rhs > thiscell->move[d]->g + 1)
	{
	    thiscell->rhs = thiscell->move[d]->g + 1;
	    thiscell->searchtree = thiscell->move[d];
	}
    }
    updatecell(thiscell);
}

int computeshortestpath()
{
    cell *tmpcell1, *tmpcell2;
    int x, d;
#ifdef RANDOMIZESUCCS
    int dcase, dtemp;
#endif

#ifdef TIEBREAKING
    if (mazegoal->g < mazegoal->rhs)
    {
	goaltmpcell.key[0] = mazegoal->g + keymodifier;
	goaltmpcell.key[1] = mazegoal->g + keymodifier;
	goaltmpcell.key[2] = mazegoal->g;
    }
    else
    {
	goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
	goaltmpcell.key[1] = mazegoal->rhs + keymodifier + 1;
	goaltmpcell.key[2] = keymodifier;
    }
#else
    if (mazegoal->g < mazegoal->rhs)
    {
	goaltmpcell.key[0] = mazegoal->g + keymodifier;
	goaltmpcell.key[1] = mazegoal->g;
    }
    else
    {
	goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
	goaltmpcell.key[1] = mazegoal->rhs;
    }
#endif
    while (topheap() && (mazegoal->rhs > mazegoal->g || keyless(topheap(), &goaltmpcell)))
    {
	tmpcell1 = topheap();
	oldtmpcell.key[0] = tmpcell1->key[0];
	oldtmpcell.key[1] = tmpcell1->key[1];
#ifdef TIEBREAKING
	oldtmpcell.key[2] = tmpcell1->key[2];
#endif
	updatekey(tmpcell1); 
	if (keyless(&oldtmpcell, tmpcell1))
	    updatecell(tmpcell1);
	else if (tmpcell1->g > tmpcell1->rhs)
	{
	    tmpcell1->g = tmpcell1->rhs;
	    deleteheap(tmpcell1);
#ifdef RANDOMIZESUCCS
	    dcase = random() % permutations;
	    for (dtemp = 0; dtemp < DIRECTIONS; ++dtemp)
	    {
		d = permutation[dtemp][dcase];
#else
	    for (d = 0; d < DIRECTIONS; ++d)
	    {
#endif
		if (tmpcell1->move[d])
		{
		    tmpcell2 = tmpcell1->move[d];
		    initializecell(tmpcell2);
		    if (tmpcell2 != mazestart && tmpcell2->rhs > tmpcell1->g + 1)
		    {
			tmpcell2->rhs = tmpcell1->g + 1;
			tmpcell2->searchtree = tmpcell1;
			updatecell(tmpcell2);
		    }
		}
	    }
	}
      else
      {
	  tmpcell1->g = LARGE;
	  updatecell(tmpcell1);
#ifdef RANDOMIZESUCCS
	  dcase = random() % permutations;
	  for (dtemp = 0; dtemp < DIRECTIONS; ++dtemp)
	  {
	      d = permutation[dtemp][dcase];
#else
	  for (d = 0; d < DIRECTIONS; ++d) 
	  {
#endif
	      if (tmpcell1->move[d])
	      {
		  tmpcell2 = tmpcell1->move[d];
		  initializecell(tmpcell2);
		  if (tmpcell2 != mazestart && tmpcell2->searchtree == tmpcell1)
		      updaterhs(tmpcell2);
	      }
	  }
      }
#ifdef TIEBREAKING
	if (mazegoal->g < mazegoal->rhs)
	{
	    goaltmpcell.key[0] = mazegoal->g + keymodifier;
	    goaltmpcell.key[1] = mazegoal->g + keymodifier;
	    goaltmpcell.key[2] = mazegoal->g;
	}
	else
	{
	    goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
	    goaltmpcell.key[1] = mazegoal->rhs + keymodifier + 1;
	    goaltmpcell.key[2] = keymodifier;
	}    
#else
	if (mazegoal->g < mazegoal->rhs)
	{
	    goaltmpcell.key[0] = mazegoal->g + keymodifier;
	    goaltmpcell.key[1] = mazegoal->g;
	}
	else
	{
	    goaltmpcell.key[0] = mazegoal->rhs + keymodifier;
	    goaltmpcell.key[1] = mazegoal->rhs;
	}
#endif
    }
  return (mazegoal->rhs == LARGE);
}

void updatemaze(cell *robot)
{
    int d1, d2;
    cell *tmpcell;
#ifdef RANDOMIZESUCCS
    int dcase, dtemp;
#endif

#ifdef RANDOMIZESUCCS
      dcase = random() % permutations;
      for (dtemp = 0; dtemp < DIRECTIONS; ++dtemp)
      {
	  d1 = permutation[dtemp][dcase];
#else
    for (d1 = 0; d1 < DIRECTIONS; ++d1)
    {
#endif
	if (robot->move[d1] && robot->move[d1]->obstacle)
	{
	    tmpcell = robot->move[d1];
	    initializecell(tmpcell);
	    for (d2 = 0; d2 < DIRECTIONS; ++d2)
		if (tmpcell->move[d2])
		{
		    tmpcell->move[d2] = NULL;
		    tmpcell->succ[d2]->move[revers[d2]] = NULL;
		    initializecell(tmpcell->succ[d2]);
		    if (tmpcell->succ[d2] != mazestart && tmpcell->succ[d2]->searchtree == tmpcell)
			updaterhs(tmpcell->succ[d2]);
		}
	    if (tmpcell != mazestart)
	    {
		tmpcell->rhs = LARGE;
		updatecell(tmpcell);
	    }
	}
    }
}

// ForSyDe code

void controller_func(tuple<vector<cmd_type>,vector<controller_state>>& outs,
                    const tuple<vector<odom_type>,vector<laser_type>,vector<controller_state>>& inps)
{
    auto odom_data = get<0>(inps)[0];
    auto laser_data = get<1>(inps)[0];
    auto controller_st = get<2>(inps)[0];
    switch (controller_st)
    {
        case INIT:
        {
            //initialize();
            mazegoal = &maze[get<1>(odom_data)][get<0>(odom_data)];
            lastcell = mazegoal;
            get<0>(outs)[0] = NOP;
            get<1>(outs)[0] = PLAN;
            break;
        }
        case PLAN:
        {
            if (computeshortestpath())
		        SC_REPORT_ERROR("Controller","Cannot compute the shortest path!");
            mazegoal->trace = NULL;
            get<0>(outs)[0] = NOP;
            get<1>(outs)[0] = MOVE;
            break;
        }
        case MOVE:
        {
            if (mazestart==mazegoal)
            {
                get<0>(outs)[0] = NOP;
                get<1>(outs)[0] = IDLE;
            }
            else{
                // Check if a new obstacle has been detected
                if (mazegoal->searchtree->obstacle)
                {
                    // keymodifier += H(lastcell);
                    // lastcell = mazegoal;
                    // for (tmpcell=mazegoal; tmpcell; tmpcell=tmpcell->trace)
                    //     updatemaze(tmpcell);
                    // get<0>(outs)[0] = NOP;
                    // get<1>(outs)[0] = PLAN;
                    ;
                }
                else{
                    int next;
                    for (next=0;next<DIRECTIONS;next++)
                        if (&maze[mazegoal->y+dy[next]][mazegoal->x+dx[next]] == mazegoal->searchtree)
                            break;
                    // move command to simulator
                    get<0>(outs)[0] = (cmd_type)next;
                    get<1>(outs)[0] = MOVECHECK;
                }
            }
            break;
        }
        case MOVECHECK:
        {
            // if the robot has made its next move
            if (get<0>(odom_data) == mazegoal->searchtree->x && get<1>(odom_data) == mazegoal->searchtree->y)
            {
                if (mazestart==mazegoal)
                {
                    get<0>(outs)[0] = NOP;
                    get<1>(outs)[0] = IDLE;
                }
                else
                {
                    mazegoal->searchtree->trace = mazegoal;
                    mazegoal = mazegoal->searchtree;
                    get<0>(outs)[0] = NOP;
                    get<1>(outs)[0] = MOVE;
                    // print the current status of the maze
                    printknownmaze(stdout);
                }
                // update the maze map based on laser data
                for (int dir=0;dir<DIRECTIONS;dir++)
                    if (laser_data[dir])
                        maze[get<1>(odom_data)+dy[dir]][get<0>(odom_data)+dx[dir]].obstacle = true;
            }
            else
            {
                int next;
                for (next=0;next<DIRECTIONS;next++)
                    if (&maze[mazegoal->y+dy[next]][mazegoal->x+dx[next]] == mazegoal->searchtree)
                        break;
                // move command to simulator
                get<0>(outs)[0] = (cmd_type)next;
                get<1>(outs)[0] = MOVECHECK;
            }
            break;
        }
        default:
        {
            get<0>(outs)[0] = NOP;
            get<1>(outs)[0] = IDLE;
            break;
        }
    }
    cout<<"[Controller] command: "<<get<0>(outs)[0]<<endl;
    // getchar();
    sleep(1);
}

#endif
