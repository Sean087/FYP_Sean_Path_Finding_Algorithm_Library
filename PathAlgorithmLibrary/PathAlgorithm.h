//
//	PathAlgorithm.h - Contains declaration of function class for path finding functions
//					  Serve as a library for including A-Star path finding into project
//
//	Sean Horgan:		K00196030
//  Date Complete:		14/03/2018
//

#pragma once

#ifdef PATHALGORITHM_EXPORTS
#define PATHALGORITHM_API __declspec (dllexport)
#else
#define PATHALGORITHM_API __declspec (dllimport)
#endif

#include "SDL.h"
#include <vector>
#include <list>

struct sNode
{
	bool bObstacle = false;															// Is the node an obstruction?
	bool bVisited = false;															// Have we searched this node before?
	float fGlobalGoal;																// Distance to goal so far
	float fLocalGoal;																// Distance to goal if we took the alternative route
	int x;																			// Nodes X-position in 2D space
	int y;																			// Nodes Y-position in 2D space
	std::vector<sNode*> vecNeighbours;												// Connections to neighbours
	sNode* parent;																	// Node connecting to this node that offers shortest parent
	SDL_Rect box;
};// end struct

namespace PathAlgorithm
{
	class Functions
	{
	public:

		static PATHALGORITHM_API float distance(sNode* a, sNode* b);				// Distance between two points (nodes)

		static PATHALGORITHM_API float heuristic(sNode* a, sNode* b);				// Heuristic is the distance between points so use distance function

		static PATHALGORITHM_API void AStarSolver(sNode* start, sNode* end);		// A star algorithm implementation to find shortest path
	};
}
