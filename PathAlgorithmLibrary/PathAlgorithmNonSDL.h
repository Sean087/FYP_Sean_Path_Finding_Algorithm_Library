//
//	PathAlgorithmNonSDL.h - Contains declaration of function class for path finding functions
//							Serve as a library for including A-Star path finding into non SDL projects
//
//	Sean Horgan:			K00196030
//  Date Complete:			08/04/2018
//

#pragma once

#ifdef PATHALGORITHM_EXPORTS
#define PATHALGORITHM_API __declspec (dllexport)
#else
#define PATHALGORITHM_API __declspec (dllimport)
#endif

#include <vector>
#include <list>

/* Node implementation without SDL */
struct Node_NoSDL
{
	bool bObstacle = false;																// Is the node an obstruction?
	bool bVisited = false;																// Have we searched this node before?
	float fEstimatedTotalCost;															// Distance to goal so far
	float fCostSoFar;																	// Distance to goal if we took the alternative route
	int x;																				// Nodes X-position in 2D space
	int y;																				// Nodes Y-position in 2D space
	std::vector<Node_NoSDL*> vecNeighbours;												// Connections to neighbours
	Node_NoSDL* parent;																	// Node connecting to this node that offers shortest parent
};// end struct

namespace PathAlgorithm
{
	class Functions
	{
	public:

		static PATHALGORITHM_API float distance(Node_NoSDL* a, Node_NoSDL* b);			// Distance between two points (nodes)

		static PATHALGORITHM_API float heuristic(Node_NoSDL* a, Node_NoSDL* b);			// Heuristic is the distance between points so use distance function

		static PATHALGORITHM_API void AStarSolver(Node_NoSDL* start, Node_NoSDL* end);	// A star algorithm implementation to find shortest path
	};
}
#pragma once
