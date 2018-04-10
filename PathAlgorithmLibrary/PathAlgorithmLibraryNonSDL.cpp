//
//	PathAlgorithm.cpp - Defines the exported function for the DLL Application.
//					  Server as a library for including A-Star path finding into SDL projects
//
//	Sean Horgan:		K00196030
//  Date Complete:		14/03/2018
//

#include "stdafx.h"
#include "PathAlgorithmNonSDL.h"

namespace PathAlgorithm
{
	/* ------------------------------------------------------------------------------------------
	*	Since we do not know the distances between nodes for heuristic, use Pythagoras Theorom.
	*
	*  Used since it's effective for finding distance between center-points of nodes:
	*		- (a^a) + (b^b) = (c^c)
	*		- Example:
	*			Node a = (15, 8)
	*			Node b = (14, 8)
	*			(15 - 14)^2 + (8 - 8)^2 = 1
	*			sqrt(1) = 1
	*			c = 1
	* ------------------------------------------------------------------------------------------
	*/
	float Functions::distance(Node_NoSDL* a, Node_NoSDL* b)
	{
		return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
	}

	float Functions::heuristic(Node_NoSDL* a, Node_NoSDL* b)
	{
		return distance(a, b);															// Heuristic is the distance other heuristics can be placed here in the future
	}

	void Functions::AStarSolver(Node_NoSDL* start, Node_NoSDL* end)
	{
		Node_NoSDL *nodeStart = start;
		Node_NoSDL *nodeEnd = end;
		Node_NoSDL *nodeCurrent = nodeStart;											// Set the current node to the start node
		nodeStart->fCostSoFar = 0.0f;													// The g(n) is 0 since we didn't start searching yet
		nodeStart->fEstimatedTotalCost = heuristic(nodeStart, nodeEnd);					// f(n) = distance between start node and end node

		/* ------------------------------------------------------------------------------------------
		*	While the algorithm is searching, newly discovered open nodes are pushed to the
		*	onto not-tested list, which queues them for testing for a potential path.
		*
		*   First node to test is the start node, push it onto the list for consideration.
		*
		*	TO DO: Attempt to change list to priority queue to avoid manual sorting.
		* ------------------------------------------------------------------------------------------
		*/
		std::list<Node_NoSDL*> openNodes;												// List of open nodes where new nodes are queded for testing
		openNodes.push_back(nodeStart);													// Add the start node to be tested

		/* ------------------------------------------------------------------------------------------
		*	CONDITION: (!listNotTestedNodes.empty() && !nodeCurrent->bVisited)
		*		- While there are nodes to test && the end node has not been visited, run A*
		*		- Just because it has found the end node doesn't mean it found the shortest path.
		*		- This will find a potentially optimal path by continuing the search
		*		  while potentially shorter paths exist.
		* ------------------------------------------------------------------------------------------
		*/
		while (!openNodes.empty() && !nodeEnd->bVisited)								// Finds the shortest path - while there are nodes to test and the end node has not been visited
		{
			openNodes.sort([](const Node_NoSDL* leftMostNode, const Node_NoSDL* rightMostNode)
			{ return leftMostNode->fEstimatedTotalCost < rightMostNode->fEstimatedTotalCost; });	// Sort all untested nodes by lowest g(n) (possible shortest path)

			while (!openNodes.empty() && openNodes.front()->bVisited)					// Potential to have already-visited nodes at front of list therefore:
				openNodes.pop_front();													// Remove them from the list so they won't be considered

			if (openNodes.empty())														// If there are no valid nodes to test, stop A*.
				break;

			nodeCurrent = openNodes.front();											// Set node at front of the list to current node for testing
			nodeCurrent->bVisited = true;												// Only need to test a node once so set it to visited

			for (auto nodeNeighbour : nodeCurrent->vecNeighbours)						// Check every neighbour of the current node
			{
				if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)			// If the neighbour has not been visited and is not an obstacle:
					openNodes.push_back(nodeNeighbour);									// add it to the open node list for future testing

				float fPotentiallyLowestCSF = nodeCurrent->fCostSoFar + distance(nodeCurrent, nodeNeighbour);		// Calculate the neighbours possibly lowest g(n)

				/* ------------------------------------------------------------------------------------------
				*	If it decides to continue the path through the current neighbour due to it having the
				*	lowest g(n), set the neighbouring nodes parent to the current node so it can
				*	use the current node as the source of the path from the neighbour.
				*
				*	Then update the neighbours g(n) to its distance from its parent plus its parents
				*	g(n).
				*
				*	Basically update the path.
				* ------------------------------------------------------------------------------------------
				*/
				if (fPotentiallyLowestCSF < nodeNeighbour->fCostSoFar)
				{
					nodeNeighbour->parent = nodeCurrent;
					nodeNeighbour->fCostSoFar = fPotentiallyLowestCSF;

					/* ------------------------------------------------------------------------------------------
					*	Update the f(n) of the neighbour using the heuristic to get the distance from
					*	the neighbour to the end goal.
					*
					*	When sort the open node list, it may see that the current path has become too long
					*	and ditch it in favour of a new potentially shorter path. This is what sets A* apart
					*	from Dijskstra's
					* ------------------------------------------------------------------------------------------
					*/
					nodeNeighbour->fEstimatedTotalCost = nodeNeighbour->fCostSoFar + heuristic(nodeNeighbour, nodeEnd);
				}// end if
			}// end for
		}// end while
	}
}