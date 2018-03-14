//
//	PathAlgorithm.h - Defines the exported function for the DLL Application.
//					  Server as a library for including A-Star path finding into project
//
//	Sean Horgan:		K00196030
//  Date Complete:		14/03/2018
//

#include "stdafx.h"
#include "PathAlgorithm.h"

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
	float Functions::distance(sNode* a, sNode* b)
	{
		return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
	}

	float Functions::heuristic(sNode* a, sNode* b)
	{
		return distance(a, b);															// Heuristic is the distance between points so use distance function
	}

	void Functions::AStarSolver(sNode* start, sNode* end)
	{
		sNode *nodeStart = start;
		sNode *nodeEnd = end;
		sNode *nodeCurrent = nodeStart;													// Set the current node to the beginning
		nodeStart->fLocalGoal = 0.0f;													// The local goal is 0 since we didn't start searching yet
		nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);							// Global goal = distance between start node and end node

		/* ------------------------------------------------------------------------------------------
		*	While the algorithm is searching, newly discovered nodes are added to the not-tested list,
		*	which queues them for testing for a potential path.
		*
		*  The first node we want to test if the start node so we queue that node first.
		*
		*	NOTE TO SELF: Attempt to change list to priority queue to avoid manual sorting.
		* ------------------------------------------------------------------------------------------
		*/
		std::list<sNode*> listNotTestedNodes;											// Not tested list where new nodes are queded for testing
		listNotTestedNodes.push_back(nodeStart);										// Add the start node for testing

		/* ------------------------------------------------------------------------------------------
		*	CONDITION: (!listNotTestedNodes.empty() && !nodeCurrent->bVisited)
		*		- While there are nodes to test && the end node has not been visited, run A*
		*		- Just because it has found the end node doesn't mean it found the shortest path.
		*		- This will find THE SHORTEST PATH by continuuing the search while potentially
		*		  shorter paths exist.
		* ------------------------------------------------------------------------------------------
		*/
		while (!listNotTestedNodes.empty() && !nodeEnd->bVisited)						// Finds the shortest path - while there are nodes to test and the end node has not been visited
		{
			listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs)
			{ return lhs->fGlobalGoal < rhs->fGlobalGoal; });							// Sort all untested nodes by shortest global goal (possible shortest path)

			while (!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)	// Potential to have already visited nodes at front of list therefore:
				listNotTestedNodes.pop_front();											// Remove them from the list as they have already been visited

			if (listNotTestedNodes.empty())												// If there are no valid nodes to test, abort.
				break;

			nodeCurrent = listNotTestedNodes.front();									// Set node at front of the list to current node for testing
			nodeCurrent->bVisited = true;												// Only need to test a node once so set it to visited

			for (auto nodeNeighbour : nodeCurrent->vecNeighbours)						// Check every neighbour of the current node
			{
				if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)			// If the neighbour has not been visited and is not an obstacle:
					listNotTestedNodes.push_back(nodeNeighbour);						// add it to the not-tested list for future testing

				float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);		// Calculate the neighbours possibly lowest parent distance

				/* ------------------------------------------------------------------------------------------
				*	If we decide to continue the path through the current neighbour due to it having the
				*	lowest distance, set the neighbouring nodes parent to the current node so we can
				*	use it the current node as the source of the path from the neighbour.
				*
				*	Then update the neighbours local goal to its distance from its parent plus its parents
				*	local goal.
				*
				*	Basically update our current path.
				* ------------------------------------------------------------------------------------------
				*/
				if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
				{
					nodeNeighbour->parent = nodeCurrent;
					nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;

					/* ------------------------------------------------------------------------------------------
					*	Update the global goal of the neighbour using the heuristic to get the distance from
					*	the neighbour to the end goal.
					*
					*	When sort the nodes-to-test list, it may see that the current path has become too long
					*	and ditch it in favour of a new shorter path. This bias is what I found to be the
					*	most interesting part of A*.
					* ------------------------------------------------------------------------------------------
					*/
					nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
				}// end if
			}// end for
		}// end while
	}
}