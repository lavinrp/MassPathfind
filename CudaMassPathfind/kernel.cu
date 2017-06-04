
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <iostream>

#define NAV_GRID_WIDTH 15
#define NAV_GRID_HEIGHT 15

#define MAX_NEIGHBORS 4


#define BLOCKED_GRID_WEIGHT 1000

struct IntPair {
	int x;
	int y;
};

__device__
int getGlobalIdx_1D_1D() 
{
	return blockIdx.x *blockDim.x + threadIdx.x;
}

__device__
void getNeighbors(const IntPair & gridSquareCoordinate, IntPair* neighbors) {
	for (unsigned int ii = 0; ii < MAX_NEIGHBORS; ++ii)
	{
		neighbors[ii].x = neighbors[ii].y = -1;
	}

	bool xUp = (gridSquareCoordinate.x + 1 < NAV_GRID_WIDTH);
	bool xDown = (gridSquareCoordinate.x - 1 >= 0);
	bool yUp = (gridSquareCoordinate.y + 1 < NAV_GRID_HEIGHT);
	bool yDown = (gridSquareCoordinate.y - 1 >= 0);

	if (xUp) {
		neighbors[0].x = gridSquareCoordinate.x + 1;
		neighbors[0].y = gridSquareCoordinate.y;
	}
	if (xDown) {
		neighbors[0].x = gridSquareCoordinate.x - 1;
		neighbors[0].y = gridSquareCoordinate.y;
	}
	if (yUp) {
		neighbors[0].x = gridSquareCoordinate.x;
		neighbors[0].y = gridSquareCoordinate.y + 1;
	}
	if (yDown) {
		neighbors[0].x = gridSquareCoordinate.x;
		neighbors[0].y = gridSquareCoordinate.y - 1;
	}
}

__device__
void reconstructPath(const IntPair & beginPoint, const IntPair & endPoint, const IntPair cameFrom[NAV_GRID_WIDTH][NAV_GRID_HEIGHT], IntPair* backwardsOrderPath, int* length) {
	IntPair foundSquare = endPoint;

	int pathIndex = 0;
	while (foundSquare.x != beginPoint.x && foundSquare.y != beginPoint.y)
	{
		backwardsOrderPath[pathIndex].x = foundSquare.x;
		backwardsOrderPath[pathIndex].y = foundSquare.y;

		foundSquare = cameFrom[foundSquare.x][foundSquare.y];
		++pathIndex;
	}

	*length = pathIndex;
}

__global__
void BatchPathFindKernel(int* fromXs, int* fromYs, int* toXs, int* toYs, int numPaths, int* flatNavGrid, IntPair** returnedPaths, int* length)
{
	int thid = getGlobalIdx_1D_1D();

	bool closedSet[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	int closedSetSize;

	bool openSet[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	int openSetSize;
	
	int coordinateToScoreMap[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];

	IntPair cameFrom[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];

	//TODO: INIT ALL THESE VALUES

	openSet[fromXs[thid]][fromYs[thid]] = true;
	openSetSize = 1;

	while (openSetSize) 
	{
		//check grid square
		//TODO: ADD STUb
		IntPair current; // = chooseNextGridSquare(pathRequest, openSet);
		if (current.x == toXs[thid] && current.y == toYs[thid]) 
		{
			IntPair beginPoint;
			beginPoint.x = fromXs[thid];
			beginPoint.y = fromYs[thid];
			reconstructPath(beginPoint, current, cameFrom, returnedPaths[thid], length);
			break;
		}
		//grid square was not destination
		openSet[current.x][current.y] = false;
		--openSetSize;
		closedSet[current.x][current.y] = true;
		++closedSetSize;
	
		//find other neighbors
		//TOOD: FIND NEIGHBORS
		IntPair neighbors[MAX_NEIGHBORS];
		getNeighbors(current, neighbors);
		int neighborCount;

		for (int i = 0; i < neighborCount; i++) 
		{
			if (closedSet[neighbors[i].x][neighbors[i].y]) 
			{
				continue;
			}
		}
		
	}



}

void BatchPathfind(int* fromXs, int* fromYs, int* toXs, int* toYs, int numPaths, int (&navGrid)[NAV_GRID_WIDTH][NAV_GRID_HEIGHT]) 
{

}

int main() 
{
	std::cout << "path from: " << std::endl;
	int fromX = 0;
	int fromY = 0;
	//std::cin >> fromX;
	//std::cin >> fromY;

	std::cout << "path to: " << std::endl;
	int toX = 5;
	int toY = 2;
	/*std::cin >> toX;
	std::cin >> toY;*/

	//S*x************
	//**x************
	//**x**O**x******
	//**x*****x******
	//**x*****x******
	//**xxxxxxx******
	//***************
	//***************
	//***************
	//***************
	//***************
	//***************
	//***************
	//***************
	//***************

	//create clear nav grid
	int navGrid[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	for (unsigned int i = 0; i < NAV_GRID_WIDTH; i++) 
	{
		for (unsigned int j = 0; j <NAV_GRID_HEIGHT; j++) 
		{
			navGrid[i][j] = 0;
		}
	}

	//block grid
	navGrid[2][0] = BLOCKED_GRID_WEIGHT;
	navGrid[2][1] = BLOCKED_GRID_WEIGHT;
	navGrid[2][2] = BLOCKED_GRID_WEIGHT;
	navGrid[2][3] = BLOCKED_GRID_WEIGHT;
	navGrid[2][4] = BLOCKED_GRID_WEIGHT;
	navGrid[2][5] = BLOCKED_GRID_WEIGHT;
	navGrid[3][5] = BLOCKED_GRID_WEIGHT;
	navGrid[4][5] = BLOCKED_GRID_WEIGHT;
	navGrid[5][5] = BLOCKED_GRID_WEIGHT;
	navGrid[6][5] = BLOCKED_GRID_WEIGHT;
	navGrid[7][5] = BLOCKED_GRID_WEIGHT;
	navGrid[8][5] = BLOCKED_GRID_WEIGHT;
	navGrid[8][4] = BLOCKED_GRID_WEIGHT;
	navGrid[8][3] = BLOCKED_GRID_WEIGHT;
	navGrid[8][2] = BLOCKED_GRID_WEIGHT;

    return 0;
}

