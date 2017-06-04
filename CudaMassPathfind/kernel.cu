
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
int flatten2dCoordinate(int x, int y) 
{
	return x * NAV_GRID_WIDTH + y;
}

__global__
void BatchPathFindKernel(int* fromXs, int* fromYs, int* toXs, int* toYs, int numPaths, int* flatNavGrid, IntPair** returnedPahts) 
{
	int thid = getGlobalIdx_1D_1D();

	bool closedSet[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	int closedSetSize;

	bool openSet[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	int openSetSize;
	
	int score[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];

	IntPair cameFrom[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];

	//TODO: INIT ALL THESE VALUES

	openSet[fromXs[thid]][fromYs[thid]] = true;
	openSetSize = 1;

	while (openSetSize) 
	{
		//check grid square
		//TODO: ADD STUb
		IntPair current; // = chooseNextGridSquare(pathRequest, openSet);
		if (current.x = toXs[thid] && current.y == toYs[thid]) 
		{
			//TODO: RECONSTRUCT PATH
		}
		//grid square was not destination
		openSet[current.x][current.y] = false;
		--openSetSize;
		closedSet[current.x][current.y] = true;
		++closedSetSize;
	
		//find other neighbors
		//TOOD: FIND NEIGHBORS
		IntPair neighbors[MAX_NEIGHBORS];
		int neighborCount;

		for (int i = 0; i < neighborCount; i++) 
		{
			if (closedSet[neighbors[i].x][neighbors[i].y]) 
			{
				continue;// no need to evaluate already evaluated nodes
			}
			//cost of reaching neighbor using current path
			IntPair neighbor = neighbors[i];
			int transitionCost = ( flatNavGrid[flatten2dCoordinate(current.x, current.y)] + flatNavGrid[flatten2dCoordinate(neighbor.x, neighbor.y)] ) / 2;
			int tentativeScore = score[current.x][current.y] + transitionCost;
				
			//discover new node
			if (!openSet[neighbor.x][neighbor.y]) 
			{

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

