
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <iostream>

#define NAV_GRID_WIDTH 2
#define NAV_GRID_HEIGHT 2

#define MAX_NEIGHBORS 4


#define BLOCKED_GRID_WEIGHT 1000

struct IntPair {
	int x;
	int y;
};

__device__
float CalcSquaredDistance(IntPair point1, IntPair point2) {
	return powf(point1.x - point2.x, 2) + powf(point1.y - point2.y, 2);
}

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

__device__
void chooseNextGridSquare(const IntPair & beginPoint, const IntPair & endPoint, const bool availableGridSquares[NAV_GRID_WIDTH][NAV_GRID_HEIGHT], IntPair* bestGridSquare) {

	float shortestDistance = FLT_MAX;

	for (unsigned int ii = 0; ii < NAV_GRID_WIDTH; ++ii){
		for (unsigned int jj = 0; jj < NAV_GRID_HEIGHT; ++jj) {
			if (availableGridSquares[ii][jj]){
				IntPair gridSquare;
				gridSquare.x = ii;
				gridSquare.y = jj;

				float gridSquareDistance = CalcSquaredDistance(gridSquare, endPoint);
				if (gridSquareDistance < shortestDistance) {
					shortestDistance = gridSquareDistance;
					*bestGridSquare = gridSquare;
				}
			}
		}
	}
}



__global__
void BatchPathFindKernel(int* fromXs, int* fromYs, int* toXs, int* toYs, int numPaths, int* flatNavGrid, IntPair** returnedPaths, int* length)
{
	int thid = getGlobalIdx_1D_1D();

	IntPair beginPoint;
	beginPoint.x = fromXs[thid];
	beginPoint.y = fromYs[thid];

	IntPair endPoint;
	endPoint.x = toXs[thid];
	endPoint.y = toYs[thid];

	bool closedSet[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	int closedSetSize = 0;
	for (unsigned int ii = 0; ii < NAV_GRID_WIDTH; ++ii) {
		for (unsigned int jj = 0; jj < NAV_GRID_HEIGHT; ++jj) {
			closedSet[ii][jj] = false;
		}
	}

	bool openSet[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	int openSetSize = 0;
	for (unsigned int ii = 0; ii < NAV_GRID_WIDTH; ++ii) {
		for (unsigned int jj = 0; jj < NAV_GRID_HEIGHT; ++jj) {
			openSet[ii][jj] = false;
		}
	}
	
	int score[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	for (unsigned int ii = 0; ii < NAV_GRID_WIDTH; ++ii) {
		for (unsigned int jj = 0; jj < NAV_GRID_HEIGHT; ++jj) {
			score[ii][jj] = INT_MAX;
		}
	}
	score[beginPoint.x][beginPoint.y] = 0;

	IntPair cameFrom[NAV_GRID_WIDTH][NAV_GRID_HEIGHT];
	// In case we end up needing to initialize. As of now, we do not.
	/*for (unsigned int ii = 0; ii < NAV_GRID_WIDTH; ++ii) {
		for (unsigned int jj = 0; jj < NAV_GRID_HEIGHT; ++jj) {
			cameFrom[ii][jj].x = 0;
			cameFrom[ii][jj].y = 0;
		}
	}*/

	openSet[fromXs[thid]][fromYs[thid]] = true;
	openSetSize = 1;

	while (openSetSize) 
	{
		//check grid square
		IntPair current;
		chooseNextGridSquare(beginPoint, endPoint, openSet, &current);


		if (current.x == toXs[thid] && current.y == toYs[thid]) 
		{
			reconstructPath(beginPoint, current, cameFrom, returnedPaths[thid], length);
			break;
		}
		//grid square was not destination
		openSet[current.x][current.y] = false;
		--openSetSize;
		closedSet[current.x][current.y] = true;
		++closedSetSize;
	
		//find other neighbors
		IntPair neighbors[MAX_NEIGHBORS];
		getNeighbors(current, neighbors);
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
				//add blocked to closed set and unblocked to open set
				if (flatNavGrid[flatten2dCoordinate(neighbor.x, neighbor.y)] >= BLOCKED_GRID_WEIGHT) 					
				{
					closedSet[neighbor.x][neighbor.y] = true;
					closedSetSize++;
					continue;
				}
				else 
				{
					openSet[neighbor.x][neighbor.y] = true;
					openSetSize++;
				}
			}
			else if (tentativeScore >= score[neighbor.x][neighbor.y]) 
			{
				//found a worse path
				continue;
			}
			
			//update or insert values for node
			cameFrom[neighbor.x][neighbor.y] = current;
			score[neighbor.x][neighbor.y] = tentativeScore;

		}
		
	}

}

void BatchPathfind(int* h_fromXs, int* h_fromYs, int* h_toXs, int* h_toYs, int numPaths, int (&h_navGrid)[NAV_GRID_WIDTH][NAV_GRID_HEIGHT])
{
	int cudaStatus = 0;
	int* d_fromXs;
	cudaStatus = cudaMalloc(&d_fromXs, sizeof(int)*numPaths);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc1 failed!");
		return;
	}
	cudaStatus = cudaMemcpy(d_fromXs, h_fromXs, sizeof(int)*numPaths, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy1 failed!");
		return;
	}

	int* d_fromYs;
	cudaStatus = cudaMalloc(&d_fromYs, sizeof(int)*numPaths);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc2 failed!");
		return;
	}
	cudaStatus = cudaMemcpy(d_fromYs, h_fromYs, sizeof(int)*numPaths, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy2 failed!");
		return;
	}

	int* d_toXs;
	cudaStatus = cudaMalloc(&d_toXs, sizeof(int)*numPaths);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc3 failed!");
		return;
	}
	cudaStatus = cudaMemcpy(d_toXs, h_toXs, sizeof(int)*numPaths, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy3 failed!");
		return;
	}
	
	int* d_toYs;
	cudaStatus = cudaMalloc(&d_toYs, sizeof(int)*numPaths);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc4 failed!");
		return;
	}
	cudaStatus = cudaMemcpy(d_toYs, h_toYs, sizeof(int)*numPaths, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy4 failed!");
		return;
	}
	
	int* d_flatNavGrid;
	cudaStatus = cudaMalloc(&d_flatNavGrid, sizeof(int)*NAV_GRID_HEIGHT*NAV_GRID_WIDTH);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc5 failed!");
		return;
	}
	cudaStatus = cudaMemcpy(d_flatNavGrid, h_navGrid, sizeof(int)*numPaths, cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy5 failed!");
		return;
	}

	IntPair** d_returnedPaths;
	cudaStatus = cudaMalloc(&d_returnedPaths, sizeof(int)*NAV_GRID_HEIGHT*NAV_GRID_WIDTH);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc6 failed!");
		return;
	}

	int* d_length;
	cudaStatus = cudaMalloc(&d_length, sizeof(int)*NAV_GRID_HEIGHT*NAV_GRID_WIDTH);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMalloc7 failed!");
		return;
	}

	BatchPathFindKernel <<<1, 1>>>(d_fromXs, d_fromYs, d_toXs, d_toYs, numPaths, d_flatNavGrid, d_returnedPaths, d_length);

	IntPair** h_returnedPaths = (IntPair**)malloc(sizeof(int)*NAV_GRID_HEIGHT*NAV_GRID_WIDTH);
	cudaStatus = cudaMemcpy(h_returnedPaths, d_returnedPaths, sizeof(int)*numPaths, cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy6 failed!");
		return;
	}

	int* h_length = (int*)malloc(sizeof(int));
	cudaStatus = cudaMemcpy(h_length, d_length, sizeof(int)*numPaths, cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaMemcpy7 failed!");
		return;
	}
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
	/*navGrid[2][0] = BLOCKED_GRID_WEIGHT;
	navGrid[2][1] = BLOCKED_GRID_WEIGHT;
	navGrid[2][2] = BLOCKED_GRID_WEIGHT;
	navGrid[2][3] = BLOCKED_GRID_WEIGHT;
	navGrid[2][4] = BLOCKED_GRID_WEIGHT;
	/*navGrid[2][5] = BLOCKED_GRID_WEIGHT;
	navGrid[3][5] = BLOCKED_GRID_WEIGHT;
	navGrid[4][5] = BLOCKED_GRID_WEIGHT;
	navGrid[5][5] = BLOCKED_GRID_WEIGHT;
	navGrid[6][5] = BLOCKED_GRID_WEIGHT;
	navGrid[7][5] = BLOCKED_GRID_WEIGHT;
	navGrid[8][5] = BLOCKED_GRID_WEIGHT;
	navGrid[8][4] = BLOCKED_GRID_WEIGHT;
	navGrid[8][3] = BLOCKED_GRID_WEIGHT;
	navGrid[8][2] = BLOCKED_GRID_WEIGHT;*/

	const int NumPaths = 1;

	int fromXs[NumPaths] = { 0 };
	int fromYs[NumPaths] = { 0 };
	int toXs[NumPaths] = { 1 };
	int toYs[NumPaths] = { 1 };

	BatchPathfind(fromXs, fromYs, toXs, toYs, NumPaths, navGrid);

    return 0;
}

