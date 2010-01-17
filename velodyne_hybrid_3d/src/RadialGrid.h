/*
 *  RadialGrid.h
 *  
 *  Copyright (C) 2009 Chau Nguyen
 *
 *  License: Modified BSD Software License Agreement
 */
#ifndef RADIALGRID_H
#define RADIALGRID_H
#include "Point.h"
#include "Cell.h"
#include <vector>
using namespace std;
class RadialGrid{
private:
	int grid_rows; // = length of r_bench = 55
	int grid_cols; // grid = n/deltaTheta
	int maxSegmentNum; //500 is the maximum number of segments
	static const double r_bench[55];
	static const double rad[55];
	int min_cell_to_fit_plane;
	bool segment0TooSmall;
	double deltaTheta; //2 degree for each theta
	double width; //3 meter
	int segmentNum;
	Cell** grid;
	vector<double*> planes;
	vector< vector<Cell> > listOfCellList; //list of cells in each segment
	vector<Cell*>** path; //used to determine path from one segment to another if they are reachable
						 //path[i][j] stores the neighbor cells between segment i and segment j
	vector<Cell*>** boundary;
	double** maxDistance; //maxDistance[i][j] stores the with of the continuous neighbor cells from i to j
	void segmenting(); 
	void recursiveSegmenting(int row, int col, int height, int segnum, vector<Cell>& cellList, vector<Cell*>& neighbor);
	void merging();
	bool reachable(Cell& one, Cell& two);
	void crawl(int seg, int r, int c, vector<Cell*>& drivable, bool visited[55][90]);
	void plane_fitting(vector<Cell>& l, double X[]);
	
public:
	RadialGrid();
	~RadialGrid() { 
		for (int i=0;i<grid_rows;i++) delete [] grid[i];
		delete [] grid;
		for (int i=0;i<maxSegmentNum-1;i++)
		{
			delete [] maxDistance[i];
			delete [] path[i];
			delete [] boundary[i];
		}
		delete [] maxDistance;
		delete [] path;
		delete [] boundary;
	};
	void addPoint(Point& p); //add a point to a grid
	bool finishAddingPoints(); //after adding all points, all computation starts
	void clear();
	int getSegmentNumber();
	int getGridRows();
	int getGridCols();
	Cell** getGrid();
};
#endif

