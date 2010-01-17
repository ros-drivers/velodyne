/*
 *  RadialGrid.cpp
 *  
 *  Copyright (C) 2009 Chau Nguyen
 *
 *  License: Modified BSD Software License Agreement
 */

#include "RadialGrid.h"
#include "Point.h"
#include "Cell.h"
#include <vector>
#include <math.h>

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
using namespace std;

//maximum radius is 500 meter, any point beyond 100 meter from the center is in the last r bench
const double RadialGrid::r_bench[55] = {4.83,4.94,5.06,5.18,5.32,5.45,5.60,5.75,5.91,6.07,6.24,6.44,6.63,6.82,7.03,7.26,7.49,7.75,8.02,8.31,8.62,8.95,
9.31,9.69,10.11,10.56,11.06,11.60,12.19,12.85,13.59,14.33,15.00,15.64,16.34,17.11,17.95,18.88,19.91,21.05,22.34,
23.79,25.43,27.32,29.51,32.08,35.14,38.85,43.43,49.23,56.83,67.21,82.29,106.26,500};
const double RadialGrid::rad[55] = {4.78,4.88,5.00,5.12,5.25,5.38,5.53,5.67,5.83,5.99,6.15,6.33,6.55,6.72,6.92,7.14,7.37,7.62,7.88,8.16,8.46,8.78,9.12,
9.49,9.89,10.32,10.80,11.32,11.88,12.50,13.20,13.97,14.69,15.30,15.98,16.71,17.51,18.40,19.37,20.45,21.66,23.02,24.55,
26.31,28.33,30.69,33.48,36.81,40.89,45.97,52.49,61.16,73.26,91.32,500};

void RadialGrid::segmenting()
{
	segmentNum = 0;
	int c=0;
	int col = 0;
	int row = -1;
	int initialHeight=0;
	int i,j;		
	while (row==-1){
		for(i=0;i<grid_rows;i++){
			for(j=0;j<grid_cols;j++){
				if (grid[i][j].isDefined()&& grid[i][j].getHeight()==initialHeight){
					row = i;
					col = j;
				}
				int k = (grid_cols-j)%grid_cols;
				if (grid[i][k].isDefined()&& grid[i][k].getHeight()==initialHeight){
					row = i;
					col = k;
				}
				if (row!=-1) break;
			}
			if (row!=-1) break;
		}
		initialHeight++;
	}
	vector<Cell*> neighborList;
	neighborList.push_back(&grid[row][col]);
	int k=0;
	while (k<(int)neighborList.size()){
		Cell mycell = (Cell)(*neighborList.at(k));
		if (mycell.getSegNum() == -1){
			vector<Cell> cellList;
			double pl[3];
			recursiveSegmenting(mycell.getRow(),mycell.getCol(),mycell.getHeight(),segmentNum,cellList,neighborList);
			listOfCellList.push_back(cellList);
			if ((int)cellList.size()>=min_cell_to_fit_plane){
				plane_fitting(cellList, pl);
				c++;
			}
			planes.push_back(pl);
			segmentNum++;				
		}		
		k++;
	}

	vector<Cell> cl;
	for(i=0;i<grid_rows;i++)
		for(j=0;j<grid_cols;j++)
			if(grid[i][j].isDefined() && grid[i][j].getSegNum()==-1){
				grid[i][j].setSegNum(segmentNum);
				cl.push_back(grid[i][j]);
			}
	listOfCellList.push_back(cl);
	segmentNum++;
	
	//check if segment 0 is too small
	int left_c = col;
	int right_c = col;
	i=0;
	while(i<grid_cols && grid[row][left_c].getSegNum()==0){
		left_c = (left_c - 1 + grid_cols)% grid_cols;
		i++;
	}
	if (i==grid_cols){
		segment0TooSmall = false;
		return;
	}
	i=0;
	while(i<grid_cols && grid[row][right_c].getSegNum()==0){
		right_c = (right_c + 1) % grid_cols;
		i++;
	}
	if (i==grid_cols){
		segment0TooSmall = false;
		return;
	}
	if (left_c>right_c) left_c = left_c - grid_cols;
	double angle = (right_c - left_c + 1)*deltaTheta;
	double w = 0;
	if (angle>=180) w = 2*rad[row];
	else w = 2*rad[row]*sin(angle/2*M_PI/180);
	if (w<width) segment0TooSmall = true;
}

void RadialGrid::recursiveSegmenting(int row, int col, int height, int segnum, vector<Cell>& cellList, vector<Cell*> & neighbor)
{
	//if this cell does not have any point or this cell already belongs to a segment
	if ((!grid[row][col].isDefined())||(grid[row][col].getSegNum()!=-1)) return;
	if (grid[row][col].getHeight()==height){
		cellList.push_back(grid[row][col]);
		grid[row][col].setSegNum(segnum);
		if (row>0) recursiveSegmenting(row-1,col,height,segnum,cellList,neighbor);
		if (row<grid_rows-1) recursiveSegmenting(row+1,col,height,segnum,cellList,neighbor);
		recursiveSegmenting(row,(col-1+grid_cols)%grid_cols,height,segnum,cellList,neighbor);
		recursiveSegmenting(row,(col+1)%grid_cols,height,segnum,cellList,neighbor);
    	}
	if (fabs(grid[row][col].getHeight()-height)==1) neighbor.push_back(&grid[row][col]);
}

void RadialGrid::merging()
{
//	vector<Cell*>** boundary;
	int i,j;
	bool flag[55][90];
	int m,h,k;
	for(m=0;m<segmentNum-1;m++) // iterate over each segment
	{
		vector<Cell> list = listOfCellList.at(m);
		for(h=0;h<(int)list.size();h++){
			i = list.at(h).getRow();
			j = list.at(h).getCol();
			k = -1;
			if (i>0){
				k = grid[i-1][j].getSegNum();
				if (reachable(grid[i][j],grid[i-1][j])){
					grid[i-1][j].setReachable(m);
					grid[i][j].setReachable(k);
					boundary[m][k].push_back(&grid[i-1][j]);
				}
			}
			if (i<grid_rows-1){
				k = grid[i+1][j].getSegNum();
				if (reachable(grid[i][j], grid[i+1][j])){
					grid[i+1][j].setReachable(m);
					grid[i][j].setReachable(k);
					boundary[m][k].push_back(&grid[i+1][j]);
				}
			}
			k = grid[i][(j-1+grid_cols)%grid_cols].getSegNum();
			if (reachable(grid[i][j],grid[i][(j-1+grid_cols)%grid_cols])){
				grid[i][(j-1+grid_cols)%grid_cols].setReachable(m);
				grid[i][j].setReachable(k);
				boundary[m][k].push_back(&grid[i][(j-1+grid_cols)%grid_cols]);
			}
			k = grid[i][(j+1)%grid_cols].getSegNum();
			if (reachable(grid[i][j], grid[i][(j+1)%grid_cols])){
				grid[i][(j+1)%grid_cols].setReachable(m);
				grid[i][j].setReachable(k);
				boundary[m][k].push_back(&grid[i][(j+1)%grid_cols]);
			}
		}
		
		for(k=0;k<segmentNum-1;k++){
			i=0;
			j=0;
			for(i=0;i<grid_rows;i++)
				for(j=0;j<grid_cols;j++) flag[i][j] = false;
			double maxD = 0; //maximum distance for from m to k
			vector<Cell*> a; //a stores the continuous neighbor cells from m to k 
			int size = (int)boundary[m][k].size();
			for(i=0;i<size;i++){
				vector<Cell*> drivable;
				crawl(m,boundary[m][k].at(i)->getRow(),boundary[m][k].at(i)->getCol(),drivable,flag);
				double d = 0; // d is the max distance of each block of continuous cells
				if (drivable.size()>0){
					if (drivable.size()==1) //calculate the size of the cell
						d = 2*rad[drivable.at(0)->getRow()]*sin(deltaTheta/2*M_PI/180);
					else{
						int l=0;
						for(j=0;j<(int)drivable.size()-1;j++)
							for(l=j+1;l<(int)drivable.size();l++){
								double dist = sqrt(pow(drivable.at(j)->getX()-drivable.at(l)->getX(),2)+pow(drivable.at(j)->getY()-drivable.at(l)->getY(),2));
								if (dist>d) d = dist;
							}
					}
				}
	/*			if ((d>=width) || (m==0 && segment0TooSmall)){
					for(j=0;j<(int)drivable.size();j++)
						grid[drivable.at(j)->getRow()][drivable.at(j)->getCol()].setBoundary();
				}*/
				if (d>maxD){
					maxD = d;
					a = drivable;
				}
			}
			//is segment 0 is only a few cells, just merge it with the adjacent segments
			if (m==0 && segment0TooSmall){
				path[m][k] = a;
				maxDistance[m][k] = maxD;
			}
			else{
				if (maxD>=width) //if the width of these continuous cells are over 3 meter
				{
					path[m][k] = a;
					maxDistance[m][k] = maxD;
				}
				else{
					path[m][k].clear();
					maxDistance[m][k] = 0;
				}
			}
		}
	}
	// if the boundary cells are not continuous or not over 3.0 wide, set their segment number
	// to segmentNum-1 
/*	for(m=0;m<segmentNum-1;m++)
		for(k=0;k<segmentNum-1;k++)
		{
			vector<Cell*> v = boundary[m][k];
			for (h=0;h<(int)v.size();h++){
				Cell c = (Cell)(*v.at(h));
				if (!grid[c.getRow()][c.getCol()].isBoundaryCell())
					grid[c.getRow()][c.getCol()].setSegNum(segmentNum-1);
			}
		}*/
	//after processing all potential reachable segment, reset them to reachable to -1 then
	//set them to their real reachable segment
	
	for(i=0;i<grid_rows;i++)
		for(j=0;j<grid_cols;j++) grid[i][j].setReachable(-1);
	for(m=0;m<segmentNum-1;m++)
		for(k=0;k<segmentNum-1;k++)
			if (maxDistance[m][k]>0){
				for(i=0;i<(int)path[m][k].size();i++) 
					grid[path[m][k].at(i)->getRow()][path[m][k].at(i)->getCol()].setReachable(m);
			}	
	//this part added only for outputing segments.txt, since it reset segment number of any
	//segment reachable from 0 to 0. if we want to find path from one segment to another,
	//we should set them back to their original segment number.
	
	vector<int> queue;
	queue.push_back(0);
	bool visited[segmentNum];
	visited[0]=true;
	for(i=1;i<segmentNum;i++)  visited[i]=false;
	i=0;
	while(i<(int)queue.size()){
		m = queue.at(i);
		for(k=0;k<segmentNum-1;k++)
			if(maxDistance[m][k]>0 && visited[k]==false){
				queue.push_back(k);
				visited[k] = true;
				for(j=0;j<(int)listOfCellList.at(k).size();j++){
					listOfCellList.at(k).at(j);
					int row = listOfCellList.at(k).at(j).getRow();
					int col = listOfCellList.at(k).at(j).getCol();
				//	if (grid[row][col].getSegNum()<segmentNum-1) 
					grid[row][col].setSegNum(0);
				}
			}
		i++;
	}
}

bool RadialGrid::reachable(Cell& one, Cell& two)
{
	if (one.getSegNum()==two.getSegNum()) return false;
	if ((two.getSegNum()==segmentNum-1)||(two.getSegNum()==-1)) return false;
	double* pl1 = planes.at(one.getSegNum());
	double* pl2 = planes.at(two.getSegNum());
	double x1 = one.getX();
	double y1 = one.getY();
	double z11 = x1*pl1[0] + y1*pl1[1] + pl1[2];
	double z12 = x1*pl2[0] + y1*pl2[1] + pl2[2];
	if (fabs(z11-z12)<=one.unitHeight) return true;
	double x2 = two.getX();
	double y2 = two.getY();
	double z22 = x2*pl2[0] + y2*pl2[1] + pl2[2];
	double z21 = x2*pl1[0] + y2*pl1[1] + pl1[2];
	if (fabs(z22-z21)<=one.unitHeight) return true;
	return false; 
}

void RadialGrid::crawl(int seg, int r, int c, vector<Cell*>& drivable, bool visited[55][90])
{
	if (visited[r][c]==true) return;
	if (grid[r][c].reachableFrom()!=seg) return;
	visited[r][c]=true;
	drivable.push_back(&grid[r][c]);
	if (r>0){
		crawl(seg,r-1,c,drivable,visited);
		crawl(seg,r-1,(c-1+grid_cols)%grid_cols,drivable,visited);
		crawl(seg,r-1,(c+1)%grid_cols,drivable,visited);
    	}
	if (r<grid_rows-1){ 
		crawl(seg,r+1,c,drivable,visited);
		crawl(seg,r+1,(c-1+grid_cols)%grid_cols,drivable,visited);
		crawl(seg,r+1,(c+1)%grid_cols,drivable,visited);
	}
	crawl(seg,r,(c-1+grid_cols)%grid_cols,drivable,visited);
	crawl(seg,r,(c+1)%grid_cols,drivable,visited);
}

void RadialGrid::plane_fitting(vector<Cell>& l, double X[])
{
	double A[3][3];
	double B[3];
	//initialize all matrices to all zeros
	for(int i=0;i<3;i++){
		B[i]=0;
		X[i]=0;
		for(int j=0;j<3;j++) A[i][j]=0;
	}
	//filling value for maxtrix
	for(int i=0;i<(int)l.size();i++){
		double xi = l.at(i).getPoint().getX();
		double yi = l.at(i).getPoint().getY();
		double zi = l.at(i).getPoint().getZ();
		//matrix A
		A[0][0] += xi*xi;
		A[0][1] += xi*yi;
		A[0][2] += xi;
		A[1][1] += yi*yi;
		A[1][2] += yi;
		//matrix B
		B[0] += zi*xi;
		B[1] += zi*yi;
		B[2] += zi;
	}
	A[1][0] = A[0][1];
	A[2][0] = A[0][2];
	A[2][1] = A[1][2];
	A[2][2] = l.size();
	//compute determinant of A
	double det = A[0][0]*A[1][1]*A[2][2] + A[0][1]*A[1][2]*A[2][0] + A[0][2]*A[1][0]*A[2][1] - A[2][0]*A[1][1]*A[0][2] - A[2][1]*A[1][2]*A[0][0] - A[2][2]*A[1][0]*A[0][1];
	//matrixX contains the equation for the plane.
	//now find the inverse of A
	double c[3][3];
	c[0][0] = (A[1][1]*A[2][2]-A[2][1]*A[1][2]);
	c[0][1] = (A[2][0]*A[1][2]-A[1][0]*A[2][2]);
	c[0][2] = (A[1][0]*A[2][1]-A[2][0]*A[1][1]);
	c[1][0] = (A[2][1]*A[0][2]-A[0][1]*A[2][2]);
	c[1][1] = (A[0][0]*A[2][2]-A[2][0]*A[0][2]);
	c[1][2] = (A[2][0]*A[0][1]-A[0][0]*A[2][1]);
	c[2][0] = (A[0][1]*A[1][2]-A[1][1]*A[0][2]);
	c[2][1] = (A[1][0]*A[0][2]-A[0][0]*A[1][2]);
	c[2][2] = (A[0][0]*A[1][1]-A[1][0]*A[0][1]);
	//matrixX contains the equation for the plane.
	for(int i=0;i<3;i++) X[i] = (c[i][0]*B[0] + c[i][1]*B[1] + c[i][2]*B[2])/det;		
}

RadialGrid::RadialGrid()
{
	deltaTheta = 4.0;
	grid_rows = 55;
	grid_cols = (int)(360/deltaTheta);
	width = 3.0;  //car width is 2.12 meter
	segmentNum= 0;
	min_cell_to_fit_plane = 3;
	segment0TooSmall=false;
  	// Allocate space for the matrix
	grid = new Cell* [grid_rows];
	for(int i = 0; i < grid_rows; i++) grid[i] = new Cell[grid_cols];
	for(int i = 0;i<grid_rows;i++)
		for(int j= 0;j<grid_cols;j++){
			grid[i][j].setRow(i);
			grid[i][j].setCol(j);
		}
	maxSegmentNum=500;
	boundary = new vector<Cell*>* [maxSegmentNum-1];
	path = new vector<Cell*>* [maxSegmentNum-1];	
	maxDistance = new double* [maxSegmentNum-1];
	for(int i=0;i<maxSegmentNum-1;i++){
		boundary[i] = new vector<Cell*>[maxSegmentNum-1];
		path[i] = new vector<Cell*>[maxSegmentNum-1];
		maxDistance[i] = new double[maxSegmentNum-1];	
	}
}

void RadialGrid::addPoint(Point& p)
{
	double theta = p.getAngle();
	if (theta >= 360) theta -= 360;
	if (theta < 0) theta += 360;
	double d = sqrt(p.getX()*p.getX() + p.getY()*p.getY());
	int r = -1;
	for(int i = 0; i < grid_rows; i++)
		if (d < r_bench[i]){
			r = i;
			break;
		}
	if (r > -1) grid[r][(int) floor(theta/deltaTheta)].setHeight(p);
}

bool RadialGrid::finishAddingPoints()
{
	int count=0;
	for(int i=0;i<grid_rows;i++){
		for(int j=0;j<grid_cols;j++){
			if (grid[i][j].isDefined())
				count++;
		}
	}
	if (count<0.02*grid_rows*grid_cols) return false;
	segmenting();
	if (segmentNum>maxSegmentNum) return false;
	merging();
	return true;
}	

Cell** RadialGrid::getGrid(){return grid;}

void RadialGrid::clear(){
	for(int i = 0;i<grid_rows;i++)
		for(int j= 0;j<grid_cols;j++) grid[i][j].clear();
	for(int i = 0; i<maxSegmentNum-1;i++)
		for(int j = 0;j<maxSegmentNum-1;j++){
			boundary[i][j].clear();
			path[i][j].clear();
		}
	planes.clear();
	listOfCellList.clear();
}

int RadialGrid::getSegmentNumber(){ return segmentNum;}
int RadialGrid::getGridRows(){ return grid_rows;}
int RadialGrid::getGridCols(){return grid_cols;}
