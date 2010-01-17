/*
 *  Cell.cpp
 *  
 *  Copyright (C) 2009 Chau Nguyen
 *
 *  License: Modified BSD Software License Agreement
 */
#include "Point.h"
#include "Cell.h"
#include <vector>
#include <math.h>

using namespace std;

Cell::Cell(){
	defined = false;
	reachable = -1; //not reachable by any segments
	segNum = -1; //segment number is not defined
	sumX = 0; //sumX and sumY is used to compute the average of all points in the cell
	sumY = 0;
	unitHeight=0.25;
	size = 0;
}

void Cell::setRow(int r){
	row = r;
}

void Cell::setCol(int c){
	col = c;
}

void Cell::clear(){
	defined = false;
	reachable = -1; //not reachable by any segments
	segNum = -1; //segment number is not defined
	sumX = 0; //sumX and sumY is used to compute the average of all points in the cell
	sumY = 0;
	unitHeight=0.25;
	size = 0;
	list.clear();
}

void Cell::setHeight(Point& p){
	int h = (int) (floor(p.getZ()/unitHeight));
	//when no point in the cell, set pt to be p
	if (!defined){
		defined = true;
		height = h;
		pt = p;
		sumX += p.getX();
		sumY += p.getY();
		size++;
	}
	//choose the highest points among all points with the same unit height to set to pt
	else if (height<h || (height==h && p.getZ()> pt.getZ())){
		height = h; 
		pt = p;
		sumX = p.getX();
		sumY = p.getY();
		size = 1;
	}
	else if (height == h){
		sumX += p.getX();
		sumY += p.getY();
		size++;
	}
	list.push_back(p);
}

void Cell::setReachable(int n){reachable = n;}

int Cell::reachableFrom(){return reachable;}

void Cell::setSegNum(int s){segNum = s;}

bool Cell::isDefined(){return defined;}

int Cell::getHeight(){return height;}

int Cell::getSegNum(){return segNum;}

int Cell::getRow(){return row;}

int Cell::getCol(){return col;}

double Cell::getX(){return (double)(sumX/((double)size));}

double Cell::getY(){return (double)(sumY/((double)size));}

Point& Cell::getPoint(){return pt;}

vector<Point>& Cell::getAllPoints(){return list;}

