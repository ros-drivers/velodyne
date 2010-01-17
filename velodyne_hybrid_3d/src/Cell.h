/*
 *  Cell.h
 *  
 *  Copyright (C) 2009 Chau Nguyen
 *
 *  License: Modified BSD Software License Agreement
 */
#ifndef CELL_H
#define CELL_H
#include "Point.h"
#include <vector>

using namespace std;

class Cell{
private:
	bool defined;
	int reachable,height,segNum,row,col,size;
	Point pt;
	vector<Point> list;
	double sumX,sumY;
public:
	Cell();
	void clear();
	double unitHeight;
	void setRow(int r);
	void setCol(int c);
	void setHeight(Point& p);
	void setReachable(int n);
	int reachableFrom();
	void setSegNum(int s);
	bool isDefined();
	int getHeight();
	int getSegNum();
	int getRow();
	int getCol();
	double getX();
	double getY();
	Point& getPoint();
	vector<Point>& getAllPoints();
};
#endif

