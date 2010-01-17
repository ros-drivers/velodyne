/*
 *  Point.h
 *  
 *  Copyright (C) 2009 Chau Nguyen
 *
 *  License: Modified BSD Software License Agreement
 */
#ifndef POINT_H
#define POINT_H
class Point
{
private:
	double x,y,z,theta; //theta is in degree
	bool define;
public:
	Point();
	Point(double x1, double y1, double z1);
	Point(double x1, double y1, double z1, double theta1);
	bool isDefined();
	double getX();
	double getY();
	double getZ();
	double getAngle();
	void setX(double x1);
	void setY(double y1);
	void setZ(double z1);
	void setAngle(double theta1);
};
#endif

