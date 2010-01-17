/*
 *  Point.cpp
 *  
 *  Copyright (C) 2009 Chau Nguyen
 *
 *  License: Modified BSD Software License Agreement
 */

#include "Point.h"
Point::Point(){
	x=0;
	y=0;
	z=0;
	define=false;
}
Point::Point(double x1, double y1, double z1){
	x = x1;
	y = y1;
	z = z1;
	define=true;
}
Point::Point(double x1, double y1, double z1, double theta1){
	x = x1;
	y = y1;
	z = z1;
	theta = theta1; //theta in degree
	define=true;
}
bool Point::isDefined(){return define;}
double Point::getX(){return x;}
double Point::getY(){return y;}
double Point::getZ(){return z;}
double Point::getAngle(){return theta;}
void Point::setX(double x1){x = x1;}
void Point::setY(double y1){y = y1;}
void Point::setZ(double z1){z = z1;}
void Point::setAngle(double theta1){theta = theta1;}

