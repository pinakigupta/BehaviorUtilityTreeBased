#pragma once
#include "../PCH/pch.hpp"

class Point
{
	double x;
	double y;
public:
	Point():x(0),y(0){};
	Point(double x,double y):x(x),y(y){}

	Point operator+(double shift);
	Point operator+=(const Point& pt);
	Point operator+=(double shift);
	Point operator-(double shift);
	Point operator-=(const Point& pt);
	Point operator-=(double shift);
	Point operator/(double scale);
	Point operator/=(const Point& pt);
	Point operator/=(double shift);
	Point operator*(double scale);
	Point operator*=(const Point & pt);
	Point operator*=(double scale);
	double getX ();
	double getY ();
	double dotProd(const Point &P);
	double crossProd(const Point &P);


	friend std::ostream& operator<<(std::ostream& out,const Point& p)
	{
		out<<p.x<<"\t"<<p.y;
		return out;
	}
		
};

Point operator+( Point & pt1,  Point & pt2);
Point operator-(Point & pt1, Point & pt2);

