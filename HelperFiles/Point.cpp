#include "../PCH/pch.hpp"
#include "../include/Point.hpp"


Point operator+( Point & pt1,  Point & pt2)
{
	double x=pt2.getX()+pt1.getX();
       double y= pt2.getY()+pt1.getY();
       return Point(x,y);
}
Point operator-(Point & pt1, Point & pt2)
{
	double x=pt2.getX()-pt1.getX();
       double y= pt2.getY()-pt1.getY();
       return Point(x,y);
}

Point Point::operator+(double shift)
{
	return	Point(x+shift,y+shift);
}
Point Point::operator+=(const Point& pt)
{
	x+=pt.x;
	y+=pt.y;
	return *this;
}
Point Point::operator+=(double shift)
{
	x+=shift;
	y+=shift;
	return *this;
}


Point Point::operator-(double shift)
{
	return Point(x-shift,y-shift);
}

Point Point::operator-=(const Point& pt)
{
	x-=pt.x;
	y-=pt.y;
	return *this;
}

Point Point::operator-=(double shift)
{
	x-=shift;
	y-=shift;
	return *this;
}



Point Point::operator/(double scale)
{
	return Point(x/scale,y/scale);
}

Point Point::operator/=(const Point& pt)
{
	x/=pt.x;
	y/=pt.y;
	return *this;
}

Point Point::operator/=(double shift)
{
	x/=shift;
	y/=shift;
	return *this;
}


Point Point::operator*(double scale)
{
	return Point(x*scale,y*scale);
}


Point Point::operator*=(const Point & pt)
{
	x*=pt.x;
	y*=pt.y;
	return *this;
}

Point Point::operator*=(double scale)
{
	x*=scale;
	y*=scale;
	return *this;
}
double Point::getX (){ return x;}
double Point::getY (){ return y;}


double Point::dotProd(const Point &P)
{
	return 	this->x*P.x+this->y*P.y;
}

double Point::crossProd(const Point &P)
{
	return this->x*P.y-this->y*P.x;
}
