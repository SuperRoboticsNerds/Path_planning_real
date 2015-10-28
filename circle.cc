//
// = FILENAME
//    createCircle.cc
//    
// = AUTHOR(S)
//    Patric Jensfelt
//
/*----------------------------------------------------------------------*/

//
// = FILENAME
//    createCircle.cc
//
// = AUTHOR(S)
//    Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "circle.hh"
#include <iostream>
#include <cmath>


Circle::Circle(std::istream &fs)
	: obstacle(fs)
{
	fs >> m_Radius;
}

Circle::~Circle()
{
}

bool Circle::collidesWith(double x, double y)
{
    double dx = x - m_X;
    double dy = y - m_Y;

    return (dx*dx + dy*dy) < m_Radius*m_Radius;
}


void Circle::writeMatlabDisplayCode(std::ostream &fs)
{
	fs << "plot("
		<< m_X << " + " << m_Radius << "*cos((0:5:360)/180*pi),"
		<< m_Y << " + " << m_Radius << "*sin((0:5:360)/180*pi));"
		<< "hold on;"
		<< std::endl;
}

