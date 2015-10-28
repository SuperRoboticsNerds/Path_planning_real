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

//#include "createCircle.hh"

#include <iostream>
#include "obstacle.hh"
#include <cmath>


obstacle::obstacle(std::istream &fs)
{
        fs >> m_X;
        fs >> m_Y;
}

obstacle::~obstacle()
{
}

bool obstacle::collidesWith(double x, double y)
{
	return false;
}


void obstacle::writeMatlabDisplayCode(std::ostream &fs)
{
  std::cerr << std::endl
            << "World::writeMatlabDisplayCode not overloaded"
            << std::endl
            << "Not writing any matlab code"
            << std::endl << std::endl;
}

