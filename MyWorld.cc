//
// = FILENAME
//    test.cc
//
// = AUTHOR(S)
//    Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "MyWorld.hh"
#include "circle.hh"
#include "rectangle.hh"
#include "obstacle.hh"
#include "World.hh"
#include <iostream>
#include <sstream>
#include <cmath>
#include <fstream>
#include <stdlib.h>
#include <vector>
using namespace std;
//main()


bool
MyWorld::readObstacles(std::istream &fs)
{
	std::string key;

	cout << "begin! \n";

	while (!fs.eof())
	{
		fs >> key;

		if (key == "START_AND_GOAL")
		{
		}
		else if (key == "CIRCLE")
		{
            m_Obstacles.push_back(new Circle(fs));
		}
		else if (key == "RECTANGLE")
		{
            m_Obstacles.push_back(new Rectangle(fs));
		}
		else
		{
		}

	}
	return true;
}

MyWorld::MyWorld()
{
}

MyWorld::~MyWorld()
{
}


bool
MyWorld::collidesWith(double x, double y)
{
	for (int i = 0; i < m_Obstacles.size(); i++)
	{
        if (m_Obstacles[i]->collidesWith(x, y))
		{
			return true;
		}
	}

	return false;
}

void
MyWorld::writeMatlabDisplayCode(std::ostream &fs)
{
  for (int i = 0; i < m_Obstacles.size(); i++)
  {
     m_Obstacles[i]->writeMatlabDisplayCode(fs);
  }
/*
fs << "plot("
<< centerX << " + " << radius << "*cos((0:5:360)/180*pi),"
<< centerY << " + " << radius << "*sin((0:5:360)/180*pi));"
<< "hold on;"
<< std::endl;

  fs << "rectangle("
         " 'Position' ,["<< m_Xc_R << " - " << ((m_w_R/2.0)*cos(m_theta_R))<< " - " << ((m_h_R/2.0)*sin(m_theta_R)) << ","
         << m_Yc_R << " - " << (m_h_R/2.0) * cos(m_theta_R)<< " + " << ((m_w_R/2.0) * sin(m_theta_R)) << ","

       << m_Xc_R << " + " << m_w_R/2 * cos(m_theta_R)<< " - " << ((m_h_R/2)*sin(m_theta_R)) << ","
       << m_Yc_R << " + " << m_h_R/2 * cos(m_theta_R) << " + " << ((m_w_R/2.0) * sin(m_theta_R))  << "])"
       << std::endl;

*/
}
