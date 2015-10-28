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

#include "rectangle.hh"

#include <iostream>

#include <cmath>


Rectangle::Rectangle(std::istream &fs)
	: obstacle(fs)
{
	fs >> m_Width;
	fs >> m_Height;
	fs >> m_Angle;
}

Rectangle::~Rectangle()
{
}

bool Rectangle::collidesWith(double x, double y)
{
    double dx = x - m_X;
    double dy = y - m_Y;
    double X_rot = m_X + cos(m_Angle)*dx + sin(m_Angle)*dy;
    double Y_rot = m_Y - sin(m_Angle)*dx + cos(m_Angle)*dy;
    double min_X = m_X - m_Width/2;
    double max_X = m_X + m_Width/2;

    if(X_rot < min_X || X_rot > max_X) return false;

    double min_Y = m_Y - m_Height/2;
    double max_Y = m_Y + m_Height/2;

    if(Y_rot < min_Y || Y_rot > max_Y) return false;

    return true;
}


void Rectangle::writeMatlabDisplayCode(std::ostream &fs)
{
    double x [] = {-m_Width/2 , m_Width/2 , m_Width/2 , -m_Width/2};
    double y [] = {-m_Height/2 , -m_Height/2 , m_Height/2 , m_Height/2};

    double x_tr [4]; //transform
    double y_tr [4]; //transform


    for(int i = 0 ; i<4 ;i++)
    {
        x_tr [i]= m_X + cos(m_Angle)*x[i] - sin(m_Angle)*y[i];
        y_tr [i]= m_Y + sin(m_Angle)*x[i] + cos(m_Angle)*y[i];
    }
    //fs << "plot(["<< x_tr[0]<<","<<x_tr[1]<<","<< x_tr[2]<<","<< x_tr[3] << "],["<< y_tr[0]<<","<<y_tr[1]<<","<< y_tr[2]<<","<< y_tr[3]<<"]);" << std::endl;
    for(int i = 0 ; i<4 ;i++)
    {
        fs << "plot(["<< x_tr[i]<<" "<<x_tr[(i+1)%4]<<"],["<< y_tr[i]<<" "<< y_tr[(i+1)%4] << "]);" << std::endl;
    }


}


