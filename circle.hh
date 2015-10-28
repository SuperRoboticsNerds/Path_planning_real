//
// = FILENAME
//    createCircle.hh
//
// = AUTHOR(S)
//    Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef circle_hh
#define circle_hh

#include "obstacle.hh"
#include <iostream>

/**
 * The base class for represents the world
 */
class Circle : public obstacle {
public:

  /**
   * Constructs an empty world
   */

	Circle(std::istream &fs);

  /**
   * Destructor deletes all geometric objects in the world
   */
	~Circle();

  /**
   * Use this function to check if a certain point collides with any
   * of the obstales in the world
   *
   * @param x x-coordinate of point to check for collision
   * @param y y-coordinate of point to check for collision
   * @return true if point (x,y) collides with any of the obstacles
   */
  
	virtual bool collidesWith(double x, double y);

  /**
   * This function will go through all the obstacles in the world and
   * ask for them to to write matlab display code to the stream.
   *
   * @param fs reference to and output stream, for example an fstream (file)
   * @return N/A
   */
  virtual void writeMatlabDisplayCode(std::ostream &fs);

protected:

  /// The center coordinate of the circle

	double m_Radius;
};

#endif // createCircle_hh
