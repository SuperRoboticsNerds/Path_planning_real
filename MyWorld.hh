//
// = FILENAME
//    test.hh
//
// = AUTHOR(S)
//    Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef MyWorld_hh
#define MyWorld_hh

#include "World.hh"
#include "circle.hh"
#include "rectangle.hh"

#include <vector>

/**
 * The base class for represents the world
 */
class MyWorld : public World {
public:

  /**
   * Constructs an empty world
   */
	MyWorld();

  bool readObstacles(std::istream &fs);

  /**
   * Destructor deletes all geometric objects in the world
   */
  ~MyWorld();

  /**
   * Use this function to check if a certain point collides with any
   * of the obstales in the world
   * 
   * @param x x-coordinate of point to check for collision
   * @param y y-coordinate of point to check for collision
   * @return true if point (x,y) collides with any of the obstacles
   */
  bool collidesWith(double x, double y);

  /**
   * This function will go through all the obstacles in the world and
   * ask for them to to write matlab display code to the stream.
   *
   * @param fs reference to and output stream, for example an fstream (file)
   * @return N/A
   */
  void writeMatlabDisplayCode(std::ostream &fs);

protected:
    std::vector<obstacle*> m_Obstacles;
};

#endif // test_hh
