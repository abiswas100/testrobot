
#ifndef TESTROBOTS_CONVEXHULL_H
#define TESTROBOTS_CONVEXHULL_H

#include<vector>

namespace testrobots {

  struct Point2D {    //define points for 2d plane
     double x, y;
  };

  std::vector<Point2D> findConvexHull(std::vector<Point2D> points);
  
}

#endif

