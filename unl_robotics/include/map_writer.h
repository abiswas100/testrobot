///////////////////////////////////////////////////
// 
//  A node to edit the pgm map in real-time placing 
//  recognized objects within the map scene
//
//  Written by:  Matthew Peavy
//               Kyungki Kim
//  (c) 2021
//

#ifndef TESTROBOTS_MAP_WRITER
#define TESTROBOTS_MAP_WRITER

#include "convexHull.h"
//std
#include <string>
#include <tuple>
#include <vector>

namespace testrobots {

  class map_writer {
  public:
    map_writer();
    ~map_writer();

    //The path_to_pgm should be a path the will be prepended
    // to the pgm file name found in the yaml file.
    //It does not need to be terminated with a forward slash.
    bool insertObject(const std::string& yaml_filepath,
                      const std::string& path_to_pgm,
                      const std::vector<Point2D> convexHull);
    
  private:
    std::string m_pgmFilePath;
    double m_resolution;
    std::tuple<double, double, double> m_origin;
    unsigned m_strideX;   //Width
    unsigned m_strideY;   //Height
    
    void plotLine(const Point2D& pt1, const Point2D& pt2);
  };
}

#endif

