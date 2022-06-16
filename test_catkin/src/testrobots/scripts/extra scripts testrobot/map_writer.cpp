

// NOTE: this method of copying files is very inefficient and should be updated
//       to making changes directly to the file in place.  This is only a prototype!


#include "map_writer.h"
//std
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iostream>
#include <experimental/filesystem>

namespace {
  
  //A simple implementation of Bresenham's Algoirthm for drawing lines in a pixaled manner
  std::vector<testrobots::Point2D> generateLinePixels(double x1, double y1, double x2, double y2)
  {
    std::vector<testrobots::Point2D> pixels;
    
    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if(steep) {
      std::swap(x1, y1);
      std::swap(x2, y2);
    }
    
    if(x1 > x2) {
      std::swap(x1, x2);
      std::swap(y1, y2);
    }
    
    const double dx = x2 - x1;
    const double dy = fabs(y2 - y1);
    
    double error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = static_cast<int>(y1);
    const int maxX = static_cast<int>(x2);
    
    for(int x=static_cast<int>(x1); x<=maxX; ++x) {
      
      if(steep)
        pixels.push_back(testrobots::Point2D{static_cast<double>(y), static_cast<double>(x)});
      else
        pixels.push_back(testrobots::Point2D{static_cast<double>(x), static_cast<double>(y)});
      
      error -= dy;
      if(error < 0) {
        y += ystep;
        error += dx;
      }
    }

    return pixels;
  }
}

//////////////////////

testrobots::map_writer::map_writer()
{
}

testrobots::map_writer::~map_writer()
{
}

bool testrobots::map_writer::insertObject(const std::string& yaml_filepath,
                                            const std::string& path_to_pgm,
                                            const std::vector<Point2D> convexHull) 
{
  if(convexHull.size() < 2) {
    std::cout << "Fewer than 2 hull points. No lines can be drawn." << std::endl;
    return false;
  }

  //Parse in the yaml file
  std::ifstream yamlFile(yaml_filepath.c_str());
  if(!yamlFile) {
    std::cout << "Couldn't open yaml file with path: " << yaml_filepath << std::endl;
    return false;
  }

  std::string temp, line, pgmFilename;
  yamlFile >> temp;
  yamlFile >> pgmFilename;
  yamlFile >> temp;
  yamlFile >> m_resolution;
  getline(yamlFile, temp, '[');    //Disacrd up to, and discard, the  [
  getline(yamlFile, line, ']');    //Read everything up to, and discard, the  ]  so that line contains a comma-delimited list of numbers
  std::replace(line.begin(), line.end(), ',', ' ');  //Replace commas with spaces for simpler stream extraction
  std::stringstream ss(line);
  ss >> std::get<0>(m_origin) >> std::get<1>(m_origin) >> std::get<2>(m_origin);  

  m_pgmFilePath = path_to_pgm + "/" + pgmFilename;
  
  std::cout << "map_writer node" << std::endl;
  std::cout << "PGM file: " << m_pgmFilePath << std::endl;
  std::cout << "Resolution: " << m_resolution << std::endl;
  std::cout << "Origin: ("
            << std::get<0>(m_origin) << ", "
            << std::get<1>(m_origin) << ", "
            << std::get<2>(m_origin) << ")"
            << std::endl;

  std::ifstream pgmFile(m_pgmFilePath);
  if(!pgmFile) {
    std::cout << "Couldn't open original PGM file. Path = " << m_pgmFilePath << std::endl;
    std::cout << "Must exit." << std::endl;
    exit(EXIT_FAILURE);
  }
  getline(pgmFile, line);
  getline(pgmFile, line);
  pgmFile >> m_strideX;
  pgmFile >> m_strideY;
  //std::cout << "PGM strides: " << m_strideY << " x " << m_strideY << std::endl;
  
  //Iterate over all the points in the hull. Start at the second point (index 1)
  for(unsigned pt = 1; pt < convexHull.size(); ++pt) {
    plotLine(convexHull[pt-1], convexHull[pt]);
  }
  
  //And do the last to first point (but only if more than 2 points -- otherwise we've just got 2 points or 1 line, which has already been done)
  if(convexHull.size() > 2)
    plotLine(convexHull[ convexHull.size()-1 ], convexHull[0]);
  
  return true;
}

void testrobots::map_writer::plotLine(const testrobots::Point2D& pt1, const testrobots::Point2D& pt2)
{
  //Convert these points from global coords to "pixel" points, which is the resolution of the PGM map.
  //Then add on the origin.
  //This will result in each pixel on the line corresponding to its index in the PGM file.
  double x1 = (pt1.x - std::get<0>(m_origin)) / m_resolution;
  double y1 = m_strideY - (pt1.y - std::get<1>(m_origin)) / m_resolution;
  double x2 = (pt2.x - std::get<0>(m_origin)) / m_resolution;
  double y2 = m_strideY - (pt2.y - std::get<1>(m_origin)) / m_resolution;
  std::vector<Point2D> linePixels = generateLinePixels(x1, y1, x2, y2);

  //Put all these pixels into a list and then order it
  std::vector<unsigned> indicesOfFilledPixels;
  for(auto pixel : linePixels)
    indicesOfFilledPixels.push_back(pixel.y * m_strideX + pixel.x);
  std::sort(indicesOfFilledPixels.begin(), indicesOfFilledPixels.end());
  
  std::string line; //Read lines into this

  //Open the input and output files
  std::experimental::filesystem::path inPath(m_pgmFilePath);
  std::ifstream pgmFileIn(inPath);
  if(!pgmFileIn) {
    std::cout << "Couldn't open PGM input file. Must exit." << std::endl;
    exit(EXIT_FAILURE);
  }
  std::experimental::filesystem::path outPath(m_pgmFilePath + ".working");
  std::ofstream pgmFileOut(outPath);
  if(!pgmFileOut) {
    std::cout << "Couldn't open PGM output file. Must exit." << std::endl;
    exit(EXIT_FAILURE);
  }
  
  //Do the first 4 lines, which are mpa data
  getline(pgmFileIn, line);            //The "magic" number indicating file type, usually P# e.g., P2
  pgmFileOut << line << std::endl;
  getline(pgmFileIn, line);            //A comment line
  pgmFileOut << line << std::endl;
  getline(pgmFileIn, line);            // height width
  pgmFileOut << line << std::endl;
  getline(pgmFileIn, line);            //Max grayscale value (usually 255)
  pgmFileOut << line << std::endl;

  unsigned prevIndex(0);          
  for(auto index : indicesOfFilledPixels) {

    //Copy out these lines between indices without change
    unsigned delta = index - prevIndex;
    for(unsigned i=0; i < delta-1; ++i) {   //Note: delta -1
      getline(pgmFileIn, line);
      pgmFileOut << line << std::endl;
    }

    //This is the line we want to set the pixel in
    getline(pgmFileIn, line);
    pgmFileOut << "0" << std::endl;   //Mark this pixel as black. Overwrite the 255

    prevIndex = index;   //Update the previous index to the current and continue
  }

  //Now copy over the remainder of the file
  while(getline(pgmFileIn, line)) {
    pgmFileOut << line << std::endl;
  }

  ////////////

  //Close the files
  pgmFileIn.close();
  pgmFileOut.close();

  //Copy the "working" file to the original
  bool copyResult = std::experimental::filesystem::copy_file(outPath, inPath,
                                                             std::experimental::filesystem::copy_options::overwrite_existing);
  if(!copyResult) {
    std::cout << "ERROR copying file from: " << outPath.string() << "  to  " << inPath.string() << std::endl;
    std::cout << "Not going to exit, but this is severe." << std::endl;
  }
}
