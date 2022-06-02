
//Implementation file

#include <cmath>



template<typename PointType>
void testrobots::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
                                typename pcl::PointCloud<PointType>::Ptr targetCloud,
                                unsigned xmin, unsigned xmax,
                                unsigned ymin, unsigned ymax)
{
  //Implement in terms of the other function
  unsigned imageWidth = sourceCloud->width;
  unsigned imageHeight = sourceCloud->height;
  extractFrame<PointType>(sourceCloud, targetCloud, xmin, xmax, ymin, ymax, imageWidth, imageHeight);
}


template<typename PointType>
void testrobots::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
                                typename pcl::PointCloud<PointType>::Ptr targetCloud,
                                unsigned xmin, unsigned xmax,
                                unsigned ymin, unsigned ymax,
                                unsigned imageWidth,
                                unsigned imageHeight)
{
  copyPointCloud(*sourceCloud, *targetCloud);
  
  double nan = std::nan("");
  PointType nanPt(nan, nan, nan);
  for(unsigned row =0; row < imageHeight; ++row) {
    for(unsigned col =0; col < imageWidth; ++col) {
      unsigned index = row * imageWidth  + col;
      if((col < xmin) || (xmax < col) || (row < ymin) || (ymax < row))  {
         targetCloud->operator[](index) = nanPt;
      }
    }
  }
}
                    