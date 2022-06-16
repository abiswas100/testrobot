
//Implementation file

#include <cmath>

template<typename PointType>
void UNL_Robotics::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
                                      typename pcl::PointCloud<PointType>::Ptr targetCloud,
                                      unsigned xmin, unsigned xmax,
                                      unsigned ymin, unsigned ymax)
{
  copyPointCloud(*sourceCloud, *targetCloud);
  
  unsigned imageWidth = sourceCloud->width;
  unsigned imageHeight = sourceCloud->height;
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

