
// /////////////////
// // Point cloud utilities
// //
// // Written by:  Matthew Peavy
// //              Kyungki Kim
// // (c) 2021
// //

// #ifndef TESTROBOTS_PCLUTILS_H
// #define TESTROBOTS_PCLUTILS_H

// #include <pcl/point_cloud.h>

// namespace testrobots {

//   //Extracts the points from a point cloud that fall within the frame (pixel) coordinates.
//   //Supply a source cloud which is read from and a target cloud which is written to.
//   //Also, the frame coordinates (the "min" coordinates are the upper-left corner,
//   // the "max" coords are the lower-right corner, in terms of pixels).
//   //This version assumes a cloud that is "organized", ie, it has definte
//   // width and height (usually 640 x 480), and is not 1-dimensional (size 307200)
//   template<typename PointType>
//   void extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
//                     typename pcl::PointCloud<PointType>::Ptr targetCloud,
//                     unsigned xmin, unsigned xmax,
//                     unsigned ymin, unsigned ymax);

//   //This version assumes a cloud that is not "organized", ie, it has only
//   // one dimension (often the size 640x480 = 307200). In this case, pass in the
//   // width of the image that we want to stride with (usually 640).
//   template<typename PointType>
//   void extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
//                     typename pcl::PointCloud<PointType>::Ptr targetCloud,
//                     unsigned xmin, unsigned xmax,
//                     unsigned ymin, unsigned ymax,
//                     unsigned imageWidth,
//                     unsigned imageHeight);
// };

// #include "pclUtils.ipp"

// #endif


#ifndef TESTROBOTS_PCLUTILS_H
#define TESTROBOTS_PCLUTILS_H

#include <pcl/point_cloud.h>

namespace testrobots {

  //Extracts the points from a point cloud that fall within the frame (pixel) coordinates.
  //Supply a source cloud which is read from and a target cloud which is written to.
  //Also, the frame coordinates (the "min" coordinates are the upper-left corner,
  // the "max" coords are the lower-right corner, in terms of pixels)
  template<typename PointType>
  void extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
                    typename pcl::PointCloud<PointType>::Ptr targetloud,
                    unsigned xmin, unsigned xmax,
                    unsigned ymin, unsigned ymax);
};

#include "pclUtils.ipp"

#endif

