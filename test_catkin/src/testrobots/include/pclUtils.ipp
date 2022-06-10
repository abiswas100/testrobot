
//Implementation file

#include <cmath>
#include <iostream>




// void testrobots::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
//                                       typename pcl::PointCloud<PointType>::Ptr targetCloud,
//                                       unsigned xmin, unsigned xmax,
//                                       unsigned ymin, unsigned ymax)
// {
//   copyPointCloud(*sourceCloud, *targetCloud);
//   unsigned xmin = 20;
//   unsigned xmax = 40;
//   unsigned ymin = 10;
//   unsigned ymax = 30;
//   unsigned imageWidth = 230;//sourceCloud->width; 
//   unsigned imageHeight = 100;//sourceCloud->height;
//   double nan = std::nan("");
//   PointType nanPt(nan, nan, nan);
//   for(unsigned row =0; row < imageHeight; ++row) {
//     for(unsigned col =0; col < imageWidth; ++col) {
//       unsigned index = row * imageWidth  + col;
//       if((col < xmin) || (xmax < col) || (row < ymin) || (ymax < row))  {
//          targetCloud->operator[](index) = nanPt;
//       }
//     }
//   }
// }
template<typename PointType>
void testrobots::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
                                      typename pcl::PointCloud<PointType>::Ptr targetCloud,
                                      unsigned imageWidth,
                                      unsigned xmin, unsigned xmax,
                                      unsigned ymin, unsigned ymax)
{
  double nan = std::nan("");
  PointType nanPt(nan, nan, nan);
//   for(unsigned row =0; row < image
  // unsigned x_delta = xmax - xmin; // width of box  = 230
  // unsigned y_delta = ymax - ymin; // height of box = 100
  ROS_INFO_STREAM("getting before loop");
  // copyPointCloud(*sourceCloud, *targetCloud);
    for (unsigned r = 0; r < 1081; r ++){     //imageheight = 1080
    for (unsigned c = 0; c < 1921;  c ++){   // imagewidth = 1920
      // unsigned i = (c-1)*1080 + r;
      // unsigned i = (r-1)*(1063-563) + c;
      
      if((c > 664 && c < 894) && (r > 563 && r < 1063)){
        // i  = row_number * total_columns + col_number
        unsigned i = (r) * (1080) + c ;
        // if(i < 2074600){
          std::cout << i << std::endl;
        targetCloud->push_back( sourceCloud->operator[](i));
        // }
      }
    }
  }
    // use this loop to check the number of points on the pointcloud
  // unsigned i = 1;
  // while( i > 0){
  //   std::cout << i << std::endl;
  //   targetCloud->push_back( sourceCloud->operator[](i));
  //   i = i +1;
  }


// }

  // for(unsigned row =0; row < x_delta; ++row) {
  //   for(unsigned col =0; col < y_delta; ++col) {
  //     unsigned x = xmin + col;  // xmin = 664 ;  x = 664 + (1,2,3,4,5 .... 230)
  //     unsigned y = ymin + row;  // ymin = 563  ; y = 563 + (12,3,4,5, ..... 100) 
  //     unsigned index = y * imageWidth + x;  // image width = 1920; y*imagewidth + x
  //     if (index <= 2073612){
  //         targetCloud->push_back( sourceCloud->operator[](index));
  //     }
  //     else{
  //       std::cout << index << std::endl;
  //       return;
  //     }
      
      //  std::cout << "image width = " << imageWidth << std::endl;
      //  std::cout << "index = " << index << std::endl;
      //  try{
      //    targetCloud->push_back( sourceCloud->operator[](index)); [x,y,z]
      //  }
      //  catch (std::exception& e){
      //    std::cerr << "Exception caught : " << e.what() << std::endl;
      //  }
//     }
//   }
// }



// template<typename PointType>
// void UNL_Robotics::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
//                                       typename pcl::PointCloud<PointType>::Ptr targetCloud,
//                                       unsigned xmin, unsigned xmax,
//                                       unsigned ymin, unsigned ymax)
// {
//   copyPointCloud(*sourceCloud, *targetCloud);
  
//   unsigned imageWidth = sourceCloud->width;
//   unsigned imageHeight = sourceCloud->height;
//   double nan = std::nan("");
//   PointType nanPt(nan, nan, nan);
//   for(unsigned row =0; row < imageHeight; ++row) {
//     for(unsigned col =0; col < imageWidth; ++col) {
//       unsigned index = row * imageWidth  + col;
//       if((col < xmin) || (xmax < col) || (row < ymin) || (ymax < row))  {
//          targetCloud->operator[](index) = nanPt;
//       }
//     }
//   }
// }

// template<typename PointType>
// void testrobots::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
//                                 typename pcl::PointCloud<PointType>::Ptr targetCloud,
//                                 unsigned xmin, unsigned xmax,
//                                 unsigned ymin, unsigned ymax)
// {
//   //Implement in terms of the other function
//   unsigned imageWidth = sourceCloud->width;
//   unsigned imageHeight = sourceCloud->height;
//   extractFrame<PointType>(sourceCloud, targetCloud, xmin, xmax, ymin, ymax, imageWidth, imageHeight);
// }
// template<typename PointType>
// void testrobots::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
//                                 typename pcl::PointCloud<PointType>::Ptr targetCloud,
//                                 unsigned xmin, unsigned xmax,
//                                 unsigned ymin, unsigned ymax,
//                                 unsigned imageWidth,
//                                 unsigned imageHeight)
// {
//   copyPointCloud(*sourceCloud, *targetCloud);
  
//   double nan = std::nan("");
//   PointType nanPt(nan, nan, nan);
//   for(unsigned row =0; row < imageWidth; ++row) { //imageHeight
//     for(unsigned col =0; col < imageHeight; ++col) { //imageWidth
//       unsigned index = row * imageWidth  + col;
//       if((col < xmin) || (xmax < col) || (row < ymin) || (ymax < row))  {
//          std::cout << "here - " << index <<targetCloud->operator[](index)<< std::endl;
//          targetCloud->operator[](index) = nanPt;
//       }
//     }
//   }
// }
                    