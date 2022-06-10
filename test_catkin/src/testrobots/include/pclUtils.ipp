
//Implementation file

#include <cmath>
#include <iostream>


template<typename PointType>
void testrobots::extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
                                      typename pcl::PointCloud<PointType>::Ptr targetCloud,
                                      unsigned imageWidth,
                                      unsigned xmin, unsigned xmax,
                                      unsigned ymin, unsigned ymax)
{ 
  unsigned w = sourceCloud->width;
  unsigned h = sourceCloud->height;
  double nan = std::nan("");
  PointType nanPt(nan, nan, nan);

  ROS_INFO_STREAM("getting before loop");
  // copyPointCloud(*sourceCloud, *targetCloud);
    for (unsigned r = 0; r < h; r ++){     //imageheight = 1080
    for (unsigned c = 0; c < w;  c ++){   // imagewidth = 1920
      // unsigned i = (c-1)*1080 + r;
      // unsigned i = (r-1)*(1063-563) + c;
      
      if((c > 664 && c < 894) && (r > 563 && r < 1063)){
        // i  = row_number * total_columns + col_number
        unsigned i = (r) * (1080) + c ;
        if(i < w*h){  //so that it doesn't crash of segmentation faults
          std::cout << i << std::endl;
        targetCloud->push_back( sourceCloud->operator[](i));
        }
      }
    }
  }
    // use this loop below to check the number of points on the pointcloud
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


