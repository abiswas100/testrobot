//include dirs

#include <ros/ros.h>
#include <testrobots/Boundingbox.h>
#include <pclUtils.h>
#include <convexHull.h>
#include <map_writer.h>
#include <string>
#include <vector>
#include<iostream>
#include<algorithm>
#include<stack>

// ROS Topics
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h>  
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
// pcl
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
// PCL - PCA
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
// PCL - visualization
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
// std
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

//for sampling from example.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/lexical_cast.hpp>
pcl::PCDWriter m_writer;
#include <chrono>
using namespace std::chrono;
static const std::string PCL_TOPIC = "/camera/depth/points";
#define YELLOW  "\033[33m"      /* Yellow */
#define GREEN   "\033[32m"      /* Green */
#define MAGENTA "\033[35m"      /* Magenta */
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pclXYZ(new pcl::PointCloud<pcl::PointXYZ>);

  unsigned xmin = 0;
  unsigned xmax = 0;
  unsigned ymin = 0;
  unsigned ymax = 0;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ros::Publisher plane_segmented;
ros::Publisher except_plane;
ros::Publisher voxel_filtered;
std::vector<ros::Publisher> cluster_vector;
// extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
//                                 typename pcl::PointCloud<PointType>::Ptr targetCloud,
//                                 unsigned xmin, unsigned xmax,
//                                 unsigned ymin, unsigned ymax);
// template<typename PointType>
// void extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
//                                 typename pcl::PointCloud<PointType>::Ptr targetCloud,
//                                 unsigned xmin, unsigned xmax,
//                                 unsigned ymin, unsigned ymax,
//                                 unsigned imageWidth,
//                                 unsigned imageHeight);
// {
//   copyPointCloud(*sourceCloud, *targetCloud);
  
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

void BB_Callback(const testrobots::Boundingbox::ConstPtr &msg){
  unsigned xmin = msg->xmin;
  unsigned xmax = msg->xmax;
  unsigned ymin = msg->ymin;
  unsigned ymax = msg->ymax;

  pcl::PointCloud<pcl::PointXYZ>::Ptr final_extracted_pclXYZ(new pcl::PointCloud<pcl::PointXYZ>);
  unsigned imageWidth = downsampled_pclXYZ->width;
  unsigned imageHeight = downsampled_pclXYZ->height;

  copyPointCloud(*downsampled_pclXYZ, *final_extracted_pclXYZ);

  double nan = std::nan("");
  PointType nanPt(nan, nan, nan);
  for(unsigned row =0; row < imageWidth; ++row) { //imageHeight
    for(unsigned col =0; col < imageHeight; ++col) { //imageWidth
      unsigned index = row * imageWidth  + col;
      if((col < xmin) || (xmax < col) || (row < ymin) || (ymax < row))  {
         std::cout << "here - " << index <<final_extracted_pclXYZ->operator[](index)<< std::endl;
         final_extracted_pclXYZ->operator[](index) = nanPt;
      }
    }
  }



}
{

}
void sampling_PointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)

{
   
     
    ROS_INFO_STREAM("In callback function");
    auto start8 = high_resolution_clock::now();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

     
    pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud, *inputCloud);
    pcl::PCLPointCloud2::Ptr downsampled_pcl2 (new pcl::PCLPointCloud2); //Container: down sampled pointcloud2


  

    //filter setup and run
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    
    auto start1 = high_resolution_clock::now();
    vg.setInputCloud (inputCloud);
    vg.setLeafSize (0.02, 0.00, 0.02);
    vg.filter (*downsampled_pcl2);
    auto stop1 = high_resolution_clock::now();
    auto duration1 = duration_cast<microseconds>(stop1 - start1);
    cout << YELLOW << "filtering time:"<< duration1.count()/1000000.0<< "  s\n"<< endl;
    ROS_INFO_STREAM("downsampled data size: " << downsampled_pcl2->data.size()<<"\n");
    ROS_INFO_STREAM("downsampled data width: " << downsampled_pcl2->width<<"\n");
    ROS_INFO_STREAM("downsampled data height: " << downsampled_pcl2->height<<"\n");
     

    //publish: voxel grid filtering result
    sensor_msgs::PointCloud2 downsampled_ros;
    pcl_conversions::fromPCL(*downsampled_pcl2, downsampled_ros);
    voxel_filtered.publish (downsampled_ros); 
    


    //converstion from pcl pointcloud2 to pcl::PointCloud<pcl::PointXYZ>
    //pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pclXYZ(new pcl::PointCloud<pcl::PointXYZ>);//made it global variable
    pcl::fromPCLPointCloud2(*downsampled_pcl2, *downsampled_pclXYZ);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_extracted_pclXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    //extractFrame(downsampled_pclXYZ,final_extracted_pclXYZ )
   


}

extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
                                typename pcl::PointCloud<PointType>::Ptr targetCloud,
                                unsigned xmin, unsigned xmax,
                                unsigned ymin, unsigned ymax)
{
  //Implement in terms of the other function
  unsigned imageWidth = sourceCloud->width;
  unsigned imageHeight = sourceCloud->height;
  extractFrame<PointType>(sourceCloud, targetCloud, xmin, xmax, ymin, ymax, imageWidth, imageHeight);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//main function

int main(int argc, char **argv)
{
    ros::init (argc, argv, "plotting_human");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe(PCL_TOPIC, 10, sampling_PointCloud);
    ros::Subscriber bbsub = n.subscribe("/BBox", 10, BB_Callback);
    voxel_filtered = nh.advertise<sensor_msgs::PointCloud2> ("voxel_filtered", 1);
    // plane_segmented = nh.advertise<sensor_msgs::PointCloud2> ("plane_segmented", 1);
    // except_plane = nh.advertise<sensor_msgs::PointCloud2> ("except_plane",1);
    

    ros::spin();

}