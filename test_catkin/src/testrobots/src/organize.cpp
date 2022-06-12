//imports//////////////////////////////////////////////////////////////////////////////////////////////////////////

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

//calculate time
#include <chrono>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/point_types.h>
using namespace std::chrono;
static const std::string PCL_TOPIC = "/camera/depth/points";


ros::Publisher organizer;
pcl::PCDWriter m_writer;
double xmin = 0;
double xmax = 0;
double ymin = 0;
double ymax = 0;
int row = 0;
int col = 0;
// void crop_bb(pcl::PCLPointCloud2::Ptr  input_cloud_ptr, pcl::PCLPointCloud2::Ptr output_cloud_ptr,
// double x_min, double x_max,double  y_min,double  y_max,double  z_min,double z_max);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BBoxCallback (const testrobots::Boundingbox::ConstPtr &msg){
    xmin = msg->xmin;
    xmax = msg->xmax;
    ymin = msg->ymin;
    ymax = msg->ymax;
}
void blah(const sensor_msgs::PointCloud2 &cloud_msg){
   ROS_INFO_STREAM("In callback function");
   auto start1 = high_resolution_clock::now();
   pcl::PointCloud<pcl::PointXYZ> org_cloud;


   pcl::fromROSMsg(cloud_msg,org_cloud);
   pcl::io::savePCDFileASCII ("organized.pcd", org_cloud);
   std::cerr << "Saved " << org_cloud.size () << " data points to organized.pcd." << std::endl;

   std::cout<<"width = "<< org_cloud.width<<std::endl;
   std::cout<<"height = "<< org_cloud.height<<std::endl;

   //read the file
   pcl::PCDReader reader;
   pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
   pcl::PCLPointCloud2::Ptr outputCloud (new pcl::PCLPointCloud2);
   reader.read ("organized.pcd", *inputCloud);

//    //directly without saving
//    pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
//    pcl::PCLPointCloud2::Ptr outputCloud (new pcl::PCLPointCloud2);
//    pcl::fromPCLPointCloud2 (cloud_msg, *inputCloud);
  
   //filtering
   pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
   vg.setInputCloud(inputCloud);
   vg.setLeafSize(0.05,0.05,0.0);
   vg.filter(*outputCloud);
   std::cerr << "PointCloud after filtering: " << outputCloud->width * outputCloud->height 
      << " data points (" << pcl::getFieldsList (*outputCloud) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write ("filtered.pcd", *outputCloud, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    //crop bounding box
   //pcl::PCLPointCloud2::Ptr crop_cloud;
   //crop_bb(outputCloud,crop_cloud,xmax,xmin,ymax,ymin, 0,0);

   auto stop1 = high_resolution_clock::now();
   auto duration1 = duration_cast<microseconds>(stop1 - start1);
   std::cout << "total time: "<< duration1.count()/1000000.0 << " s" << std::endl;
    


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// void crop_bb(pcl::PCLPointCloud2::Ptr input_cloud_ptr, pcl::PCLPointCloud2::Ptr output_cloud_ptr,
// double x_min, double x_max,double  y_min,double  y_max,double  z_min,double z_max){
 
//    boost::shared_ptr<pcl::PCLPointCloud2<pcl::PointXYZ>> bb_ptr (new pcl::PCLPointCloud2<pcl::PointXYZ>);

//     pcl::PointXYZ point;

//     point.x = xmin;
//     point.y = y_min;
//     point.z = z_min;

//     bb_ptr->push_back(point);

//     point.x = x_min;
//     point.y = y_min;
//     point.z = z_max;

//     bb_ptr->push_back(point);

//     //x_min,y_max,z_min
//     point.x = x_min;
//     point.y = y_max;
//     point.z = z_min;

//     bb_ptr->push_back(point);

//     //x_min,y_max,z_max
//     point.x = x_min;
//     point.y = y_min;
//     point.z = z_min;

//     bb_ptr->push_back(point);

//     //x_max,y_min,z_min
//     point.x = x_max;
//     point.y = y_min;
//     point.z = z_min;

//     bb_ptr->push_back(point);

//     //x_max,y_min,z_max
//     point.x = x_max;
//     point.y = y_min;
//     point.z = z_max;

//     bb_ptr->push_back(point);

//     //x_max,y_max,z_min
//     point.x = x_max;
//     point.y = y_max;
//     point.z = z_min;

//     bb_ptr->push_back(point);

//     //x_max,y_max,z_max
//     point.x = x_max;
//     point.y = y_max;
//     point.z = z_max;

//     bb_ptr->push_back(point);

//     pcl::ConvexHull<pcl::PointXYZ> hull;
//     std::vector<pcl::Vertices> polygons;

//     boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > surface_hull;

//     hull.setInputCloud(bb_ptr);
//     hull.setDimension(3);

//     surface_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
//     hull.reconstruct(*surface_hull, polygons);
//     pcl::CropHull<pcl::PointXYZ> bb_filter;

//    bb_filter.setDim(3);
//    bb_filter.setNegative(false);
//    bb_filter.setInputCloud(input_cloud_ptr);
//    bb_filter.setHullIndices(polygons);
//    bb_filter.setHullCloud(surface_hull);
//    bb_filter.filter(*output_cloud_ptr.get());




// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//main function

int main(int argc, char **argv)
{
   ros::init (argc, argv, "plotting_human");
   ros::NodeHandle nh;
   ros::Subscriber BBsub = nh.subscribe("/BBox", 10, BBoxCallback);
   ros::Subscriber sub = nh.subscribe(PCL_TOPIC, 10, blah);
   

  ros::spin();
  return 0;

}
