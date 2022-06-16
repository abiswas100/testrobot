
#include <ros/ros.h>
#include <testrobots/Boundingbox.h>
#include <pclUtils.h>
#include <convexHull.h>
#include <map_writer.h>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stack>

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

static const std::string PCL_TOPIC = "/camera/depth/points";



pcl::PCDWriter m_writer;
std::ofstream m_out;

// Camera Specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(1920);
const unsigned CAMERA_NUM_PIXELS_HEIGHT(1080);
const double CAMERA_HORIZONTAL_VIEW_ANGLE(1.19);

// bounding box attributes
unsigned xmin = 664;
unsigned xmax = 894;
unsigned ymin = 563;
unsigned ymax = 663;
unsigned imagewidth = 1920;


///////////////////////////////////////////// Variable Description ///////////////////////////////////////////////////////////
void sampleandextract(); // this will be the function that does plane extract and recieves a pointer


void sampleandextract()
{
  ROS_INFO_STREAM("getting here in sampleandextract");

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
     
     pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/apramani/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk 
     
     std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from PointCloud.pcd with the following fields: "
            << std::endl;
    
    pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud, *inputCloud);
    pcl::PCLPointCloud2::Ptr downsampled_pcl2 (new pcl::PCLPointCloud2); //Container: down sampled pointcloud2


    // define the sampling 
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    std::cout<< "Filtering point cloud .."<<std::endl;
    
    vg.setInputCloud (inputCloud);
    vg.setLeafSize (0.02, 0.02, 0.0);
    vg.filter (*downsampled_pcl2);
    ROS_INFO_STREAM("downsampled data size: " << downsampled_pcl2->data.size());
    
    // now convert PCL2 to PointXYZ for extraction
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*downsampled_pcl2, *downsampled_pclXYZ);
    std::stringstream ss;
    ss << "voxel.pcd";
    m_writer.write<pcl::PointXYZ>(ss.str(), *downsampled_pclXYZ, false);
  ////////////////////////////////////////////////////////// This code is to run extraction//////////////////////////
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrop(new pcl::PointCloud<pcl::PointXYZ>);
  testrobots::extractFrame<pcl::PointXYZ>(downsampled_pclXYZ, cloudCrop,
                                          imagewidth,
                                          xmin,
                                          xmax,
                                          ymin,
                                          ymax);

  std::cout << "getting here after extract frame " << std::endl;
  // this will be used to save the extracted pointcloud as a pcd file
  
  // ss << "extractBBcrop.pcd";
  // m_writer.write<pcl::PointXYZ>(ss.str(), *cloudCrop, false);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // call function here

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/apramani/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk
  // pcl::io::loadPCDFile<pcl::PointXYZ>("/home/tran/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); // loads the PointCloud data from disk

  sampleandextract();
  // extractObjectInBoundingBox(cropPercentage);
}