#include <ros/ros.h>
#include <testrobots/Boundingbox.h>
#include <string>

// ROS Topics
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h>  

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
// std
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <iostream>

static const std::string PCL_TOPIC = "/camera/depth/points";
const double normalThreshold = 0.97;

enum class Normal
{
  eX = 0,
  eY,
  eZ
};
const Normal normal = Normal::eY;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 
    // pcl::PCLPointCloud2* m_cloud = new pcl::PCLPointCloud2;
    // pcl::PCLPointCloud2ConstPtr cloudPtr(m_cloud);
    pcl::PCLPointCloud2Ptr p_cloud(new pcl::PointCloud2());
    pcl::PCLPointCloud2 m_cloud;
    pcl_conversions::toPCL(*cloud_msg,m_cloud);

    *p_cloud = *cloud_msg;


    ROS_INFO_STREAM("getting here 1");
    pcl::PCLPointCloud2 final_planeless_cloud;
    pcl::PCLPointCloud2 cloud_for_plane_extraction;
    copyPointCloud(m_cloud, final_planeless_cloud);
    ROS_INFO_STREAM("getting here 2");
    copyPointCloud(m_cloud, cloud_for_plane_extraction);
    ROS_INFO_STREAM("getting here 3");
    
    pcl::PCLPointCloud2 cloud_plane;
    pcl::PCLPointCloud2 cloud_f;
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

        // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    
    ROS_INFO_STREAM("getting here 4");


    // Create the extracting object (extraction based on indices)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
        
    //Keep the removed points as NaN values to maintain the structure of the cloud
    extract.setKeepOrganized(true);
        
    unsigned planeCount =0;
    const unsigned numOriginalPoints = cloud_for_plane_extraction.width;   //How many points we originally started with
    const unsigned numOriginalPoints2 = cloud_for_plane_extraction.height;
    const unsigned numOriginalPoints3 = numOriginalPoints * numOriginalPoints2;
    unsigned numPointsExtracted =0;   //Keep track of how many points have so far been extracted
    std::cout << numOriginalPoints3 << std::endl;    
    ROS_INFO_STREAM("getting here 5");


}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe(PCL_TOPIC, 10, cloud_cb);

  ros::spin();


  return 0;
}

