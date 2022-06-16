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



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ros::Publisher plane_segmented;
ros::Publisher except_plane;
ros::Publisher voxel_filtered;
std::vector<ros::Publisher> cluster_vector;

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
    ROS_INFO_STREAM("original data size: " << inputCloud->data.size()<<"\n");
    ROS_INFO_STREAM("original data width: " << inputCloud->width<<"\n");
    ROS_INFO_STREAM("original data height: " << inputCloud->height<<"\n"); 
    //filter setup and run
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    
    auto start1 = high_resolution_clock::now();
    vg.setInputCloud (inputCloud);
    vg.setLeafSize (0.02, 0.02, 0.0);
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pclXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*downsampled_pcl2, *downsampled_pclXYZ);
    
   

    //container: plane pcl pointXYZ
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


    //segmentation :
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    auto begin2 = high_resolution_clock::now();
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(downsampled_pclXYZ);
    seg.segment(*inliers, *coefficients);
    auto stop2 = high_resolution_clock::now();
    auto time2 = duration_cast<microseconds>(stop2 - begin2);
    cout << GREEN << "segmentation time: "<< time2.count()/1000000.0 << " s\n" << endl;
    

    if(inliers->indices.size () == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

  //extraction:
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(downsampled_pclXYZ);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*plane_pclXYZ);
  

  //publish
  pcl::PCLPointCloud2 plane_pcl2;
  sensor_msgs::PointCloud2 plane_ros;
  pcl::toPCLPointCloud2(*plane_pclXYZ, plane_pcl2);
  pcl_conversions::fromPCL(plane_pcl2, plane_ros);
  plane_segmented.publish (plane_ros);

  pcl::PointCloud<pcl::PointXYZ>::Ptr except_plane_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 except_plane_pcl2;
  sensor_msgs::PointCloud2 except_plane_ros;

  extract.setInputCloud (downsampled_pclXYZ);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*except_plane_pclXYZ);

  pcl::toPCLPointCloud2(*except_plane_pclXYZ, except_plane_pcl2);
  pcl_conversions::fromPCL(except_plane_pcl2, except_plane_ros);
  except_plane.publish (except_plane_ros);

  std::cout<<"width "<<except_plane_pclXYZ ->width<<std::endl;
  std::cout<<"height "<<except_plane_pclXYZ ->height<<std::endl;

 
  
  
  


    


  


}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//main function

int main(int argc, char **argv)
{
    ros::init (argc, argv, "plotting_human");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe(PCL_TOPIC, 10, sampling_PointCloud);
    voxel_filtered = nh.advertise<sensor_msgs::PointCloud2> ("voxel_filtered", 1);
    plane_segmented = nh.advertise<sensor_msgs::PointCloud2> ("plane_segmented", 1);
    except_plane = nh.advertise<sensor_msgs::PointCloud2> ("except_plane",1);
    

    ros::spin();

}