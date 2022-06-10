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
using namespace std::chrono;

ros::Publisher passthrough_filtered;
pcl::PCDWriter m_writer;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sampling_passthrough(){

    //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
     
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
     std::cout<< "Loading pcd data..."<<std::endl;
     auto start1 = high_resolution_clock::now();
     pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/apramani/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk 
     auto stop1 = high_resolution_clock::now();
     auto duration1 = duration_cast<microseconds>(stop1 - start1);
     cout << "loading time: "<< duration1.count()/1000000.0 << " s" << endl;

     std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from PointCloud.pcd with the following fields: "
            << std::endl;
     
     pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);//container after filtering
     pcl::PassThrough<pcl::PointXYZ> pass_filter;

     //set up passhtrough filter
     std::cout<< "Passthorugh filter running .."<<std::endl;
     auto start2 = high_resolution_clock::now();
     pass_filter.setInputCloud (cloud);
     pass_filter.setFilterLimits (0.0, 1.0);
     pass_filter.setFilterLimitsNegative (true);
     pass_filter.filter (*passfiltered_pclXYZ);
     auto stop2 = high_resolution_clock::now();
     auto duration2 = duration_cast<microseconds>(stop2 - start2);
     cout << "filtering time: "<< duration2.count()/1000000.0 << " s" << endl;

     pcl::PCLPointCloud2::Ptr outputCloud (new pcl::PCLPointCloud2);
     pcl::toPCLPointCloud2(*passfiltered_pclXYZ, *outputCloud);

     ROS_INFO_STREAM("downsampled data size: " << outputCloud->data.size());

     pcl::PCLPointCloud2 passfiltered_pcl2;
     sensor_msgs::PointCloud2 passfiltered_ros;
     pcl::toPCLPointCloud2(*passfiltered_pclXYZ, passfiltered_pcl2);
     pcl_conversions::fromPCL(passfiltered_pcl2, passfiltered_ros);
     
     std::stringstream ss;
    
     ss << "downsampled_pcd_passthrough"<<".pcd";
     m_writer.write<pcl::PointXYZ>(ss.str(), *passfiltered_pclXYZ, false);     
     std::cout<< "Filtering done"<<std::endl;

    // Avhishek - adding Plane segmentation 

    //  container: plane pcl pointXYZ
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //segmentation setup
    std::cout<< "segmentation setup done!"<<std::endl;
    auto start3 = high_resolution_clock::now();
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    auto stop3 = high_resolution_clock::now();
    auto duration3 = duration_cast<microseconds>(stop3 - start3);
    cout << "segmentation time: "<< duration3.count()/1000000.0 << " s" << endl;
    std::cout<< "segmentation done!"<<std::endl;

    if(inliers->indices.size () == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*plane_pclXYZ);
    std::cout<< "extraction done!"<<std::endl;
    std::stringstream vv;
    vv << "planeextracted_voxel"<<".pcd";
    m_writer.write<pcl::PointXYZ>(vv.str(), *plane_pclXYZ, false);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//main function

int main(int argc, char **argv)
{
   
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    auto start = high_resolution_clock::now();
    sampling_passthrough ();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "total time: "<< duration.count()/1000000.0 << " s" << endl;
    

  //ros::spin();
  return 0;

}
