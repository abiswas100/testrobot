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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


ros::Publisher voxel_filtered;
std::vector<ros::Publisher> cluster_vector;

void sampling_PointCloud()//const sensor_msgs::PointCloud2ConstPtr& cloud_msg
{
   
     //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
     
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
     std::cout<< "Loading pcd data ..."<<std::endl;
     auto start4 = high_resolution_clock::now();
     pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/apramani/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk 
    //  pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/tran/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk 
     auto stop4 = high_resolution_clock::now();
     auto duration4 = duration_cast<microseconds>(stop4 - start4);
     cout << "loading time: "<< duration4.count()/1000000.0 << " s" << endl;
     std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from PointCloud.pcd "
            << std::endl;

   
     
     pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
     pcl::toPCLPointCloud2(*cloud, *inputCloud);


  //clustering:
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  auto start5 = high_resolution_clock::now();
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (20000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  std::cout<< "clustering ongoin 1!"<<std::endl;

  std::cout << cluster_indices.size() << " clusters" << std::endl;

  ros::NodeHandle cluster_nh;

  for (int i = 0; i < cluster_indices.size(); i++)
  {
    std::string topicName = "cluster" + boost::lexical_cast<std::string>(i);
    ros::Publisher pub = cluster_nh.advertise<sensor_msgs::PointCloud2> (topicName, 1);
    cluster_vector.push_back(pub);
  }

  for (int i = 0; i < cluster_indices.size(); i++)
  {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cluster_pcl2;
    sensor_msgs::PointCloud2 cluster_ros;

    for (int j = 0; j < cluster_indices[i].indices.size(); j++)
    {
      int index = cluster_indices[i].indices[j];
      cluster_pclXYZ->points.push_back(cloud->points[index]);
    }
    std::cout<< "clustering ongoing 2!"<<std::endl;
    std::cout << i << " PointCloud representing the Cluster: " << cluster_pclXYZ->points.size () << " data " << std::endl;
    cluster_pclXYZ->width = cluster_pclXYZ->points.size();
    cluster_pclXYZ->height = 1;
    cluster_pclXYZ->is_dense = true;
    pcl::toPCLPointCloud2( *cluster_pclXYZ ,cluster_pcl2);
    pcl_conversions::fromPCL(cluster_pcl2, cluster_ros);
    auto stop5 = high_resolution_clock::now();
    auto duration5 = duration_cast<microseconds>(stop5 - start5);
    cout << "clustering time: "<< duration5.count()/1000000.0 << " s" << endl;
    //cluster_ros.header.frame_id = downsampled_pclXYZ->header.frame_id;
    //std::cout << "Cluster Frame id: " << downsampled_pclXYZ->header.frame_id << std::endl;
    //cluster_vector[i].publish (cluster_ros);
    std::cout<< "clustering done!"<<std::endl;
    std ::stringstream bb;
    bb << "clustered_voxel"<<".pcd";
    m_writer.write<pcl::PointXYZ>(bb.str(), *cluster_pclXYZ, false);

  }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//main function

int main(int argc, char **argv)
{
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    auto start = high_resolution_clock::now(); //start timer
    //call function here
    sampling_PointCloud ();
    //stop timer 
    auto stop = high_resolution_clock::now();
    //calculate computation time
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "Total time: "<< duration.count()/1000000.0 << " s" << endl;
    //ros::spin();

}