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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


ros::Publisher voxel_filtered;

void sampling_PointCloud()//const sensor_msgs::PointCloud2ConstPtr& cloud_msg
{
   
     //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
     std::cout<< "1"<<std::endl;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
     std::cout<< "2"<<std::endl;
     pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/apramani/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk 
     std::cout<< "3"<<std::endl;

     std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

   
     
     pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
     pcl::toPCLPointCloud2(*cloud, *inputCloud);
     pcl::PCLPointCloud2::Ptr downsampled_pcl2 (new pcl::PCLPointCloud2); //Container: down sampled pointcloud2

     //filter setup and run
     pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
     vg.setInputCloud (inputCloud);
     vg.setLeafSize (0.05, 0.05, 0.0);
     vg.filter (*downsampled_pcl2);
     ROS_INFO_STREAM("downsampled data size: " << downsampled_pcl2->data.size());

     //publish: voxel grid filtering result
     sensor_msgs::PointCloud2 downsampled_ros;
     pcl_conversions::fromPCL(*downsampled_pcl2, downsampled_ros);
     std::stringstream ss;
     ss << "downsampled_pcd"<<".pcd";
    
    // voxel_filtered.publish (downsampled_ros);

     //converstion from pcl pointcloud2 to pcl::PointCloud<pcl::PointXYZ>
     pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(*downsampled_pcl2, *downsampled_pclXYZ);
     m_writer.write<pcl::PointXYZ>(ss.str(), *downsampled_pclXYZ, false);



}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//main function

int main(int argc, char **argv)
{
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    
    sampling_PointCloud ();
    //voxel_filtered = nh.advertise<sensor_msgs::PointCloud2> ("voxel_filtered", 1);
    std::cout<< "here "<<std::endl;

//   ros::init(argc, argv, "listener");

//   ros::NodeHandle n;

 // ros::Subscriber sub = n.subscribe(PCL_TOPIC, 10, cloud_cb);

  //ros::Subscriber BBsub = n.subscribe("/BBox", 10, BBoxCallback);

  // ros::Subscriber current_pose = n.subscribe("/amcl_pose", 10, poseCallback);

  ros::spin();

}