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
#include <pcl/filters/crop_box.h>
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
#include <chrono>

using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace sensor_msgs;
using namespace Eigen;

pcl::PCDWriter m_writer;
ros::Publisher cropbox_filtered;
using namespace std::chrono;
static const std::string PCL_TOPIC = "/camera/depth/points";
#define YELLOW  "\033[33m"      /* Yellow */
#define GREEN   "\033[32m"      /* Green */
#define MAGENTA "\033[35m"      /* Magenta */

float xmin = 664.0;
float xmax = 894.0;
float ymin = 563.0;
float ymax = 663.0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void bb_callback(const testrobots::Boundingbox::ConstPtr &msg){
//     xmin = msg->xmin;
//     xmax = msg->xmax;
//     ymin = msg->ymin;
//     ymax = msg->ymax;

// }
void crop_box(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    // std::cout<< "Loading pcd data..."<<std::endl;
    // pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/apramani/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk 
    // std::cout << "Loaded "
    //         << cloud->width * cloud->height
    //         << " data points from PointCloud.pcd with the following fields: "
    //         << std::endl;
    



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

     
    // // pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
    // // pcl::toPCLPointCloud2(*cloud, *inputCloud);

    CropBox<PointXYZ> cropBoxFilter (true);
    cropBoxFilter.setInputCloud (cloud);
    Eigen::Vector4f min_pt (xmin, -1.0f, 1.0f, 1.0f);
    Eigen::Vector4f max_pt (xmax,1.0f,2.0f, 1.0f);

    // Cropbox slighlty bigger then bounding box of points
    cropBoxFilter.setMin (min_pt);
    cropBoxFilter.setMax (max_pt);

    // Indices
    vector<int> indices;
    cropBoxFilter.filter (indices);

    // Cloud
    //PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>); 
    PointCloud<PointXYZ> cloud_out;
    PointCloud<pcl::PointXYZ>::Ptr cloud_save (new pcl::PointCloud<pcl::PointXYZ>);
    //*cloud_save = &cloud_out;
    cropBoxFilter.filter (cloud_out);
    

    PCLPointCloud2 output_cloud_pcl2;
    sensor_msgs::PointCloud2 cloud_ros;
    pcl::toPCLPointCloud2(cloud_out, output_cloud_pcl2);
    pcl_conversions::fromPCL(output_cloud_pcl2, cloud_ros);
    cropbox_filtered.publish(cloud_ros);

    // std::stringstream vv;
    // vv << "crop_box"<<".pcd";
    // m_writer.write<pcl::PointXYZ>(vv.str(), *cloud_save, false);

 
}
int main(int argc, char **argv)
{
    ros::init (argc, argv, "plotting_human");
    ros::NodeHandle nh;
    // crop_box();
    // cout<<"done"<<endl;
    
    ros::Subscriber sub = nh.subscribe(PCL_TOPIC, 10, crop_box);
    cropbox_filtered = nh.advertise<sensor_msgs::PointCloud2> ("cropbox_filtered", 1);

    // voxel_filtered = nh.advertise<sensor_msgs::PointCloud2> ("voxel_filtered", 1);
    // plane_segmented = nh.advertise<sensor_msgs::PointCloud2> ("plane_segmented", 1);
    // except_plane = nh.advertise<sensor_msgs::PointCloud2> ("except_plane",1);
    

    ros::spin();

}


