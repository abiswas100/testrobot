//include dirs

#include <ros/ros.h>
#include <testrobots/Boundingbox.h>
#include <testrobots/newBoundingbox.h>
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
static const std::string PCL_TOPIC = "/camera/depth/points"; // cropped_PCL
#define YELLOW  "\033[33m"      /* Yellow */
#define GREEN   "\033[32m"      /* Green */
#define MAGENTA "\033[35m"      /* Magenta */


double xmin = 0;
double xmax = 0;
double ymin = 0;
double ymax = 0;
double zmin = 0;
double zmax = 0;

double A_x = 0;
double A_y = 0;
double A_z = 0;
double B_x = 0;
double B_y = 0;
double B_z = 0;
double C_x = 0;
double C_y = 0;
double C_z = 0;
double D_x = 0;
double D_y = 0;
double D_z = 0;

double xmin_left = 0;
double xmin_right = 0;

double xmax_left = 0;
double xmax_right = 0;

double ymin_left = 0;
double ymin_right = 0;

double ymax_left = 0;
double ymax_right = 0;

double zmin_left = 0;
double zmin_right = 0;

double zmax_left = 0;
double zmax_right = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void bb_callback(const testrobots::Boundingbox::ConstPtr &msg){
//     xmin = msg->xmin;
//     xmax = msg->xmax;
//     ymin = msg->ymin;
//     ymax = msg->ymax;

// }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BBoxCallback (const testrobots::newBoundingbox::ConstPtr &msg){

   //getting coordinated from msg
   A_x = msg->A_x;
   A_y = msg->A_y;
   A_z = msg->A_z;

   B_x = msg->B_x;
   B_y = msg->B_y;
   B_z = msg->B_z;

   C_x = msg->C_x;
   C_y = msg->C_y;
   C_z = msg->C_z;

   D_x = msg->D_x;
   D_y = msg->D_y;
   D_z = msg->D_z;
   
  
   //calculating max and min for each axis
   xmin_left = std::min(A_x, B_x);
   xmin_right = std::min(C_x, D_x);
   xmin = std::min(xmin_left,xmin_right);

   xmax_left = std::max(A_x, B_x);
   xmax_right= std::max(C_x, D_x);
   xmax = std::max(xmax_left,xmax_right);

   ymin_left = std::min(A_y, B_y);
   ymin_right = std::min(C_y,D_y);
   ymin = std::min(ymin_left,ymin_right);

   ymax_left = std::max(A_y, B_y);
   ymax_right = std::max(C_y,D_y);
   ymax = std::max(ymax_left,ymax_right);

   zmin_left = std::min(A_z, B_z);
   zmin_right = std::min(C_z, D_z);
   zmin = std::min(zmin_left, zmin_right);

   zmax_left = std::max(A_z, B_z);
   zmax_right = std::max(C_z, D_z);
   zmax = std::max(zmax_left, zmax_right);

   

  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    std::cout << "width:  "
            << cloud->width << "height: "<<cloud->height<< std::endl;

     
    // // pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
    // // pcl::toPCLPointCloud2(*cloud, *inputCloud);

    CropBox<PointXYZ> cropBoxFilter (true);
    cropBoxFilter.setInputCloud (cloud);
    Eigen::Vector4f min_pt (xmin,ymin,zmin,  1.0f);
    Eigen::Vector4f max_pt (xmax,ymax,zmax, 1.0f); //ymax -0.05
    std::cout<<"min pt vector"<<min_pt<<std::endl;
    std::cout<<"max pt vector"<<max_pt<<std::endl;

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
    ros::Subscriber BBsub = nh.subscribe("/Box_values", 10, BBoxCallback);
    ros::Subscriber sub = nh.subscribe(PCL_TOPIC, 10, crop_box);
    cropbox_filtered = nh.advertise<sensor_msgs::PointCloud2> ("cropbox_filtered", 1);

    // voxel_filtered = nh.advertise<sensor_msgs::PointCloud2> ("voxel_filtered", 1);
    // plane_segmented = nh.advertise<sensor_msgs::PointCloud2> ("plane_segmented", 1);
    // except_plane = nh.advertise<sensor_msgs::PointCloud2> ("except_plane",1);
    

    ros::spin();

}


