//imports//////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <testrobots/Boundingbox.h>
#include <testrobots/newBoundingbox.h>
#include <tf/transform_listener.h>
#include <pclUtils.h>
#include <convexHull.h>
#include <map_writer.h>
// #include <buffer.h>
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
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
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
using namespace std;

//variable declarations:
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
double C_z;
double D_x;
double D_y;
double D_z;

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

int counter = 0;
pcl::PCDReader reader; 
pcl::PCDWriter writer;
ros::Publisher tf_pub;
pcl::PCDWriter m_writer;
ros::Publisher organizer;
sensor_msgs::PointCloud2 obj_msg;
sensor_msgs::PointCloud2 proj_msg;
ros::Publisher pub_cropped_cloud;
ros::Publisher pub_extracted_cloud;
ros::Publisher pub_projected_cloud;
tf::TransformListener *tf_listener; 
ros::Publisher passthrough_filtered;
pcl::PCLPointCloud2 passfiltered_pcl2;
sensor_msgs::PointCloud2 crop_cloud_msg;
sensor_msgs::PointCloud2 passfiltered_ros;



pcl::PointCloud<pcl::PointXYZ> extract;
pcl::PointCloud<pcl::PointXYZ> project;
pcl::PointCloud<pcl::PointXYZ> org_cloud;
pcl::PointCloud<pcl::PointXYZ> transformed;
pcl::PointCloud<pcl::PointXYZ> pass_filtered_cloud;
static const std::string PCL_TOPIC = "/camera/depth/points";
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > surface_hull;
pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr outputCloud (new pcl::PCLPointCloud2());
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr no_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > bb_ptr (new pcl::PointCloud<pcl::PointXYZ>);


//function declarations:

void crop_bb(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >  input_cloud_ptr, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > output_cloud_ptr,
double x_min, double x_max,double  y_min,double  y_max,double  z_min,double z_max);
void extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud_ptr);


//function definitions:

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void save_pcd(sensor_msgs::PointCloud2 ros_msg, int counter,string file_name ){
   pcl::PointCloud<pcl::PointXYZ> save_cloud;
   pcl::fromROSMsg(ros_msg,save_cloud);
   pcl::io::savePCDFileASCII (file_name + std::to_string(counter)+".pcd", save_cloud);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void blah(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) { 
   
   //start timer
   auto start1 = high_resolution_clock::now();
   counter++;

   
   pcl_conversions::toPCL(*cloud_msg, *inputCloud);

   //do voxel filtering and save to pcd
   pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
   vg.setInputCloud(inputCloud);
   vg.setLeafSize(0.07,0.0,0.07);
   vg.filter(*outputCloud);
   std::cerr << "PointCloud after filtering: " << outputCloud->width * outputCloud->height << " data points (" << pcl::getFieldsList (*outputCloud) << ").\n" << std::endl;    
   pcl::fromPCLPointCloud2(*outputCloud, *output_ptr);
   


   // //call crop bounding boxfunction and convert to ros msg  
   std::cout<<"cropping bounding box...\n"<< std::endl; 
   crop_bb(no_plane_cloud,cropped_cloud_ptr,xmin,xmax,ymin,ymax,zmin,zmax);   
   pcl::toROSMsg(*cropped_cloud_ptr.get(),crop_cloud_msg );


   // //save cropped cloud to pcd and publish   
   // //save_pcd(crop_cloud_msg,counter, "cropped");
   pub_cropped_cloud.publish (crop_cloud_msg);

   
   

   //call extract function and convert to ros msg and publish 
   std::cout<<"extracting object...\n"<< std::endl; 
   extractObject(output_ptr);//output_ptr//cropped_cloud_ptr
   pcl::toROSMsg(*no_plane_cloud.get(),obj_msg );
   pub_extracted_cloud.publish(obj_msg);


   //save extracted cloud to pcd   
   // save_pcd(obj_msg,counter, "extracted");

 

   // //project points on XZ plane:
   coefficients->values.resize (4);
   coefficients->values[0] = 0;
   coefficients->values[1] = 1;
   coefficients->values[2] = 0.0;
   coefficients->values[3] = 0;
   pcl::ProjectInliers<pcl::PointXYZ> proj;
   proj.setModelType (pcl::SACMODEL_PLANE);
   proj.setInputCloud (no_plane_cloud);
   proj.setModelCoefficients (coefficients);
   proj.filter (*cloud_projected);

   //print projected cloud:
   // std::cerr << "Cloud after projection: " << std::endl;
   // for (const auto& point: *cloud_projected)
   //  std::cerr << "    " << point.x << " "
   //                      << point.y << " "
   //                      << point.z << std::endl;

   pcl::toROSMsg(*cloud_projected.get(),proj_msg);
   pub_projected_cloud.publish(proj_msg);

   // save in pcd
   // save_pcd(proj_msg,counter, "projected");
  

   //calculate computation time
   auto stop1 = high_resolution_clock::now();
   auto duration1 = duration_cast<microseconds>(stop1 - start1);
   std::cout << "total time: "<< duration1.count()/1000000.0 << " s\n" << std::endl;
   std::cout << "**************************\n"<<std::endl;
    


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud_ptr)
{
    no_plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);    
    pcl::PointIndices::Ptr planar_inliers (new pcl::PointIndices);// Cloud indices representing planar components inliers   
    pcl::ModelCoefficients::Ptr planar_coefficients (new pcl::ModelCoefficients); // Cloud coefficients for planar components inliers
    pcl::SACSegmentation<pcl::PointXYZ> SAC_filter;// Segmentation object
    pcl::ExtractIndices<pcl::PointXYZ> planar_inliers_extraction;// Euclidean Cluster Extraction object


    // Segmentation object initialization
    SAC_filter.setOptimizeCoefficients (true);
    SAC_filter.setModelType(pcl::SACMODEL_PLANE);
    SAC_filter.setMethodType (pcl::SAC_RANSAC);
    SAC_filter.setMaxIterations (200);
    SAC_filter.setDistanceThreshold (0.001);
   

    // Segment the dominant plane cluster
    SAC_filter.setInputCloud (crop_cloud_ptr);
    SAC_filter.segment (*planar_inliers, *planar_coefficients);

    if (planar_inliers->indices.size () == 0)
    {
       return ;
    }

    // Remove the planar cluster from the input cloud
    planar_inliers_extraction.setInputCloud (crop_cloud_ptr);
    planar_inliers_extraction.setIndices (planar_inliers);
    planar_inliers_extraction.setNegative (true);
    planar_inliers_extraction.filter (*no_plane_cloud);
    std::vector<int> no_Nan_vector;
    pcl::removeNaNFromPointCloud(*no_plane_cloud,*no_plane_cloud,no_Nan_vector);

    //euclidian cluster extraction
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (no_plane_cloud);
 
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (no_plane_cloud);
    ec.extract (cluster_indices);
    std::cout<<"blahhhh"<<std::endl;
 
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
   {
      std::cout<<"blahhhh 2 "<<std::endl;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& idx : it->indices)
        cloud_cluster->push_back ((*no_plane_cloud)[idx]);
        std::cout<<"blahhhh 3 "<<std::endl; //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
 
     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
     std::stringstream ss;
     ss << "cloud_cluster_" << j << ".pcd";
     writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
     j++;
   }

   
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void crop_bb(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > input_cloud_ptr, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > output_cloud_ptr,
double x_min, double x_max,double  y_min,double  y_max,double  z_min,double z_max){
 
   

   pcl::PointXYZ point;

   point.x = xmin;
   point.y = y_min;
   point.z = z_min;

   bb_ptr->push_back(point);

   point.x = x_min;
   point.y = y_min;
   point.z = z_max;

   bb_ptr->push_back(point);

   //x_min,y_max,z_min
   point.x = x_min;
   point.y = y_max;
   point.z = z_min;

   bb_ptr->push_back(point);

   //x_min,y_max,z_max
   point.x = x_min;
   point.y = y_min;
   point.z = z_min;

   bb_ptr->push_back(point);

   //x_max,y_min,z_min
   point.x = x_max;
   point.y = y_min;
   point.z = z_min;

   bb_ptr->push_back(point);

   //x_max,y_min,z_max
   point.x = x_max;
   point.y = y_min;
   point.z = z_max;

   bb_ptr->push_back(point);

    //x_max,y_max,z_min
   point.x = x_max;
   point.y = y_max;
   point.z = z_min;

   bb_ptr->push_back(point);

   //x_max,y_max,z_max
   point.x = x_max;
   point.y = y_max;
   point.z = z_max;

   bb_ptr->push_back(point);

   

   pcl::ConvexHull<pcl::PointXYZ> hull;
   std::vector<pcl::Vertices> polygons;

    

   hull.setInputCloud(bb_ptr);
   hull.setDimension(3);

   

   surface_hull.reset(new pcl::PointCloud<pcl::PointXYZ>);
   hull.reconstruct(*surface_hull, polygons);
    
   pcl::CropHull<pcl::PointXYZ> bb_filter;

   

   bb_filter.setDim(3);
   
   bb_filter.setNegative(false);
   
   bb_filter.setInputCloud(input_cloud_ptr);
   
   bb_filter.setHullIndices(polygons);
   
   bb_filter.setHullCloud(surface_hull);
   
   bb_filter.filter(*output_cloud_ptr.get());
   
   




}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//main function

int main(int argc, char **argv)
{
   ros::init (argc, argv, "plotting_human");
   ros::NodeHandle nh;


   //subscribe
   ros::Subscriber BBsub = nh.subscribe("/Box_values", 1, BBoxCallback); //Avhishek - changed from 10 to 1
   ros::Subscriber PCLsub = nh.subscribe(PCL_TOPIC, 10, blah);

   //set frame
   std::string frame_id="camera_rgb_optical_frame";
   crop_cloud_msg.header.frame_id=frame_id;
   obj_msg.header.frame_id = frame_id;
   proj_msg.header.frame_id = frame_id;
   
   //publish
   pub_cropped_cloud=nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud",1);
   pub_extracted_cloud=nh.advertise<sensor_msgs::PointCloud2>("extracted_cloud",1);
   pub_projected_cloud=nh.advertise<sensor_msgs::PointCloud2>("projected",1);
   

   ros::spin();
   return 0;

}
