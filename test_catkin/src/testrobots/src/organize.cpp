/////////////////////////////////////////////***imports***////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <testrobots/Boundingbox.h>
#include <testrobots/newBoundingbox.h>
#include <tf/transform_listener.h>
#include <pclUtils.h>
#include <convexHull.h>
#include <map_writer.h>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stack>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

// ROS Topics
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h>  
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>


// pcl
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_painter2D.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/point_types.h>

// std
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

//****************************************************declarations************************************************

//publisher declarations
ros::Publisher tf_pub;
// ros::Publisher mean_pub;
// ros ::Publisher var_pub;
ros::Publisher organizer;
ros::Publisher marker_pub;
ros::Publisher cloud_for_poly;
ros::Publisher rad_ros_pub;
ros::Publisher pub_cropped_cloud;
ros::Publisher pub_extracted_cloud;
ros::Publisher pub_projected_cloud;
ros::Publisher passthrough_filtered;
ros::Publisher passthrough_filtered_again;

//object declarations
pcl::PCDReader reader; 
pcl::PCDWriter writer;
pcl::PCDWriter m_writer;
pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
pcl::PassThrough<pcl::PointXYZ> pass_filter;
pcl::PassThrough<pcl::PointXYZ> pass_filter2;

//msg declarations
sensor_msgs::PointCloud2 rad_ros;
sensor_msgs::PointCloud2 obj_msg;
sensor_msgs::PointCloud2 proj_msg;
visualization_msgs::Marker marker;
sensor_msgs::PointCloud2 crop_cloud_msg;
sensor_msgs::PointCloud2 passfiltered_ros;
sensor_msgs::PointCloud2 passfiltered_ros_again;


// variables and pointers
Eigen::Matrix<float,4,1> mean;
Eigen::Matrix<float, 3,3> cov_matrix;
pcl::PCLPointCloud2 passfiltered_pcl2;
pcl::PointCloud<pcl::PointXYZ> final_cloud;
pcl::PCLPointCloud2 passfiltered_pcl2_again;
std::string frame_id="camera_rgb_optical_frame";
pcl::PointCloud<pcl::PointXYZ> pass_filtered_cloud;
static const std::string PCL_TOPIC = "/camera/depth/points";
pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr outputCloud (new pcl::PCLPointCloud2());
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr no_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_again (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);   

//function declarations:
void extractObject(pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud_ptr);



//////////////////////////////////////*****function definitions**********////////////////////////////////////////////////

void save_pcd(sensor_msgs::PointCloud2 ros_msg, int counter,string file_name ){
   pcl::PointCloud<pcl::PointXYZ> save_cloud;
   pcl::fromROSMsg(ros_msg,save_cloud);
   pcl::io::savePCDFileASCII (file_name + std::to_string(counter)+".pcd", save_cloud);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) { 
   
   //start timer
   auto start1 = high_resolution_clock::now(); 
   pcl_conversions::toPCL(*cloud_msg, *inputCloud);


   //do voxel filtering and save to pcd   

   vg.setInputCloud(inputCloud);
   vg.setLeafSize(0.07,0.0,0.07);
   vg.filter(*outputCloud);
   std::cerr << "PointCloud after filtering: " << outputCloud->width * outputCloud->height << " data points (" << pcl::getFieldsList (*outputCloud) << ").\n" << std::endl;    
   pcl::fromPCLPointCloud2(*outputCloud, *output_ptr);
   


   //passthrough filtering z axis   
   
   pass_filter.setInputCloud (output_ptr);
   pass_filter.setFilterFieldName ("z");
   pass_filter.setFilterLimits (0.3, 3.35);
   pass_filter.setFilterLimitsNegative (false);  
   pass_filter.filter (*passfiltered_pclXYZ);

   
   sensor_msgs::PointCloud2 passfiltered_ros;
   pcl::toPCLPointCloud2(*passfiltered_pclXYZ, passfiltered_pcl2);
   pcl_conversions::fromPCL(passfiltered_pcl2, passfiltered_ros);
   passthrough_filtered.publish(passfiltered_ros);
   

   //passthrough filtering y axis  

   pass_filter2.setInputCloud (passfiltered_pclXYZ);
   pass_filter2.setFilterFieldName ("x");
   pass_filter2.setFilterLimits (-1.58, 0.8);
   pass_filter2.setFilterLimitsNegative (false); 
   pass_filter2.filter (*passfiltered_again);
   
   
   pcl::toPCLPointCloud2(*passfiltered_again, passfiltered_pcl2_again);
   pcl_conversions::fromPCL(passfiltered_pcl2_again, passfiltered_ros_again);
   passthrough_filtered_again.publish(passfiltered_ros_again);
  

   //call extract function and convert to ros msg and publish 

   std::cout<<"extracting object...\n"<< std::endl; 
   extractObject(passfiltered_again);
   pcl::toROSMsg(*no_plane_cloud.get(),obj_msg );
   pub_extracted_cloud.publish(obj_msg);


   //save extracted cloud to pcd   
   //save_pcd(obj_msg,counter, "extracted");


   // project points on XZ plane:
   pcl::ProjectInliers<pcl::PointXYZ> proj;
   coefficients->values.resize (4);
   coefficients->values[0] = 0;
   coefficients->values[1] = 1;
   coefficients->values[2] = 0.0;
   coefficients->values[3] = 0;

   proj.setModelType (pcl::SACMODEL_PLANE);
   proj.setInputCloud (no_plane_cloud);
   proj.setModelCoefficients (coefficients);
   proj.filter (*cloud_projected);
  

   pcl::toROSMsg(*cloud_projected.get(),proj_msg);
   pub_projected_cloud.publish(proj_msg); 
   pcl::fromROSMsg(proj_msg,final_cloud );



   //calculate center and covariance matrix  

   pcl::computeMeanAndCovarianceMatrix(final_cloud,cov_matrix, mean);
   // mean_pub.publish(mean);//msg type??
   // var_pub.publish(cov_matrix);//msg type??


   //for visualization:

   uint32_t shape = visualization_msgs::Marker::SPHERE;
   marker.header.frame_id = frame_id;
   marker.header.stamp = ros::Time::now();
   marker.ns = "basic_shapes";
   marker.id = 0;
   marker.type = shape;
   marker.pose.position.x = mean.coeff(0,0);
   marker.pose.position.y = mean.coeff(1,0);
   marker.pose.position.z = mean.coeff(2,0);
   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 1.0;
   marker.scale.x = 0.8;
   marker.scale.y = 0.8;
   marker.scale.z = 0.8;
   marker.color.r = 0.0f;
   marker.color.g = 1.0f;
   marker.color.b = 0.0f;
   marker.color.a = 1.0;
   marker_pub.publish(marker);

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

  
   
}

////////////////////////***************main function**********************///////////////////////////////////////////////

int main(int argc, char **argv)
{
   ros::init (argc, argv, "plotting_human");
   ros::NodeHandle nh;
   



   //subscribe
   ros::Subscriber PCLsub = nh.subscribe(PCL_TOPIC, 10, callback);

   //set frame_id
   
   crop_cloud_msg.header.frame_id=frame_id;
   obj_msg.header.frame_id = frame_id;
   proj_msg.header.frame_id = frame_id;
   passfiltered_ros.header.frame_id = frame_id;
   passfiltered_ros_again.header.frame_id = frame_id;
   marker.header.frame_id = frame_id;

   
   
   //publish
   pub_cropped_cloud=nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud",1);
   pub_extracted_cloud=nh.advertise<sensor_msgs::PointCloud2>("extracted_cloud",1);
   pub_projected_cloud=nh.advertise<sensor_msgs::PointCloud2>("projected",1);
   passthrough_filtered=nh.advertise<sensor_msgs::PointCloud2>("passfiltered",1);
   passthrough_filtered_again=nh.advertise<sensor_msgs::PointCloud2>("passfiltered_again",1);
   // mean_pub=
   // var_pub=
   marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

   
   

   ros::spin();
   return 0;

}
