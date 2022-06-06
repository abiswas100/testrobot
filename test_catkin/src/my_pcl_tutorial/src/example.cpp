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


ros::Publisher voxel_filtered;
ros::Publisher passthrough_filtered;
ros::Publisher plane_segmented;
ros::Publisher except_plane;
std::vector<ros::Publisher> cluster_vector;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  //Container: original data
  pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2);
  std::cout << "Frame id: " << inputCloud->header.frame_id << std::endl;;
  //Conversion: ros to PCL pointcloud2
  pcl_conversions::toPCL(*cloud_msg, *inputCloud);

  //#1-1 filtering: voxel grid filtering
  //Container: down sampled pointcloud2
  pcl::PCLPointCloud2::Ptr downsampled_pcl2 (new pcl::PCLPointCloud2);
  //filter setup and run
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (inputCloud);
  vg.setLeafSize (0.05, 0.05, 0.05);
  vg.filter (*downsampled_pcl2);
  //ROS_INFO_STREAM("data size: " << downsampled_pcl2->data.size());
  
  //#1-2 publish: voxel grid filtering result
  //Container: down sampled ros pointcloud
  sensor_msgs::PointCloud2 downsampled_ros;
  pcl_conversions::fromPCL(*downsampled_pcl2, downsampled_ros);
  voxel_filtered.publish (downsampled_ros);
  
  //#2 converstion from pcl pointcloud2 to pcl::PointCloud<pcl::PointXYZ>
  //container for down sampled point XYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*downsampled_pcl2, *downsampled_pclXYZ);
  
  //#3-1 filtering: passthrough filter
  //container: pcl pointXYZ after passthrough filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr passfiltered_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
  //filter setup
  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pass_filter.setInputCloud (downsampled_pclXYZ);
  pass_filter.setFilterFieldName ("x");
  pass_filter.setFilterLimits (-1, 2);
  pass_filter.setFilterLimitsNegative (true);
  pass_filter.filter (*passfiltered_pclXYZ);

  //#3-2 publish: passthrough result
  //Containers: pcl pointcloud 2, ros pointcloud 2
  pcl::PCLPointCloud2 passfiltered_pcl2;
  sensor_msgs::PointCloud2 passfiltered_ros;
  pcl::toPCLPointCloud2(*passfiltered_pclXYZ, passfiltered_pcl2);
  pcl_conversions::fromPCL(passfiltered_pcl2, passfiltered_ros);
  passthrough_filtered.publish (passfiltered_ros);

  
  //#4-1 segmentation: ransac planar segmentation
  //container: plane pcl pointXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pclXYZ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  //segmentation setup
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold(0.03);
  seg.setInputCloud(downsampled_pclXYZ);
  seg.segment(*inliers, *coefficients);

  //#4-2 extract plane
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (downsampled_pclXYZ);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*plane_pclXYZ);

  //#4-3 publish: plane segmentation result
  //Containers: pcl pointcloud 2, ros pointcloud 2
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
  

  
  //#clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(except_plane_pclXYZ);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.1);
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (20000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (except_plane_pclXYZ);
  ec.extract (cluster_indices);

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
      cluster_pclXYZ->points.push_back(except_plane_pclXYZ->points[index]);
    }
    std::cout << i << " PointCloud representing the Cluster: " << cluster_pclXYZ->points.size () << " data " << std::endl;
    cluster_pclXYZ->width = cluster_pclXYZ->points.size();
    cluster_pclXYZ->height = 1;
    cluster_pclXYZ->is_dense = true;
    pcl::toPCLPointCloud2( *cluster_pclXYZ ,cluster_pcl2);
    pcl_conversions::fromPCL(cluster_pcl2, cluster_ros);
    cluster_ros.header.frame_id = downsampled_pclXYZ->header.frame_id;
    std::cout << "Cluster Frame id: " << downsampled_pclXYZ->header.frame_id << std::endl;
    cluster_vector[i].publish (cluster_ros);
    
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  voxel_filtered = nh.advertise<sensor_msgs::PointCloud2> ("voxel_filtered", 1);
  passthrough_filtered = nh.advertise<sensor_msgs::PointCloud2> ("passthrough_filtered", 1);
  plane_segmented = nh.advertise<sensor_msgs::PointCloud2> ("plane_segmented", 1);
  except_plane = nh.advertise<sensor_msgs::PointCloud2> ("except_plane",1);

  // Spin
  ros::spin ();
}