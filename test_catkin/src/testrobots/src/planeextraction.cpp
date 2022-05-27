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
const double cropPercentage = 0.0;  // 0.00  to  0.20
const double meanK = 50.0;          // 50.0  to  100.0
const double stddevMulThresh = 0.5; // 0.5  to    1.0

// Euclidean clustering
double CLUSTER_TOLERANCE(0.10);
unsigned MIN_CLUSTER_SIZE(10);

enum class Normal
{
  eX = 0,
  eY,
  eZ
};
const Normal normal = Normal::eY;

pcl::PCDWriter m_writer;

// PCL plane extraction - a hard threshold, especially for if something goes wrong or lots of small (insignificant) planes
const unsigned MAX_PLANE_COUNT(8);
const double PLANE_EXTRACTION_CONTINUE_THRESHOLD(0.30); // While 30% of the original cloud is still there
unsigned m_pipelineStepCount =10; 

pcl::PointCloud<pcl::PointXYZ>::Ptr m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>); // this is the pointcloud that saves the final planeless cloud

const double minThreshold = 0.97; 


std::string printStepCount() //const std::string SegmentationPipeline::printStepCount() //const
{
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(2) << m_pipelineStepCount;
  return ss.str();
}

std::string printStepCount(unsigned addition) //const
{
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(2) << (m_pipelineStepCount + addition);
  return ss.str();
}

// function declarations
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void extractObjectInBoundingBox(double cropPercentage, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void removeNaNs(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest);
void removeOutliers(double meanK, double stddevMulThresh);
void performEuclideanExtraction();

void planeextract(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud); // this will be the function that does plane extract and recieves a pointer

/* Avhishek - Variable Description for cloud_cb
  
  Inputs ----------------------------------------- 
  sensor msg - Pointcloud2

  final_planeless_cloud - final cloud as the name suggests
  cloud_for_plane_extraction - as the name suggests
  cloud_plane - defined in function 
  cloud_f - defined in function
  coefficients - model coefficients in PCL segmentation
  inliers - pcl library object 

  PLANE_EXTRACTION_CONTINUE_THRESHOLD - defined as a global variable
  planeCount - defined in function



  Output -----------------------------------------
  Saves the cropped pointcloud in PCD. 

*/
void planeextract(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud){
  ROS_INFO_STREAM("getting here plane extract");
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_planeless_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*m_cloud, *final_planeless_cloud);
  ROS_INFO_STREAM("getting here 2");

        copyPointCloud(*m_cloud, *final_planeless_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_plane_extraction(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*m_cloud, *cloud_for_plane_extraction);
    
    // ROS_INFO_STREAM("getting here 2");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    
    ROS_INFO_STREAM("getting here 3");
    
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


    // // Create the extracting object (extraction based on indices)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
        
    // //Keep the removed points as NaN values to maintain the structure of the cloud
    extract.setKeepOrganized(true);
        
    unsigned planeCount =0;
    
    const unsigned numOriginalPoints = cloud_for_plane_extraction->size();   //How many points we originally started with
    unsigned numPointsExtracted =0;   //Keep track of how many points have so far been extracted  
    std::cout << numOriginalPoints << std::endl;
    ROS_INFO_STREAM("getting here 5");

      //Do until we extract a certain percentage of the points -or-
    // until the maximum number of planes is met, which is checked at the end of this loop
  while(numPointsExtracted < numOriginalPoints * (1.0 - PLANE_EXTRACTION_CONTINUE_THRESHOLD)) {
      
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_for_plane_extraction);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size () == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    
    //Accumulate the number of points extracted for this plane
    numPointsExtracted += inliers->indices.size();
      
    // Extract the inliers into a cloud that contains exactly (and only) the plane
    extract.setInputCloud(cloud_for_plane_extraction);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    std::cout << "   - Plane #" << planeCount << std::endl;
    std::cout << "     PointCloud representing the planar component: "
              << inliers->indices.size() << " data points." << std::endl;

    std::stringstream ss;
    // ss << m_baseName << "_step" << printStepCount() << "_plane_" << planeCount << ".pcd";
    ss << "h_step" << printStepCount() << "_plane_" << planeCount << ".pcd";
    m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_plane, false);

    ////////////
    //Calculate the plane normals - used to determine if the plane is a floor plane

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    //Calculate the least-squares plane value
    float nx, ny, nz, curvature;
    ne.computePointNormal(*cloud_for_plane_extraction, inliers->indices, nx, ny, nz, curvature);

    //Now calc each normal
    ne.setInputCloud(cloud_plane);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);
    
    // Compute the features
    ne.compute(*cloud_normals);
    std::cout << "     Least-squares plane fit: (" << nx << ", " << ny << ", " << nz << ")"
              << ", curvature = " << curvature << std::endl;

    bool minThresholdMet = false;
    if((normal == Normal::eX) && (fabs(nx) > minThreshold))
      minThresholdMet = true;
    if((normal == Normal::eY) && (fabs(ny) > minThreshold))
      minThresholdMet = true;
    if((normal == Normal::eZ) && (fabs(nz) > minThreshold))
      minThresholdMet = true;

    if(minThresholdMet)
    {
      std::cout << endl;
      std::cout << "     ==> Minimum threshold of " << minThreshold << " met. Extracting this plane (#" << planeCount << "). <==" << std::endl;
      std::cout << endl;
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minus_plane(new pcl::PointCloud<pcl::PointXYZ>);
      
      // Create another filtering object 
      pcl::ExtractIndices<pcl::PointXYZ> normalPlaneExtractor;
      normalPlaneExtractor.setInputCloud(final_planeless_cloud);
      normalPlaneExtractor.setIndices(inliers);
      normalPlaneExtractor.setNegative(true);
    
      //Keep the removed points as NaN values to maintain the structure of the cloud
      normalPlaneExtractor.setKeepOrganized(true);

      normalPlaneExtractor.filter(*cloud_minus_plane);

      std::cout << "final_planeless_cloud" << std::endl;
      std::cout << "width = " << final_planeless_cloud->width << std::endl;
      std::cout << "height = " << final_planeless_cloud->height << std::endl;

      std::cout << "cloud_minus_plane" << std::endl;
      std::cout << "width = " << cloud_minus_plane->width << std::endl;
      std::cout << "height = " << cloud_minus_plane->height << std::endl;

      //If this file already exists (i.e., part of the floor was removed in an earlier pass),
      // it will be over written here, and so just the updated, most recent version (with
      // the least amount of floor plane) will remain.
      std::stringstream ss;
      ss << "i_step" << printStepCount(1) << "_floorless_cloud.pcd";
      m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_minus_plane, false);
      
      //Make this is our new cloud
      copyPointCloud(*cloud_minus_plane, *final_planeless_cloud);
    }

    //Report the number of points extracted after this plane extraction
    std::cout << "     " << numPointsExtracted << " points extracted so far (of a total "
              << numOriginalPoints << ")." << std::endl << std::endl;
    
    // Remove the plane from our working cloud
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_for_plane_extraction.swap(cloud_f);

    //Increment the current plane count
    ++planeCount;
    
    //Put in a max number of planes so if something goes wrong, we don't write a billion and 1 files
    if(planeCount >= MAX_PLANE_COUNT) {
      std::cout << std::endl << "Maximum threshold reached: plane count = " << MAX_PLANE_COUNT << ". Forcing end of plane extraction." << std::endl;
      break;
    }
  }
    //std::cout << "Min-max after extracting plane:" << std::endl;
  //printMinMax(final_planeless_cloud);
  
  //Copy this into the destination cloud
//   copyPointCloud(*final_planeless_cloud, *destination);

  //Also, cache a copy of teh post-extracted plane, so we can restart from this point
  // again in the future
  copyPointCloud(*final_planeless_cloud, *m_postPlaneExtractedCloud);
  std::cout << "     " << "Post-plane extraction cloud cached for possible re-use again" << std::endl;
  
  m_pipelineStepCount += 10;

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 
    ROS_INFO_STREAM("getting here 1");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_planeless_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*m_cloud);

    planeextract(m_cloud);

    /*
      Avhishek - all this below code is now in plane extract, run the code and remove things from below, this is our driver code 

      1. do planeextract(m_cloud)
      2. extract BoundingBox
      3. remove outliers
      4. euclidean distance
      5.
    */


//   // Call the other functions here
//   extractObjectInBoundingBox(10, m_postPlaneExtractedCloud);

//   removeOutliers(double meanK, double stddevMulThresh);

//   performEuclideanExtraction();

}

// call this function at the end of cloud_cb
void extractObjectInBoundingBox(double cropPercentage, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   
    std::cout<< cropPercentage << std::endl;
//   // Extract the object PCL knowing the bounding box values, possibly with an additional cropping border (reduced by the crop percentage)
//   unsigned x_delta = m_boundingBox.xmax - m_boundingBox.xmin;
//   unsigned y_delta = m_boundingBox.ymax - m_boundingBox.ymin;
//   unsigned cloudWidth = m_cloud->width;
//   unsigned cloudHeight = m_cloud->height;

//   std::cout << "Cloud width = " << cloudWidth << std::endl;
//   std::cout << "Cloud height = " << cloudHeight << std::endl;
//   std::cout << "Crop percentage = " << (cropPercentage * 100) << "%" << std::endl;
//   std::cout << "BB xmin = " << m_boundingBox.xmin << std::endl;
//   std::cout << "BB xmax = " << m_boundingBox.xmax << std::endl;
//   std::cout << "BB ymin = " << m_boundingBox.ymin << std::endl;
//   std::cout << "BB ymax = " << m_boundingBox.ymax << std::endl;
//   std::cout << "BB xmin, cropped = " << m_boundingBox.xmin + static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
//   std::cout << "BB xmax, cropped = " << m_boundingBox.xmax - static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
//   std::cout << "BB ymin, cropped = " << m_boundingBox.ymin + static_cast<unsigned>(y_delta * cropPercentage) << std::endl;
//   std::cout << "BB ymax, cropped = " << m_boundingBox.ymax - static_cast<unsigned>(y_delta * cropPercentage) << std::endl;

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrop(new pcl::PointCloud<pcl::PointXYZ>);
  
//     // Otherwise we have an organized cloud, so use this version
//     UNL_Robotics::extractFrame<pcl::PointXYZ>(m_cloud, cloudCrop,
//                                                 m_boundingBox.xmin + static_cast<unsigned>(x_delta * cropPercentage),
//                                                 m_boundingBox.xmax - static_cast<unsigned>(x_delta * cropPercentage),
//                                                 m_boundingBox.ymin + static_cast<unsigned>(y_delta * cropPercentage),
//                                                 m_boundingBox.ymax - static_cast<unsigned>(y_delta * cropPercentage));
// }

  // this will be used to save the extracted pointcloud as a pcd file
  std::stringstream ss;
  ss << "k_step" << printStepCount() << "_extractBBcrop" << std::setprecision(2) << std::fixed << cropPercentage << ".pcd";
//   m_writer.write<pcl::PointXYZ>(ss.str(), *cloudCrop, false);
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloud, false);

  // Copy this into the destination cloud
//   copyPointCloud(*cloudCrop, *destination);

//   m_pipelineStepCount += 10;
}

void removeNaNs(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest)
{
  //m_cloud->is_dense = false;  
  //boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  //pcl::removeNaNFromPointCloud(*m_cloud, *m_cloud, *indices);

  /*  
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(m_cloud);
  extract.setIndices(indices);
  extract.setNegative(true);
  extract.filter(*m_cloud);
  */

  for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = source->begin(); it != source->end(); ++it) {

    if(!(std::isnan(it->x)  ||  std::isnan(it->y)  ||  std::isnan(it->z))) {
      dest->push_back(*it);
    }
  }

  pcl::PointXYZ pt = *source->begin();
  cout << "pt = " << pt.x << " " << pt.y << " " << pt.z << std::endl;
  cout << "is nan x : " << std::isnan(pt.x) << std::endl;
  cout << "combined ! is nan :  " << !(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) << std::endl;

  /*  
  std::copy_if(source->begin(), source->end(), dest->begin(), 
               [](pcl::PointXYZ pt)
               {
                 return(!(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)));
               });
  */
}

void removeOutliers(double meanK, double stddevMulThresh)
{
  //Remove outliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removedOutliers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(m_postPlaneExtractedCloud); // *m_cloud
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stddevMulThresh);
  sor.filter(*cloud_removedOutliers);
  
  std::stringstream ss;
  ss << "r_step" << printStepCount() << "_removedOutliers.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_removedOutliers, false);

  //Make this is our new working cloud
  copyPointCloud(*cloud_removedOutliers, *m_postPlaneExtractedCloud); //*m_cloud

  m_pipelineStepCount += 10;
}

void performEuclideanExtraction()
{
  //Euclidean Cluster Extraction

  //Start by removing NaNs, if they exist
  pcl::PointCloud<pcl::PointXYZ>::Ptr nanlessCloud(new pcl::PointCloud<pcl::PointXYZ>);
  removeNaNs(m_postPlaneExtractedCloud, nanlessCloud);
  copyPointCloud(*nanlessCloud, *m_postPlaneExtractedCloud); //*m_cloud
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(m_postPlaneExtractedCloud); //m_cloud 
    
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  std::cout << "Cluster tolearance set to: " << CLUSTER_TOLERANCE << std::endl;
  ec.setClusterTolerance(CLUSTER_TOLERANCE);

  std::cout << "Minimum cluster size set to: " << MIN_CLUSTER_SIZE << std::endl;
  ec.setMinClusterSize(MIN_CLUSTER_SIZE);

  //This is set based on the bounding box size
        // define bounding box first before uncommenting this line
    //   unsigned maxClusterSize = (m_boundingBox.xmax - m_boundingBox.xmin) * (m_boundingBox.ymax - m_boundingBox.ymin);
//   ec.setMaxClusterSize(maxClusterSize);
//   std::cout << "Maximum cluster size set to: " << maxClusterSize << std::endl;
  ec.setSearchMethod(tree);
  std::cout << "Set search method to tree" << std::endl;
  m_postPlaneExtractedCloud->is_dense = false; //m_cloud
  ec.setInputCloud(m_postPlaneExtractedCloud);
  std::cout << "Set input cloud" << std::endl;
  ec.extract(cluster_indices);
  std::cout << "   " << cluster_indices.size() << " cluster" << (cluster_indices.size() != 1 ? "s" : "")
            << " found in pointcloud." << std::endl;
  
  //For the file name, set the cluster number width (so that numbers are formatted as  07  if there are more than 10)
  unsigned clusterNumberWidth = floor(log10(cluster_indices.size())) + 1;
  
  //Loop over all the clusters found
  for(auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back((*m_postPlaneExtractedCloud)[*pit]); //m_cloud
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
    std::cout << "    - Cluster extracted with " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "c_step" << printStepCount() << "_euclideanCluster_" << std::setfill('0') 
       << std::setw(clusterNumberWidth) << std::distance(cluster_indices.begin(), it) << ".pcd";
    m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
  }
    
  //Make the biggest cluster (cluster_indices[0]) our new working cloud  
  if(cluster_indices.size() > 0) {
    copyPointCloud(*clusters[0], *m_postPlaneExtractedCloud); // *m_cloud
    
    /* Do this if we ever figure out how to get cluster extraction to work on the NaN-full cloud
      
      pcl::ExtractIndices<pcl::PointXYZ> clusterExtractor;
      clusterExtractor.setInputCloud(m_cloud);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices(cluster_indices[0]));
      clusterExtractor.setIndices(inliers);
      clusterExtractor.setNegative(true);
      //Keep the removed points as NaN values to maintain the structure of the cloud
      clusterExtractor.setKeepOrganized(true);
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
      clusterExtractor.filter(*clusterCloud);
      copyPointCloud(*clusterCloud, *m_cloud);
    */
  }
  
  m_pipelineStepCount += 10;  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe(PCL_TOPIC, 10, cloud_cb);

  ros::spin();


  return 0;
}
