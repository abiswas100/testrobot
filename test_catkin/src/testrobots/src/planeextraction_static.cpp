
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


static const std::string PCL_TOPIC = "/camera/depth/points";
int wait_now = 1;
const double normalThreshold = 0.99; // for simulations
const double cropPercentage = 0.10;  // 0.00  to  0.20
const double minThreshold = 0.97; 

enum class Normal
{
  eX = 0,
  eY,
  eZ
};

const Normal normal = Normal::eY;
pcl::PCDWriter m_writer;
std::ofstream m_out;

//Camera Specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(1920);
const unsigned CAMERA_NUM_PIXELS_HEIGHT(1080);
const double CAMERA_HORIZONTAL_VIEW_ANGLE(1.19);

//bounding box attributes
unsigned xmin = 894;
unsigned xmax = 664;
unsigned ymin = 563;
unsigned ymax = 663;


// PCL plane extraction - a hard threshold, especially for if something goes wrong or lots of small (insignificant) planes
const unsigned MAX_PLANE_COUNT(8);
const double PLANE_EXTRACTION_CONTINUE_THRESHOLD(0.30); // While 30% of the original cloud is still there
unsigned m_pipelineStepCount =10; 

pcl::PointCloud<pcl::PointXYZ>::Ptr m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>); 

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


// mcloud is replacing m_postPlaneExtractedCloud,  so change it back before use or perish

///////////////////////////////////////////// Variable Description ///////////////////////////////////////////////////////////
void planeextract(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud); // this will be the function that does plane extract and recieves a pointer 
void extractObjectInBoundingBox(double cropPercentage);
//////////////////////////////////////////////////////////////////////////////////////////////////


void planeextract(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud)
{
    ROS_INFO_STREAM("getting here plane extract");

  ////////////////////////////////////////////////////////// This code is to run extraction without segmentation//////////////////////////        
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrop(new pcl::PointCloud<pcl::PointXYZ>);
  //   testrobots::extractFrame<pcl::PointXYZ>(m_cloud, cloudCrop,
  //                                             xmin,
  //                                             xmax,
  //                                             ymin,
  //                                             ymax);                                            

  // std::cout << "getting here after extract frame " << std::endl;
  // // this will be used to save the extracted pointcloud as a pcd file
  // std::stringstream ss;
  // ss << printStepCount() << "_extractBBcrop" << ".pcd";
  // m_writer.write<pcl::PointXYZ>(ss.str(), *cloudCrop, false);



    pcl::PointCloud<pcl::PointXYZ>::Ptr final_planeless_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*m_cloud, *final_planeless_cloud);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_plane_extraction(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*m_cloud, *cloud_for_plane_extraction);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100); 
    seg.setDistanceThreshold(0.01);
    
    // // Create the extracting object (extraction based on indices)
    pcl::ExtractIndices<pcl::PointXYZ> extract;
        
    // //Keep the removed points as NaN values to maintain the structure of the cloud
    extract.setKeepOrganized(true);
        
    unsigned planeCount =0;
    
    const unsigned numOriginalPoints = cloud_for_plane_extraction->size();   //How many points we originally started with
    unsigned numPointsExtracted =0;   //Keep track of how many points have so far been extracted  
    std::cout << numOriginalPoints << std::endl;
    

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
    extract.setNegative(true);
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
    ne.setRadiusSearch(0.03); //0.03
    
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
      m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_minus_plane, false);pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrop(new pcl::PointCloud<pcl::PointXYZ>);
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

  //Also, cache a copy of teh post-extracted plane, so we can restart from this point
  // again in the future
  copyPointCloud(*final_planeless_cloud, *m_postPlaneExtractedCloud);
  std::cout << "     " << "Post-plane extraction cloud cached for possible re-use again" << std::endl;
  
  m_pipelineStepCount += 10;
  }
}

void extractObjectInBoundingBox(double cropPercentage)
{   
  std::cout<< cropPercentage << std::endl;
  // Extract the object PCL knowing the bounding box values, possibly with an additional cropping border (reduced by the crop percentage)
  std::cout << "" << std::endl;
  std::cout << "getting here in extract ObjectinBB " << std::endl;
  unsigned x_delta = xmax - xmin;
  unsigned y_delta = ymax - ymin;
  unsigned cloudWidth = m_postPlaneExtractedCloud->width;  // m_cloud
  unsigned cloudHeight = m_postPlaneExtractedCloud->height;

  std::cout << "Cloud width = " << cloudWidth << std::endl;
  std::cout << "Cloud height = " << cloudHeight << std::endl;
  std::cout << "Crop percentage = " << (cropPercentage * 100) << "%" << std::endl;
  std::cout << "BB xmin = " << xmin << std::endl;
  std::cout << "BB xmax = " << xmax << std::endl;
  std::cout << "BB ymin = " << ymin << std::endl;
  std::cout << "BB ymax = " << ymax << std::endl;
  // std::cout << "Xdelta = " << x_delta << std::endl;
  // std::cout << "Ydelta = " << y_delta << std::endl;
  // std::cout << "BB xmin, cropped = " << xmin + static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
  // std::cout << "BB xmax, cropped = " << xmax - static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
  // std::cout << "BB ymin, cropped = " << ymin + static_cast<unsigned>(y_delta * cropPercentage) << std::endl;
  // std::cout << "BB ymax, cropped = " << ymax - static_cast<unsigned>(y_delta * cropPercentage) << std::endl;
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrop(new pcl::PointCloud<pcl::PointXYZ>);

  // extract the BB frame 
  // testrobots::extractFrame<pcl::PointXYZ>(m_postPlaneExtractedCloud, cloudCrop,
  //                                             xmin + static_cast<unsigned>(x_delta * cropPercentage),
  //                                             xmax - static_cast<unsigned>(x_delta * cropPercentage),
  //                                             ymin + static_cast<unsigned>(y_delta * cropPercentage),
  //                                             ymax - static_cast<unsigned>(y_delta * cropPercentage));

  
  testrobots::extractFrame<pcl::PointXYZ>(m_postPlaneExtractedCloud, cloudCrop,
                                              xmin,
                                              xmax,
                                              ymin,
                                              ymax);                                            

  std::cout << "getting here after extract frame " << std::endl;
  // this will be used to save the extracted pointcloud as a pcd file
  std::stringstream ss;
  ss << printStepCount() << "_extractBBcrop" << ".pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloudCrop, false);


  // Copy this into the destination cloud
  copyPointCloud(*cloudCrop, *m_postPlaneExtractedCloud);

  m_pipelineStepCount += 10;
}


int main(int argc, char **argv)
{
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
  
    //call function here

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/apramani/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk 
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/tran/testrobot/test_catkin/src/testrobots/src/PointCloud.pcd", *cloud); //loads the PointCloud data from disk 
    
    planeextract(cloud); 
    extractObjectInBoundingBox(cropPercentage);
}