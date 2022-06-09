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
struct Point2D {    //define points for 2d plane
     double x, y;
};


const Normal normal = Normal::eY;
pcl::PCDWriter m_writer;
std::ofstream m_out;

//Camera Specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(1920);
const unsigned CAMERA_NUM_PIXELS_HEIGHT(1080);
const double CAMERA_HORIZONTAL_VIEW_ANGLE(1.19);

//bounding box attributes
unsigned xmin = 0;
unsigned xmax = 0;
unsigned ymin = 0;
unsigned ymax = 0;


// PCL plane extraction - a hard threshold, especially for if something goes wrong or lots of small (insignificant) planes
const unsigned MAX_PLANE_COUNT(8);
const double PLANE_EXTRACTION_CONTINUE_THRESHOLD(0.30); // While 30% of the original cloud is still there
unsigned m_pipelineStepCount =10; 

pcl::PointCloud<pcl::PointXYZ>::Ptr m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>); // this is the pointcloud that saves the final planeless cloud
std::vector<Point2D> hullpoints;


const double minThreshold = 0.97; 
double poseAMCLx = 0.0;
double poseAMCLy = 0.0;
double poseAMCLw = 0.0;


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



// function declarations **********************************************************************************************
void poseCallback(const geometry_msgs::PoseWithCovarianceConstPtr &pose_msg);
void BBoxCallback (const testrobots::Boundingbox::ConstPtr &msg);
// mcloud is replacing m_postPlaneExtractedCloud,  so change it back before use or perish
void planeextract(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud); // this will be the function that does plane extract and recieves a pointer 
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void extractObjectInBoundingBox(double cropPercentage);
void removeNaNs(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest);
void removeOutliers(double meanK, double stddevMulThresh);
void performEuclideanExtraction();
void calcBoundingBoxInWorldCoords2(bool visualizeBB, double x, double y, double theta);
//function callbacks*************************************************************************************************

void poseCallback(const geometry_msgs::PoseWithCovarianceConstPtr &pose_msg){
    
    poseAMCLx = pose_msg->pose.position.x;
    poseAMCLy = pose_msg->pose.position.y;
    poseAMCLw = pose_msg->pose.orientation.w;

}

void BBoxCallback (const testrobots::Boundingbox::ConstPtr &msg)
{ 
  // check the value of pause, if plane extraction is working then ignore Bbox messages 
  if(wait_now == 1){ 
      // ROS_INFO("class: %s", msg->Class.c_str());  
      // ROS_INFO("%f", msg->probability);
      // ROS_INFO("%ld", msg->xmin);
      // ROS_INFO("%ld", msg->xmax);
      // ROS_INFO("%ld", msg->ymin);
      // ROS_INFO("%ld", msg->xmax);



      std::string objectName = msg->Class.c_str();
      xmin = msg->xmin;
      xmax = msg->xmax;
      ymin = msg->ymin;
      ymax = msg->ymax;

      // unsigned x_delta = xmax - xmin;
      // unsigned y_delta = ymax - ymin; 
      // ROS_INFO_STREAM("  " << objectName  << "  -  Probability " << std::setprecision(4) << (msg->probability*100) << "%" ); // not needed 
      // ROS_INFO_STREAM("    " << "BB Min (x,y) = (" << xmin << ", " << ymin << ")" );
      // ROS_INFO_STREAM("    " << "BB Max (x,y) = (" << xmax << ", " << ymax << ")" );
      // not needed ************
      // std::cout << "*) Object type:  " << objectName << std::endl;
      // std::cout << "   Probability  " << std::setprecision(4) << (msg->probability*100.0) << "%" << std::endl;
    //*******************

    //   // Avhishek - Don't know why  this is being done what is the use of calculating objectAngleOffset

      //Calculate the angle offset of the picture relative to the center of the view port
      // unsigned x_centerBB = xmin + static_cast<unsigned>(x_delta/2);
      // unsigned y_centerBB = ymin + static_cast<unsigned>(y_delta/2);
      // int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH/2) - x_centerBB;   //Can be negative! This orientation assumes CCW=+
      // double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

      // Avhishek - m_out is being used for printing and is declared at the top of the file
      // m_out << "   " << "Bounding Box (x,y):"
      //       << "   Min = (" << xmin << ", " << ymin << ")"
      //       << "   Max = (" << xmax << ", " << ymax << ")"
      //       << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
      // m_out << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;
  }

  else {
    ROS_INFO_STREAM("No need for Bbox, plane segmentation is still working" );
  }
}


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
  // setting pause to false for wait for segmentation to complete
  wait_now = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr final_planeless_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*m_cloud, *final_planeless_cloud);
  

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
    seg.setMaxIterations(1000); 
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
      m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_minus_plane, false);
      
      // Make this is our new cloud
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

  //Also, cache a copy of teh post-extracted plane, so we can restart from this point
  // again in the future
  copyPointCloud(*final_planeless_cloud, *m_postPlaneExtractedCloud);
  std::cout << "     " << "Post-plane extraction cloud cached for possible re-use again" << std::endl;
  
  m_pipelineStepCount += 10;

}
/*
Avhishek - This function is the driver function for the whole code , it calls all the other functions after planeextracct
*/
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{ 
    ROS_INFO_STREAM("getting here in Cloud callback");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_planeless_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*m_cloud);

    // added PCD saving code 
    
    // std::cout << xmax << std::endl;
    // std::cout << xmin << std::endl;
    // std::cout << ymax << std::endl;
    // std::cout << ymin << std::endl;
    // this will be used to save the extracted pointcloud as a pcd file
    // std::stringstream ss;
    // ss << "_extractBBcrop"<<".pcd";
    // m_writer.write<pcl::PointXYZ>(ss.str(), *m_cloud, false);

    // exit(1);

    /*
      Avhishek - all this below code is now in plane extract, run the code and remove things from below, this is our driver code 

      1. do planeextract(m_cloud)
      2. extract BoundingBox
      3. remove outliers
      4. euclidean distance
      5. get hullpoints from CalBoundingBox in World Coordinate
      6. Use hull points to write the map 
    */

    
    planeextract(m_cloud);

    extractObjectInBoundingBox(cropPercentage);

    // removeOutliers(meanK,  stddevMulThresh);

    // performEuclideanExtraction();

    // bool visualizeBB = true;
    // // hullpoints = calcBoundingBoxInWorldCoords(visualizeBB,poseAMCLx,
    // //                                                       poseAMCLy,
    // //                                                       poseAMCLw);

    // calcBoundingBoxInWorldCoords2(visualizeBB,poseAMCLx,poseAMCLy,poseAMCLw);

    // std::string pgmPath("./buildings/pureza/maps");
    // std::string yamlFilename("pureza.yaml");

    // std::string yamlFilepath(pgmPath + "/" + yamlFilename);
    //       testrobots::map_writer writer;
    //       bool insertResult = writer.insertObject(yamlFilepath, pgmPath, hullPoints);
    //       if(insertResult)
    //         ROS_INFO_STREAM("Updated the map with long-term object successfully");
    //       else
    //         ROS_ERROR_STREAM("Failed to insert the object into the map");
   
        
}

// call this function at the end of cloud_cb

/*
Avhishek - the function uses the global cloud finalplainless_cloud and updates it after the extract
*/
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
  std::cout << "BB xmin, cropped = " << xmin + static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
  std::cout << "BB xmax, cropped = " << xmax - static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
  std::cout << "BB ymin, cropped = " << ymin + static_cast<unsigned>(y_delta * cropPercentage) << std::endl;
  std::cout << "BB ymax, cropped = " << ymax - static_cast<unsigned>(y_delta * cropPercentage) << std::endl;
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrop(new pcl::PointCloud<pcl::PointXYZ>);

  // extract the BB frame 
  // testrobots::extractFrame<pcl::PointXYZ>(m_postPlaneExtractedCloud, cloudCrop,
  //                                             xmin + static_cast<unsigned>(x_delta * cropPercentage),
  //                                             xmax - static_cast<unsigned>(x_delta * cropPercentage),
  //                                             ymin + static_cast<unsigned>(y_delta * cropPercentage),
  //                                             ymax - static_cast<unsigned>(y_delta * cropPercentage),
  //                                             CAMERA_NUM_PIXELS_WIDTH, CAMERA_NUM_PIXELS_HEIGHT);

  std::cout << "getting here after extract frame " << std::endl;
  // this will be used to save the extracted pointcloud as a pcd file
  std::stringstream ss;
  ss << "k_step" << printStepCount() << "_extractBBcrop" << std::setprecision(2) << std::fixed << cropPercentage << ".pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloudCrop, false);


  // Copy this into the destination cloud
  copyPointCloud(*cloudCrop, *m_postPlaneExtractedCloud);

  m_pipelineStepCount += 10;
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

  // settings pause to true so that bbox can continue
  wait_now = 1;

}



void calcBoundingBoxInWorldCoords2(bool visualizeBB, double x, double y, double theta)
{
  typedef pcl::PointXYZ PointType;
  
	pcl::PointCloud<PointType>::Ptr cloud = m_postPlaneExtractedCloud;
 
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //correct vertical between main directions
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
 
	 std::cout << "Eigenvalue va(3x1):\n" << eigenValuesPCA << std::endl;
	 std::cout << "Feature vector ve(3x3):\n" << eigenVectorsPCA << std::endl;
	 std::cout << "centroid point (4x1):\n" << pcaCentroid << std::endl;
	/*
	 // Another way to calculate the eigenvalues ​​and eigenvectors of the point cloud covariance matrix: through the pca interface in PCL as follows
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	 std::cerr << std::endl << "EigenVectors: "<< pca.getEigenVectors() << std::endl;//Calculate the feature vector
	 std::cerr << std::endl << "EigenValues: "<< pca.getEigenValues() << std::endl;//Calculate characteristic values
	*/
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();
 
	 std::cout << "Transformation matrix tm(4x4):\n" << tm << std::endl;
	 std::cout << "inverter matrix tm'(4x4):\n" << tm_inv << std::endl;
 
	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);
 
	PointType min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());
 
	 std::cout << "Centre c1(3x1):\n" << c1 << std::endl;
 
	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);
 
	Eigen::Vector3f whd, whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
	 float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3; //The average scale of the point cloud, used to set the size of the main direction arrow
 
	std::cout << "width1=" << whd1(0) << std::endl;
	std::cout << "heght1=" << whd1(1) << std::endl;
	std::cout << "depth1=" << whd1(2) << std::endl;
	std::cout << "scale1=" << sc1 << std::endl;
 
	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
	const Eigen::Vector3f    bboxT1(c1);
 
	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT(c);
 
 
	 //The main direction of the point cloud transformed to the origin
	PointType op;
	op.x = 0.0;
	op.y = 0.0;
	op.z = 0.0;
	Eigen::Vector3f px, py, pz;
	Eigen::Affine3f tm_aff(tm);
	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
	PointType pcaX;
	pcaX.x = sc1 * px(0);
	pcaX.y = sc1 * px(1);
	pcaX.z = sc1 * px(2);
	PointType pcaY;
	pcaY.x = sc1 * py(0);
	pcaY.y = sc1 * py(1);
	pcaY.z = sc1 * py(2);
	PointType pcaZ;
	pcaZ.x = sc1 * pz(0);
	pcaZ.y = sc1 * pz(1);
	pcaZ.z = sc1 * pz(2);
 
 
	 //The main direction of the initial point cloud
	PointType cp;
	cp.x = pcaCentroid(0);
	cp.y = pcaCentroid(1);
	cp.z = pcaCentroid(2);
	PointType pcX;
	pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
	pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
	pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
	PointType pcY;
	pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
	pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
	pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
	PointType pcZ;
	pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
	pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
	pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;
 
 
	// visualization
	// pcl::visualization::PCLVisualizer viewer;
 
	//  pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0); //Point cloud related to the origin
	// viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
	// viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
	// viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
	// viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");
 
	// viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
	// viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
	// viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");

  // /*  
	//  pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 0, 0); //The initial point cloud input is related
	// viewer.addPointCloud(cloud, color_handler, "cloud");
	// viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
	// viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
	// viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
 
	// viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	// viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	// viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
  // */
  
	// viewer.addCoordinateSystem(0.5f*sc1);
	// viewer.setBackgroundColor(1.0, 1.0, 1.0);
	// while (!viewer.wasStopped())
	// {
	// 	viewer.spinOnce(100);
	// }
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe(PCL_TOPIC, 10, cloud_cb);

  ros::Subscriber BBsub = n.subscribe("/BBox", 10, BBoxCallback);

  // ros::Subscriber current_pose = n.subscribe("/amcl_pose", 10, poseCallback);

  ros::spin();

  return 0;
}
