#include <ros/ros.h>
#include <testrobots/Boundingbox.h>
#include <pclUtils.h>
#include <convexHull.h>
#include <map_writer.h>
#include <string>

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
// std
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <testrobots/BoundingBox.h>

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
const Normal normal = Normal::eY;

pcl::PCDWriter m_writer;

std::ofstream m_out;

//Camera Specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(1920);
const unsigned CAMERA_NUM_PIXELS_HEIGHT(1080);
const double CAMERA_HORIZONTAL_VIEW_ANGLE(1.19);

unsigned xmin = 0;
unsigned xmax = 0;
unsigned ymin = 0;
unsigned ymax = 0;

struct Point2D {    //define points for 2d plane
     double x, y;
  };

// PCL plane extraction - a hard threshold, especially for if something goes wrong or lots of small (insignificant) planes
const unsigned MAX_PLANE_COUNT(8);
const double PLANE_EXTRACTION_CONTINUE_THRESHOLD(0.30); // While 30% of the original cloud is still there
unsigned m_pipelineStepCount =10; 

pcl::PointCloud<pcl::PointXYZ>::Ptr m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>); // this is the pointcloud that saves the final planeless cloud

const double minThreshold = 0.97; 

float poseAMCLx = 0;
float poseAMCLy = 0;
float poseAMCLw = 0;

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


std::vector<Point2D> findConvexHull(std::vector<Point2D> points);
// function declarations
void BBoxCallback (const testrobots::Boundingbox::ConstPtr &msg);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
void extractObjectInBoundingBox(double cropPercentage);
void removeNaNs(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest);
void removeOutliers(double meanK, double stddevMulThresh);
void performEuclideanExtraction();
std::vector<testrobots::Point2D> calcBoundingBoxInWorldCoords(bool visualizeBB, double camera_x, double camera_y, double camera_theta);


void planeextract(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud); // this will be the function that does plane extract and recieves a pointer

void poseCallback(const geometry_msgs::PoseWithCovarianceConstPtr &pose_msg){
    
    poseAMCLx = pose_msg->pose.position.x;
    poseAMCLy = pose_msg->pose.position.y;
    poseAMCLw = pose_msg->pose.orientation.w;

}
void BBoxCallback (const testrobots::Boundingbox::ConstPtr &msg)
{ 
  // check the value of pause, if plane extraction is working then ignore Bbox messages 
  if(wait_now == 1){ 
      ROS_INFO("class: %s", msg->Class.c_str());  
      ROS_INFO("%f", msg->probability);
      ROS_INFO("%ld", msg->xmin);
      ROS_INFO("%ld", msg->xmax);
      ROS_INFO("%ld", msg->ymin);
      ROS_INFO("%ld", msg->xmax);



      std::string objectName = msg->Class.c_str();
      xmin = msg->xmin;
      xmax = msg->xmax;
      ymin = msg->ymin;
      ymax = msg->ymax;

      unsigned x_delta = xmax - xmin;
      unsigned y_delta = ymax - ymin; 
      ROS_INFO_STREAM("  " << objectName  << "  -  Probability " << std::setprecision(4) << (msg->probability*100) << "%" ); // not needed 
      ROS_INFO_STREAM("    " << "BB Min (x,y) = (" << xmin << ", " << ymin << ")" );
      ROS_INFO_STREAM("    " << "BB Max (x,y) = (" << xmax << ", " << ymax << ")" );
      // not needed ************
      std::cout << "*) Object type:  " << objectName << std::endl;
      std::cout << "   Probability  " << std::setprecision(4) << (msg->probability*100.0) << "%" << std::endl;
    //*******************

    //   // Avhishek - Don't know why  this is being done what is the use of calculating objectAngleOffset

      //Calculate the angle offset of the picture relative to the center of the view port
      unsigned x_centerBB = xmin + static_cast<unsigned>(x_delta/2);
      unsigned y_centerBB = ymin + static_cast<unsigned>(y_delta/2);
      int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH/2) - x_centerBB;   //Can be negative! This orientation assumes CCW=+
      double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

      // Avhishek - m_out is being used for printing and is declared at the top of the file
      m_out << "   " << "Bounding Box (x,y):"
            << "   Min = (" << xmin << ", " << ymin << ")"
            << "   Max = (" << xmax << ", " << ymax << ")"
            << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
      m_out << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;
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

    removeOutliers(meanK,  stddevMulThresh);

    performEuclideanExtraction();

    bool visualizeBB = true;
    // hullpoints = calcBoundingBoxInWorldCoords(visualizeBB,m_currentPose.x,
    //                                                       m_currentPose.y,
    //                                                       m_currentPose.yaw);


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
  testrobots::extractFrame<pcl::PointXYZ>(m_postPlaneExtractedCloud, cloudCrop,
                                              xmin + static_cast<unsigned>(x_delta * cropPercentage),
                                              xmax - static_cast<unsigned>(x_delta * cropPercentage),
                                              ymin + static_cast<unsigned>(y_delta * cropPercentage),
                                              ymax - static_cast<unsigned>(y_delta * cropPercentage),
                                              CAMERA_NUM_PIXELS_WIDTH, CAMERA_NUM_PIXELS_HEIGHT);

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



// std::vector<testrobots::Point2D> calcBoundingBoxInWorldCoords(bool visualizeBB, double camera_x, double camera_y, double camera_theta)
// {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = m_postPlaneExtractedCloud;
  
//   // Compute principal directions (equivalent to principal components / PCA)
//   Eigen::Vector4f pcaCentroid;
//   pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);


//   //Two methods: compute Eigen vectors, or use a PCA object

//   //Method 1
//   Eigen::Matrix3f covariance;
//   computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
//   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
//   Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
//   //The following line is necessary for proper orientation in some cases.
//   //The numbers come out the same without it, but the signs are
//   //   different and the box doesn't get correctly oriented in some cases.
//    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
  
// /*
//    //Method 2
//    // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PCA<pcl::PointXYZ> pca;
//    pca.setInputCloud(cloudSegmented);
//    pca.project(*cloudSegmented, *cloudPCAprojection);
//    Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
// */


  
//   // Transform the original cloud to the origin where the principal components correspond to the axes.
//   Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
//   projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
//   projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
//   // Get the minimum and maximum points of the transformed cloud.
//   pcl::PointXYZ minPoint, maxPoint;
//   pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
//   const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

//   // For the transformation back to world coordinates
//   const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
//   const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

//   std::stringstream ss;
//   ss <<  "world_step" << printStepCount() << "_PCA.pcd";
//   m_writer.write<pcl::PointXYZ>(ss.str(), *cloudSegmented, false);

//   std::cout << "PCA:" << std::endl;
//   std::cout << eigenVectorsPCA << std::endl;
//   std::cout << "Bounding box transform:" << std::endl;
//   std::cout << bboxTransform << std::endl;

//   Eigen::Vector3f p1(minPoint.x, minPoint.y, minPoint.z);
//   Eigen::Vector3f p2(minPoint.x, minPoint.y, maxPoint.z);
//   Eigen::Vector3f p3(maxPoint.x, minPoint.y, maxPoint.z);
//   Eigen::Vector3f p4(maxPoint.x, minPoint.y, minPoint.z);
//   Eigen::Vector3f p5(minPoint.x, maxPoint.y, minPoint.z);
//   Eigen::Vector3f p6(minPoint.x, maxPoint.y, maxPoint.z);
//   Eigen::Vector3f p7(maxPoint.x, maxPoint.y, maxPoint.z);
//   Eigen::Vector3f p8(maxPoint.x, maxPoint.y, minPoint.z);

//   std::cout << "Points - untransformed" << std::endl;
//   std::cout << "P1" << std::endl << p1  << std::endl;
//   std::cout << "P2" << std::endl << p2  << std::endl;
//   std::cout << "P3" << std::endl << p3  << std::endl;
//   std::cout << "P4" << std::endl << p4  << std::endl;
//   std::cout << "P5" << std::endl << p5  << std::endl;
//   std::cout << "P6" << std::endl << p6  << std::endl;
//   std::cout << "P7" << std::endl << p7  << std::endl;
//   std::cout << "P8" << std::endl << p8  << std::endl;

//   // Transform back to world coordinates
//   Eigen::Vector3f pr1 = bboxQuaternion * p1 + bboxTransform;
//   Eigen::Vector3f pr2 = bboxQuaternion * p2 + bboxTransform;
//   Eigen::Vector3f pr3 = bboxQuaternion * p3 + bboxTransform;
//   Eigen::Vector3f pr4 = bboxQuaternion * p4 + bboxTransform;
//   Eigen::Vector3f pr5 = bboxQuaternion * p5 + bboxTransform;
//   Eigen::Vector3f pr6 = bboxQuaternion * p6 + bboxTransform;
//   Eigen::Vector3f pr7 = bboxQuaternion * p7 + bboxTransform;
//   Eigen::Vector3f pr8 = bboxQuaternion * p8 + bboxTransform;

//   std::cout << std::endl << std::endl;
//   std::cout << "Points with rotation/translation" << std::endl;
//   std::cout << "P1" << std::endl << pr1  << std::endl;
//   std::cout << "P2" << std::endl << pr2  << std::endl;
//   std::cout << "P3" << std::endl << pr3  << std::endl;
//   std::cout << "P4" << std::endl << pr4  << std::endl;
//   std::cout << "P5" << std::endl << pr5  << std::endl;
//   std::cout << "P6" << std::endl << pr6  << std::endl;
//   std::cout << "P7" << std::endl << pr7  << std::endl;
//   std::cout << "P8" << std::endl << pr8  << std::endl;

//   //Project onto flat 2D space (in this case, x-z of the camera view)
//   std::vector<Point2D> points;
//   points.push_back({pr1[0], pr1[2]});
//   points.push_back({pr2[0], pr2[2]});
//   points.push_back({pr3[0], pr3[2]});
//   points.push_back({pr4[0], pr4[2]});
//   points.push_back({pr5[0], pr5[2]});
//   points.push_back({pr6[0], pr6[2]});
//   points.push_back({pr7[0], pr7[2]});
//   points.push_back({pr8[0], pr8[2]});
//   std::cout << "Input points are: " << std::endl;
//   std::for_each(points.begin(), points.end(),
//                 [](Point2D point) {std::cout << point.x << ", " << point.y << std::endl;});
  
//   std::vector<Point2D> resultLocal = testrobots::findConvexHull(points);
//   std::cout << "Boundary points of convex hull (local) are: "<<endl;
//   std::for_each(resultLocal.begin(), resultLocal.end(),
//                 [](Point2D point) {std::cout << point.x << ", " << point.y << std::endl;});

//   //Transform these into global coordinates
//   std::vector<Point2D> resultGlobal;
//   for(auto pt : resultLocal) {
//     //Based on where the camera is and where it is looking. Note that camera local coords are:
//     //  X = perpendicular to the right, Y = in direction of view (out the front)
//     double delta_x =        sin(camera_theta) * pt.x + cos(camera_theta) * pt.y;
//     double delta_y = -1.0 * cos(camera_theta) * pt.x + sin(camera_theta) * pt.y;
//     resultGlobal.push_back(Point2D{camera_x + delta_x, camera_y + delta_y});
//   }
//   assert(resultLocal.size() == resultGlobal.size());
//   std::cout << "Boundary points of convex hull (global) are: "<<endl;
//   std::for_each(resultGlobal.begin(), resultGlobal.end(),
//                 [](Point2D point) {std::cout << point.x << ", " << point.y << std::endl;});

//   //Just for visulizing / debugging
//   if(visualizeBB) {
//     // This viewer has 4 windows, but is only showing images in one of them as written here.
//     int argc = 1;
//     char** argv;
//     pcl::visualization::PCLVisualizer *visu = new pcl::visualization::PCLVisualizer(argc, argv, "PlyViewer");
//     int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
//     //visu->createViewPort(0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
//     //visu->createViewPort(0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
//     //visu->createViewPort(0.0, 0, 0.5, 0.5,  mesh_vp_3);
//     //visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
//     //visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);
    
//     visu->createViewPort(0.0, 0.0, 2.0, 2.0, mesh_vp_3);
//     visu->addPointCloud(cloudSegmented, "bboxedCloud", mesh_vp_3);
//     visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
    
//     while(!visu->wasStopped())
//     {
//       visu->spinOnce();
//     }
//   }

//   return resultGlobal;
// }


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe(PCL_TOPIC, 10, cloud_cb);

  ros::Subscriber BBsub = n.subscribe("/BBox", 10, BBoxCallback);

  ros::Subscriber current_pose = n.subscribe("/amcl_pose", 10, poseCallback);

  ros::spin();

  return 0;
}
