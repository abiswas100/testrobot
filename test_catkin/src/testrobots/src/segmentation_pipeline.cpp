/*
  The objDetectionCallback and detectionImageCallback has been used from inventoryClerk file
  The other functions are taken from Segmentation Pipeline function

  How are we structuring this file 
  1. We have a main function that subscribes to Bounding box message and Pointcloud data raw image from ROS
  2. We send the bounding Box info to - ObjDetectionCallback
  3. we send the pointcloud data to - pointcloudcallback
  4. We send the image data to - Imagecallback
  5. we have function named Do plane Extraction which extracts planes from a given pointcloud 
  6. we have another function named Extract Object In Bounding Box that will do the bounding box segmentation from the pointcloud 

  $ How is the code gonna function ?? $

  1. The subscribers get the data from the different topics and save them in the global variables namely (we will add it in sometime)
  2. Then, the bounding box call back calls the plane extraction and extract BB function which call these global variables and does the thing they are supposed to do 
  3. in the future we are going to add Bounding Box to world coordinate function and complete the work in this file
  4. We have added information and function definitions from the InventoryClerk and Segmentation_Pipeline header files just to reduce confusion as he are changing   
    a lot about the functions and what they are doing in this file.
*/


// #include "inventoryClerk.h"
#include "segmentation_pipeline.h"
#include "pclUtils.h"
#include "gaussKernel.h"
#include "convexHull.h"
// #include "cvUtils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> 

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

#include <testrobots/BoundingBoxes.h> // add a header file for the message or it will error 
#include <testrobots/BoundingBox.h>

// added from Inventory Clerk header file

std::ofstream m_out;         //Stream for distances datalog
std::string m_workingPath;   //Path for saving inventory file and images
std::string m_imageTopic;    //The topic used for reading images to process/pass to Yolo
bool m_pause=true;                //Only record items when this is false
bool m_YOLO_imageReceived=true;   //Set to true whenever a Yolo image is received during an unpaused period
bool m_currentlyProcessingObject; //Set to true when we are processing a Yolo-recognized object
                                    //so that we allow the image processing to finish before going on.
bool m_useDownSampling;      //Whether to use a downsampling filter on the cup
// Pose m_currentPose;          //Store current pose corresponding to the "latest" images/clouds below

//The latest image/cloud messages
sensor_msgs::Image m_latestRGBImage;
sensor_msgs::PointCloud2 m_latestPointCloud;




struct Pose 
{
  double x;
  double y;
  double yaw;
};    

struct BoundingBox
{
  unsigned xmin;
  unsigned xmax;
  unsigned ymin;
  unsigned ymax;
};

enum class Normal
{
  eX = 0,
  eY,
  eZ
};
const Normal normal = Normal::eY;
std::string timestamp()

  {
    std::time_t now_time_t = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
  }
std::vector<std::string> parseCL(int argc, char** argv)
{
  return std::vector<std::string>(argv+1, argv + argc);
}

// The following are added from Segmentation pipeline.cpp file 

// Segmentation pipeline / object extraction parameters
const double normalThreshold = 0.97;
// normal = Normal::eY;  // @Apala - can we remove the UNL_robotics thing - what is it for ?? // error in this line -- need to call the enum class
const double cropPercentage = 0.0;  // 0.00  to  0.20
const double meanK = 50.0;          // 50.0  to  100.0
const double stddevMulThresh = 0.5; // 0.5  to    1.0

// Camera specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(1920);   // Avhishek - has been updated as image sizes in respect to our Simulation Camera
const unsigned CAMERA_NUM_PIXELS_HEIGHT(1080);
const double CAMERA_HORIZONTAL_VIEW_ANGLE(1.19); // The angle (in radians) of what the camera views , Avhishek - which is an Astra camera 

// For Gaussian convolution filter
const unsigned KERNEL_RADIUS(5);                            // The Gaussian convolution radius
const unsigned NUM_GAUSS_SAMPLE_PTS(KERNEL_RADIUS * 2 + 1); // How many sample points for depth sampling

// PCL plane extraction - a hard threshold, especially for if something goes wrong or lots of small (insignificant) planes
const unsigned MAX_PLANE_COUNT(8);
const double PLANE_EXTRACTION_CONTINUE_THRESHOLD(0.30); // While 30% of the original cloud is still there

// Euclidean clustering
double CLUSTER_TOLERANCE(0.10);
unsigned MIN_CLUSTER_SIZE(10);

pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr m_postPlaneExtractedCloud;//added check

//Convenience function to print the pipline step count formatted to 2 digits, padding from the front with a 0 if necessary
std::string printStepCount(); // const;
std::string printStepCount(unsigned addition); //const;

pcl::PCDWriter m_writer;
unsigned m_pipelineStepCount;  //What step are we in the pipeline. Major steps by 10, minor by 1
std::string m_baseName;      //The base file name we will append and save to
BoundingBox m_boundingBox;
std::string timeStamp = timestamp();


//apala: added declarations from segmentation_pipeline.h
void doPlaneExtraction(Normal, double minThreshold);
void doPlaneExtraction(Normal, double minThreshold, pcl::PointCloud<pcl::PointXYZ>::Ptr destination);
void extractObjectInBoundingBox(double cropPercentage); 
void extractObjectInBoundingBox(double cropPercentage, pcl::PointCloud<pcl::PointXYZ>::Ptr destination);

UNL_Robotics::SegmentationPipeline::SegmentationPipeline(const std::string& baseName,
                                                         const BoundingBox& yoloIdentifiedBoundingBox,
                                                         pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloud)
  : m_pipelineStepCount(0),
    m_baseName(baseName),
    m_boundingBox(yoloIdentifiedBoundingBox),
    m_cloud(pointCloud),
    m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

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

/*
  Avhishek - callback for object detection Boundary Box. We have to clean this function before we can do anything 
             it has a lot of UNL robotics related stuff that we don't need
*/ 

//Saves only the cropped images as a jpeg file
void cropAndSaveImage(const sensor_msgs::Image& msg,
                                    const std::string& croppedImagePath,
                                    unsigned xmin, unsigned ymin,
                                    unsigned x_delta, unsigned y_delta)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  { 
    //Covert the image from a ROS image message to a CV image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    //Crop out the sub image, given the coordinates
    cv::Rect2d cropRect(xmin, ymin, x_delta, y_delta);
    cv::Mat croppedImage = cv_ptr->image(cropRect);
    // imshow("blah",croppedImage);
    imwrite(croppedImagePath, croppedImage);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("cv_bridge exception. What = " << e.what());
    return;
  }

}


// *********************************************************************************************************************************
void objDetectionCallback(const testrobots::Boundingbox::ConstPtr& msg)  // check this line for any change needed for custom C++ messages
{

  ROS_INFO_STREAM("segmentation pipeline - object detected, in callback");

  //Set this flag to true to keep main thread from exiting before we are done processing
  // this object. Otherwise, the long-term classification might not be set yet and the
  // thread would continue prematurely
  m_currentlyProcessingObject = true;  // comes from header file definitions

  // ******************************************************************************************************************

  //Iterate over all the items that have been identified

  unsigned itemNum = 0; //apala: 1 for human only
  
 
//*************************************************
  //Get out the object type and the bounding box information
  
  std::string objectName = msg->Class.c_str();
  unsigned xmin = msg->xmin;
  unsigned xmax = msg->xmax;
  unsigned ymin = msg->ymin;
  unsigned ymax = msg->ymax;
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


//   // Avhishek - 
//   //Convert the most recent ROS point cloud msg into a pcl::PointCloud
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::fromROSMsg(m_latestPointCloud, *pclCloud);



 //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ Added from inspect position function of inventory clerk $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

  //Capture and store the latest depth image, RGB image, and point-cloud.
  //This is done here once, and for the remainder of this loop, all 3 of those
  // images will be used together. This guarantees that the images being processed
  // by Yolo match the depth images being used for depth information.
  //The RGB image will be continuously broadcast specially for Yolo consumption
  
  ros::Duration maxWait(10.0); //Wait up to this amount of time for images/point cloud
   
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(m_latestPointCloud, *pclCloud);
  //raw image 
  boost::shared_ptr<sensor_msgs::Image const> rawImagePtr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_raw", maxWait);
  if(rawImagePtr == NULL)
    ROS_ERROR_STREAM("No raw image messages received on " << m_imageTopic);
    

  else
    m_latestRGBImage = *rawImagePtr;

  //pointcloud image
  boost::shared_ptr<sensor_msgs::PointCloud2 const> pointCloudPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points", maxWait);
  if(pointCloudPtr == NULL){  

    ROS_ERROR_STREAM("No point clound messages received on " << "/camera/depth/points");}

  else
    m_latestPointCloud = *pointCloudPtr;
       


  ROS_INFO_STREAM("Most recent images captured");

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$



  //Assert a few things about our point cloud and RGB images. They should match sizes.
  //If not, our bounding box values will not correspond
  // Avhishek - 
  
  ROS_INFO_STREAM("    " << "Latest Image Height and Width = (" << m_latestRGBImage.height << ", " << m_latestRGBImage.width << ")" );
  ROS_INFO_STREAM("    " << "Latest PCL Height and Width = (" << m_latestPointCloud.height << ", " << m_latestPointCloud.width << ")" );
  
  ROS_ASSERT(m_latestRGBImage.height ==  m_latestPointCloud.height); //pclCloud
  ROS_ASSERT(m_latestRGBImage.width  ==  m_latestPointCloud.width);
  

  /*
    Avhishek - Doing long term segmentation for only person
  */

  //Do long-term object classification for only person class 
  if(objectName == "person"){
    ROS_INFO_STREAM("BLAH");  
    // //Save the RGB image of the object as a jpeg file and the point-cloud as a PCD
    // //First create the path based on the object name and its number in this series of bounding boxes
    // // (in the case that there is more than 1 object of the same type in the image, this will be unique)
    std::stringstream ssObjName, ssObjPath;
    ssObjName << "item_" << itemNum << "_obj_" << objectName;
    
    // //ssObjPath << m_workingPath << timeStamp << "_" << ssObjName.str();// setting our own working path
  
    // // Call the crop and save function. Save only the object in this loop //UNL_Robotics::
    ssObjPath << ssObjName.str();
    // cropAndSaveImage(m_latestRGBImage, ssObjPath.str() + ".jpeg", 
    //                   xmin, ymin, x_delta, y_delta);
   
  
    //Save the full room point cloud
    // std::stringstream ssPCD;
    //  //ssPCD << m_workingPath << timeStamp << "_fullPointCloud.pcd"; // we are adding our own m_workingpath
    // // ssPCD << "home/apramani/testrobot/test_catkin/src/testrobots/src" << timeStamp << "_fullPointCloud.pcd";
    // ssPCD << "fullPointCloud.pcd";
    // pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZ>(ssPCD.str(), *pclCloud, false);

    BoundingBox boundingBox{xmin, xmax, ymin, ymax};
    // std::vector<UNL_Robotics::Point2D> hullPoints;  

    // Extract the object PCL knowing the bounding box and get it converter to map
    
      // UNL_Robotics::SegmentationPipeline segmenter(ssObjPath.str(), boundingBox, *pclCloud);
      // segmenter.doPlaneExtraction(normal, normalThreshold);
      doPlaneExtraction(normal, normalThreshold,pclCloud);
    // below this will be needed for mapping
    
    //   segmenter.extractObjectInBoundingBox(cropPercentage);
    //   segmenter.removeOutliers(meanK, stddevMulThresh);
    //   segmenter.performEuclideanExtraction();
    //   bool visualizeBB = false;
    //   hullPoints = segmenter.calcBoundingBoxInWorldCoords(visualizeBB,
    //                                                       m_currentPose.x,
    //                                                       m_currentPose.y,
    //                                                       m_currentPose.yaw);
    // }
    // LongTermObject lto {objectName, hullPoints};
    // m_longTermObjects.push_back(lto);

 
  //We're now finished processing this object, so set the processing flag to false
  //This is checked in the main thread to ensure we're not moving on before this is done.
  m_currentlyProcessingObject = false;
    
  ROS_INFO_STREAM("finished processing object");
  }
 }



/* Avhishek - Variable Description for doPlaneExtraction
  
  Inputs ----------------------------------------- 
  normal - given 

  minThreshold- given 

  destination - *pcloud recieved in Objdetection

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

void doPlaneExtraction(Normal normal, double minThreshold, pcl::PointCloud<pcl::PointXYZ>::Ptr destination)
{
  // Plane extraction
  //
    //ros::Duration maxWait(10.0);
    //pcl::fromROSMsg(m_latestPointCloud, *destination);
  //m_cloud = m_latestPointCloud;//ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points", maxWait);
  //pcl::PointCloud <pcl::PointXYZ>::Ptr
  ROS_INFO_STREAM("getting here 1");
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_planeless_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*m_cloud, *final_planeless_cloud);
  ROS_INFO_STREAM("getting here 2");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_plane_extraction(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*m_cloud, *cloud_for_plane_extraction);
  ROS_INFO_STREAM("getting here 3");
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

  // Create the extracting object (extraction based on indices)
  pcl::ExtractIndices<pcl::PointXYZ> extract;
    
  //Keep the removed points as NaN values to maintain the structure of the cloud
  extract.setKeepOrganized(true);
    
  unsigned planeCount =0;
  const unsigned numOriginalPoints = cloud_for_plane_extraction->size();   //How many points we originally started with
  unsigned numPointsExtracted =0;   //Keep track of how many points have so far been extracted

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
    ss << m_baseName << "_step" << printStepCount() << "_plane_" << planeCount << ".pcd";
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
      ss << m_baseName << "_step" << printStepCount(1) << "_floorless_cloud.pcd";
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
  
  std::stringstream ss;
  ss << m_baseName << "_step" << printStepCount(2) << "_postPlaneExtraction.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *final_planeless_cloud, false);

  //std::cout << "Min-max after extracting plane:" << std::endl;
  //printMinMax(final_planeless_cloud);
  
  //Copy this into the destination cloud
  copyPointCloud(*final_planeless_cloud, *destination);

  //Also, cache a copy of teh post-extracted plane, so we can restart from this point
  // again in the future
  copyPointCloud(*final_planeless_cloud, *m_postPlaneExtractedCloud);
  std::cout << "     " << "Post-plane extraction cloud cached for possible re-use again" << std::endl;
  
  m_pipelineStepCount += 10;

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "segmentationPCL");
  ROS_INFO("Initializiing Segmentation of PCL");
  ros::NodeHandle nodeHandle;

  // why is this line used -
  std::vector<std::string> args = parseCL(argc, argv);
  
  int QUEUE = 10;
  // creating threads for computation
  uint32_t numThreads(4);
  ros::AsyncSpinner spinner(numThreads);
  spinner.start();

  // Do subscriptions here
  ros::Subscriber m_objectDetectionSubscriber = nodeHandle.subscribe("/BBox", QUEUE, objDetectionCallback); //bbcord.msg
  
  // Rather than ros::spin(), use waitForShutdown() with the async spinner
  ros::waitForShutdown();

  ROS_INFO("Done will PCL segmentation. Good Luck!!");
  return EXIT_SUCCESS;

  return 1;
}

