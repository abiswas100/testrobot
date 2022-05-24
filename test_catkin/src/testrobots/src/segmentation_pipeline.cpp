
#include "inventoryClerk.h"
#include "segmentation_pipeline.h"
#include "pclUtils.h"
#include "cvUtils.h"
#include "gaussKernel.h"
#include "convexHull.h"

// ROS Topics
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h>  

// OpenCV
#include <cv_bridge/cv_bridge.h>
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

#include <testrobots/BoundingBox.h> // add a header file for the message or it will error 


// added from Inventory Clerk header file

std::ofstream m_out;         //Stream for distances datalog
std::string m_workingPath;   //Path for saving inventory file and images
std::string m_imageTopic;    //The topic used for reading images to process/pass to Yolo
bool m_pause;                //Only record items when this is false
bool m_YOLO_imageReceived;   //Set to true whenever a Yolo image is received during an unpaused period
bool m_currentlyProcessingObject; //Set to true when we are processing a Yolo-recognized object
                                    //so that we allow the image processing to finish before going on.
bool m_useDownSampling;      //Whether to use a downsampling filter on the cup
Pose m_currentPose;          //Store current pose corresponding to the "latest" images/clouds below

//The latest image/cloud messages
sensor_msgs::Image m_latestRGBImage;
sensor_msgs::PointCloud2 m_latestPointCloud;

  struct Pose {
  double x;
  double y;
  double yaw;
};    

// added from segmentation pipeline header file
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

// callback for object detection 
void objDetectionCallback(const testrobots::BoundingBoxes::ConstPtr& msg)  // check this line for any change needed for custom C++ messages
{
  ROS_INFO_STREAM("segmentation pipeline - object detected, in callback");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject); 

  //Set this flag to true to keep main thread from exiting before we are done processing
  // this object. Otherwise, the long-term classification might not be set yet and the
  // thread would continue prematurely
  m_currentlyProcessingObject = true;  // comes from header file definitions
  

  // waits for the image to be recieved if no image them returns or else continues
  if(m_pause) { 
    ROS_INFO_STREAM("InventoryClerk - currently paused, so abandoning callback");
    ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
    return;
  }

  ROS_INFO_STREAM("InventoryClerk - not paused so continuing with callback");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);



  //Iterate over all the items that have been identified
  unsigned itemNum = 0;
  // for(auto box : msg->bounding_boxes) {  commenting for loop
//*************************************************
  //Get out the object type and the bounding box information
  
  std::string objectName = box.Class;
  unsigned xmin = box.xmin;
  unsigned xmax = box.xmax;
  unsigned ymin = box.ymin;
  unsigned ymax = box.ymax;
  unsigned x_delta = xmax - xmin;
  unsigned y_delta = ymax - ymin; 
  ROS_INFO_STREAM("  " << objectName  << "  -  Probability " << std::setprecision(4) << (box.probability*100) << "%" ); // not needed 
  ROS_INFO_STREAM("    " << "BB Min (x,y) = (" << xmin << ", " << ymin << ")" );
  ROS_INFO_STREAM("    " << "BB Max (x,y) = (" << xmax << ", " << ymax << ")" );
  // not needed ************
  m_out << "*) Object type:  " << objectName << std::endl;
  m_out << "   Probability  " << std::setprecision(4) << (box.probability*100.0) << "%" << std::endl;
//*******************

  // Avhishek - Don't know why  this is being done what is the use of calculating objectAngleOffset

  //Calculate the angle offset of the picture relative to the center of the view port
  unsigned x_centerBB = xmin + static_cast<unsigned>(x_delta/2);
  unsigned y_centerBB = ymin + static_cast<unsigned>(y_delta/2);
  int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH/2) - x_centerBB;   //Can be negative! This orientation assumes CCW=+
  double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

  // Avhishek - m_out is being used for printing 
  m_out << "   " << "Bounding Box (x,y):"
        << "   Min = (" << xmin << ", " << ymin << ")"
        << "   Max = (" << xmax << ", " << ymax << ")"
        << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
  m_out << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;


  // Avhishek - 
  //Convert the most recent ROS point cloud msg into a pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(m_latestPointCloud, *pclCloud);

  //Assert a few things about our point cloud and RGB images. They should match sizes.
  //If not, our bounding box values will not correspond
  // Avhishek - 
  ROS_ASSERT(m_latestRGBImage.height ==  pclCloud->height);
  ROS_ASSERT(m_latestRGBImage.width  ==  pclCloud->width);


  //Do long-term object classification
  if((objectName == "sofa") || (objectName == "bench") || (objectName == "door")) {
    
    m_longTermObjectDetectedAtThisPosition = true;
  
    //Save the RGB image of the object as a jpeg file and the point-cloud as a PCD
    //First create the path based on the object name and its number in this series of bounding boxes
    // (in the case that there is more than 1 object of the same type in the image, this will be unique)
    std::stringstream ssObjName, ssObjPath;
    ssObjName << "item_" << itemNum << "_obj_" << objectName;
    ssObjPath << m_workingPath << timeStamp << "_" << ssObjName.str();
  
    //Call the crop and save function. Save only the object in this loop
    cropAndSaveImage(m_latestRGBImage, ssObjPath.str() + ".jpeg",
                      xmin, ymin, x_delta, y_delta);
  
    //Save the full room point cloud
    std::stringstream ssPCD;
    ssPCD << m_workingPath << timeStamp << "_fullPointCloud.pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>(ssPCD.str(), *pclCloud, false);

    BoundingBox boundingBox{xmin, xmax, ymin, ymax};
    std::vector<UNL_Robotics::Point2D> hullPoints;
    if(objectName == "door") {

      
      //Note: In order for doors to be recognized, you  **MUST**  use a custom version
      //      of Yolo that is trained for doors.  If this is not installed, door
      //      recognition will not work.

      
      //DoorSegmentation segmenter(ssObjPath.str(), boundingBox, pclCloud);
      //segmenter.doPlaneExtraction(normal, normalThreshold);
      //segmenter.extractObjectInBoundingBox(cropPercentage);
      //segmenter.removeOutliers(meanK, stddevMulThresh);
      //segmenter.performEuclideanExtraction();
      //hullPoints = segmenter.calcBoundingBoxInWorldCoords(m_currentPose.x,
      //                                                    m_currentPose.y,
      //                                                    m_currentPose.yaw);
    }
    else {
      //Extract the object PCL knowing the bounding box
      SegmentationPipeline segmenter(ssObjPath.str(), boundingBox, pclCloud);
      segmenter.doPlaneExtraction(normal, normalThreshold);
      segmenter.extractObjectInBoundingBox(cropPercentage);
      segmenter.removeOutliers(meanK, stddevMulThresh);
      segmenter.performEuclideanExtraction();
      bool visualizeBB = false;
      hullPoints = segmenter.calcBoundingBoxInWorldCoords(visualizeBB,
                                                          m_currentPose.x,
                                                          m_currentPose.y,
                                                          m_currentPose.yaw);
    }
    LongTermObject lto {objectName, hullPoints};
    m_longTermObjects.push_back(lto);
  }
  
  //Increment the item number not needed because only 1 item
  // ++itemNum;    
  // } commenting out the bracket for the for loop
 
  //We're now finished processing this object, so set the processing flag to false
  //This is checked in the main thread to ensure we're not moving on before this is done.
  m_currentlyProcessingObject = false;
  
  ROS_INFO_STREAM("InventoryClerk - finished processing object");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);
}




/*
  Avhishek - 

*/

void UNL_Robotics::InventoryClerk::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO_STREAM("InventoryClerk - detection image callback called");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);

  if(m_pause) {
    ROS_INFO_STREAM("InventoryClerk - currently paused. Returning");
    return;
  }

  //Save off the full image for this observation
  std::string timeStamp = timestamp();
  std::stringstream ssFullPath;
  ssFullPath << m_workingPath << timeStamp << "_fullImage.jpeg";
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(m_latestRGBImage, sensor_msgs::image_encodings::BGR8);
    imwrite(ssFullPath.str(), cv_ptr->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("cv_bridge exception. What = " << e.what());
    return;
  }
  ROS_INFO_STREAM("Saved full image to file: " << ssFullPath.str());

  //And wait until any object processing (CV) has finished
  ros::Rate loop_rate(10);
  while(m_currentlyProcessingObject)    //Until object being processed is finished
  {
    loop_rate.sleep();
  }
 
  //Set the yolo image received to indicate we just received an image during an un-paused period.
  //As soon as this is set to true, the main thread will continue
  ROS_INFO_STREAM("InventoryClerk - finished detection image callback. Setting image received flag to true");
  m_YOLO_imageReceived = true;
  
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);
}



// Segmentation pipeline / object extraction parameters
const double normalThreshold = 0.97;
const UNL_Robotics::Normal normal = UNL_Robotics::Normal::eY;
const double cropPercentage = 0.0;  // 0.00  to  0.20
const double meanK = 50.0;          // 50.0  to  100.0
const double stddevMulThresh = 0.5; // 0.5  to    1.0

// Camera specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(1920);
const unsigned CAMERA_NUM_PIXELS_HEIGHT(1080);
const double CAMERA_HORIZONTAL_VIEW_ANGLE(1.19); // The angle (in radians) of what the camera views

// For Gaussian convolution filter
const unsigned KERNEL_RADIUS(5);                            // The Gaussian convolution radius
const unsigned NUM_GAUSS_SAMPLE_PTS(KERNEL_RADIUS * 2 + 1); // How many sample points for depth sampling

// PCL plane extraction - a hard threshold, especially for if something goes wrong or lots of small (insignificant) planes
const unsigned MAX_PLANE_COUNT(8);
const double PLANE_EXTRACTION_CONTINUE_THRESHOLD(0.30); // While 30% of the original cloud is still there

// Euclidean clustering
double CLUSTER_TOLERANCE(0.10);
unsigned MIN_CLUSTER_SIZE(10);

// UNL_Robotics::SegmentationPipeline::SegmentationPipeline(pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloud)
//   : m_pipelineStepCount(0),
//     m_cloud(pointCloud),
//     m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>)
// {
// }

UNL_Robotics::SegmentationPipeline::SegmentationPipeline(const std::string &baseName,
                                                         const BoundingBox &yoloIdentifiedBoundingBox,
                                                         pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
    : m_pipelineStepCount(0),
      m_baseName(baseName),
      m_boundingBox(yoloIdentifiedBoundingBox),
      m_cloud(pointCloud),
      m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

// we don't read from pcd
// UNL_Robotics::SegmentationPipeline::SegmentationPipeline(const std::string& baseName,
//                                                          const BoundingBox& yoloIdentifiedBoundingBox,
//                                                          std::string pcdFilepath)
//   : m_pipelineStepCount(0),
//     m_baseName(baseName),
//     m_boundingBox(yoloIdentifiedBoundingBox),
//     m_cloud(new pcl::PointCloud<pcl::PointXYZ>),
//     m_postPlaneExtractedCloud(new pcl::PointCloud<pcl::PointXYZ>)

// {
//   //Read in the pointcloud from file
//   pcl::PCDReader reader;
//   reader.read(pcdFilepath, *m_cloud);
// }

void UNL_Robotics::SegmentationPipeline::resetCloudToPostPlaneExtraction()
{
  copyPointCloud(*m_postPlaneExtractedCloud, *m_cloud);
  m_pipelineStepCount = 10; // Set this to 10, so that step numbering remaining sequential and consistent
  std::cout << "     "
            << "Post-plane extraction cloud reset to use as start cloud" << std::endl;
}

void UNL_Robotics::SegmentationPipeline::resetCloudToPostPlaneExtractionAndBB(const std::string &newBaseName,
                                                                              const BoundingBox &newBoundingBox)
{
  setBaseName(newBaseName);
  setBoundingBox(newBoundingBox);
  resetCloudToPostPlaneExtraction();
}

// why
double UNL_Robotics::SegmentationPipeline::calculateObjectAngleOffset() const
{
  unsigned x_delta = m_boundingBox.xmax - m_boundingBox.xmin;
  unsigned y_delta = m_boundingBox.ymax - m_boundingBox.ymin;

  // Calculate the angle offset of the picture relative to the center of the view port
  unsigned x_centerBB = m_boundingBox.xmin + static_cast<unsigned>(x_delta / 2);
  unsigned y_centerBB = m_boundingBox.ymin + static_cast<unsigned>(y_delta / 2);
  int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH / 2) - x_centerBB; // Can be negative! This orientation assumes CCW=+
  double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

  std::cout << "   "
            << "Bounding Box (x,y):"
            << "   Min = (" << m_boundingBox.xmin << ", " << m_boundingBox.ymin << ")"
            << "   Max = (" << m_boundingBox.xmax << ", " << m_boundingBox.ymax << ")"
            << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
  std::cout << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;

  return objectAngleOffset;
}

//**********************************************************************************************************************************

// double UNL_Robotics::SegmentationPipeline::calculateDepths() const  //of despair
// {
//   unsigned x_delta = m_boundingBox.xmax - m_boundingBox.xmin;
//   unsigned y_delta = m_boundingBox.ymax - m_boundingBox.ymin;

//   //Get the Gaussian kernels for gaussian sampling
//   UNL_Robotics::kernel_type gaussianKernel1 = UNL_Robotics::produce2dGaussianKernel(KERNEL_RADIUS);  //Default mu/sigma values
//   double mu = KERNEL_RADIUS;
//   double sigma = 0.84089652;
//   UNL_Robotics::kernel_type gaussianKernel2 = UNL_Robotics::produce2dGaussianKernel(KERNEL_RADIUS, mu, sigma);  //Tighter Gaussian curve

//   //Sampling

//   //Get all the remaining point-cloud depths
//   std::vector<double> PCDepths;
//   for(unsigned i=0; i< m_cloud->size(); ++i)
//     PCDepths.push_back(m_cloud->operator[](i).z);

//   //Apply filter to remove the max and min values (removes n-samples from each side)
//   std::vector<double> PCDepths_filtered = truncate_n_return_middle_vals(PCDepths, NUM_GAUSS_SAMPLE_PTS);
//   std::vector<double> PCDepths_dblFiltered = truncate_n_return_middle_vals(PCDepths, 2*NUM_GAUSS_SAMPLE_PTS);  // 2x number of samples

//   //Raw
//   std::pair<double, double> PCDepthMeanStdev = calcMeanAndStdDev(PCDepths);
//   cout << "   PCloud raw:" <<
//     "   mean = " << PCDepthMeanStdev.first << "," <<
//     "   std. deviation = " << PCDepthMeanStdev.second << std::endl;

//   //Max-min filtered
//   std::pair<double, double> PCDepthMeanStdev_filtered = calcMeanAndStdDev(PCDepths_filtered);
//   cout << "   PCloud filter:" <<
//     "   mean = " << PCDepthMeanStdev_filtered.first << "," <<
//     "   std. deviation = " << PCDepthMeanStdev_filtered.second << std::endl;

//   //Max-min filtered with 2x
//   std::pair<double, double> PCDepthMeanStdev_dblFiltered = calcMeanAndStdDev(PCDepths_dblFiltered);
//   cout << "   PCloud double filter:" <<
//     "   mean = " << PCDepthMeanStdev_dblFiltered.first << "," <<
//     "   std. deviation = " << PCDepthMeanStdev_dblFiltered.second << std::endl;

//   /*
//   //Gauss weight 1
//   cout << "   PCloud Gauss weight1 :" <<
//     "   mean = " << std::accumulate(PCDepths_weighted1.begin(), PCDepths_weighted1.end(), 0.0) << std::endl;

//   //Gauss weight 2
//   cout << "   PCloud Gauss weight2 :" <<
//     "   mean = " << std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0) << std::endl;
//   */

//   //Using the distance and pose, calculate the position of the object
//   //double dist = std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0);
//   //double objYaw = m_currentPose.yaw + calculateObjectAngleOffset();     //The object yaw is adjusted for its position within the image

//   //Pose objectPose = { m_currentPose.x + dist * cos(objYaw),
//   //                    m_currentPose.y + dist * sin(objYaw),
//   //                    0.0 };
//   //cout << "   Object pose (x,y,yaw) = " << objectPose << std::endl;

//   return PCDepthMeanStdev.first;
// }
//***********************************************************************************************************************************************

//********************************************************************************************************************************************
///// the functions below help in calculations in other functions

//////// PRIVATE ///////////
 // remove Nans is used in Eclidean extracion
void UNL_Robotics::SegmentationPipeline::removeNaNs(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest)
{
  // m_cloud->is_dense = false;
  // boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  // pcl::removeNaNFromPointCloud(*m_cloud, *m_cloud, *indices);

  /*
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(m_cloud);
  extract.setIndices(indices);
  extract.setNegative(true);
  extract.filter(*m_cloud);
  */

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = source->begin(); it != source->end(); ++it)
  {

    if (!(std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z)))
    {
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

std::string UNL_Robotics::SegmentationPipeline::printStepCount() const
{
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(2) << m_pipelineStepCount;
  return ss.str();
}

std::string UNL_Robotics::SegmentationPipeline::printStepCount(unsigned addition) const
{
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(2) << (m_pipelineStepCount + addition);
  return ss.str();
}

void UNL_Robotics::SegmentationPipeline::printMinMax()
{
  printMinMax(m_cloud);
}

void UNL_Robotics::SegmentationPipeline::printMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PointXYZ minPoint;
  pcl::PointXYZ maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);

  std::cout << std::endl;
  std::cout << "getMinMax3D()" << std::endl;
  std::cout << "X:  " << minPoint.x << ", " << maxPoint.x << std::endl;
  std::cout << "Y:  " << minPoint.y << ", " << maxPoint.y << std::endl;
  std::cout << "Z:  " << minPoint.z << ", " << maxPoint.z << std::endl;
  std::cout << std::endl;
}


// the functions below help in doing segmentation of PCL and extracting BB
//*********************************************************************************************************************************************
void UNL_Robotics::SegmentationPipeline::doPlaneExtraction(Normal normal, double minThreshold)
{
  doPlaneExtraction(normal, minThreshold, m_cloud);
}

void UNL_Robotics::SegmentationPipeline::doPlaneExtraction(Normal normal, double minThreshold, pcl::PointCloud<pcl::PointXYZ>::Ptr destination)
{
  // Plane extraction
  //
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
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  // Create the extracting object (extraction based on indices)
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Keep the removed points as NaN values to maintain the structure of the cloud
  extract.setKeepOrganized(true);

  unsigned planeCount = 0;
  const unsigned numOriginalPoints = cloud_for_plane_extraction->size(); // How many points we originally started with
  unsigned numPointsExtracted = 0;                                       // Keep track of how many points have so far been extracted

  // Do until we extract a certain percentage of the points -or-
  //  until the maximum number of planes is met, which is checked at the end of this loop
  while (numPointsExtracted < numOriginalPoints * (1.0 - PLANE_EXTRACTION_CONTINUE_THRESHOLD))
  {

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_for_plane_extraction);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Accumulate the number of points extracted for this plane
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
    // Calculate the plane normals - used to determine if the plane is a floor plane

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    // Calculate the least-squares plane value
    float nx, ny, nz, curvature;
    ne.computePointNormal(*cloud_for_plane_extraction, inliers->indices, nx, ny, nz, curvature);

    // Now calc each normal
    ne.setInputCloud(cloud_plane);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals);
    std::cout << "     Least-squares plane fit: (" << nx << ", " << ny << ", " << nz << ")"
              << ", curvature = " << curvature << std::endl;

    bool minThresholdMet = false;
    if ((normal == Normal::eX) && (fabs(nx) > minThreshold))
      minThresholdMet = true;
    if ((normal == Normal::eY) && (fabs(ny) > minThreshold))
      minThresholdMet = true;
    if ((normal == Normal::eZ) && (fabs(nz) > minThreshold))
      minThresholdMet = true;

    if (minThresholdMet)
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

      // Keep the removed points as NaN values to maintain the structure of the cloud
      normalPlaneExtractor.setKeepOrganized(true);

      normalPlaneExtractor.filter(*cloud_minus_plane);

      std::cout << "final_planeless_cloud" << std::endl;
      std::cout << "width = " << final_planeless_cloud->width << std::endl;
      std::cout << "height = " << final_planeless_cloud->height << std::endl;

      std::cout << "cloud_minus_plane" << std::endl;
      std::cout << "width = " << cloud_minus_plane->width << std::endl;
      std::cout << "height = " << cloud_minus_plane->height << std::endl;

      // If this file already exists (i.e., part of the floor was removed in an earlier pass),
      //  it will be over written here, and so just the updated, most recent version (with
      //  the least amount of floor plane) will remain.
      std::stringstream ss;
      ss << m_baseName << "_step" << printStepCount(1) << "_floorless_cloud.pcd";
      m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_minus_plane, false);

      // Make this is our new cloud
      copyPointCloud(*cloud_minus_plane, *final_planeless_cloud);
    }

    // Report the number of points extracted after this plane extraction
    std::cout << "     " << numPointsExtracted << " points extracted so far (of a total "
              << numOriginalPoints << ")." << std::endl
              << std::endl;

    // Remove the plane from our working cloud
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_for_plane_extraction.swap(cloud_f);

    // Increment the current plane count
    ++planeCount;

    // Put in a max number of planes so if something goes wrong, we don't write a billion and 1 files
    if (planeCount >= MAX_PLANE_COUNT)
    {
      std::cout << std::endl
                << "Maximum threshold reached: plane count = " << MAX_PLANE_COUNT << ". Forcing end of plane extraction." << std::endl;
      break;
    }
  }

  // this is to write the pcd
  std::stringstream ss;
  ss << m_baseName << "_step" << printStepCount(2) << "_postPlaneExtraction.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *final_planeless_cloud, false);

  // std::cout << "Min-max after extracting plane:" << std::endl;
  // printMinMax(final_planeless_cloud);

  // Copy this into the destination cloud
  copyPointCloud(*final_planeless_cloud, *destination);

  // Also, cache a copy of teh post-extracted plane, so we can restart from this point
  //  again in the future
  copyPointCloud(*final_planeless_cloud, *m_postPlaneExtractedCloud);
  std::cout << "     "
            << "Post-plane extraction cloud cached for possible re-use again" << std::endl;

  m_pipelineStepCount += 10;
}

void UNL_Robotics::SegmentationPipeline::extractObjectInBoundingBox(double cropPercentage)
{
  extractObjectInBoundingBox(cropPercentage, m_cloud);
}

void UNL_Robotics::SegmentationPipeline::extractObjectInBoundingBox(double cropPercentage, pcl::PointCloud<pcl::PointXYZ>::Ptr destination)
{
  // Extract the object PCL knowing the bounding box values, possibly with an additional cropping border (reduced by the crop percentage)
  unsigned x_delta = m_boundingBox.xmax - m_boundingBox.xmin;
  unsigned y_delta = m_boundingBox.ymax - m_boundingBox.ymin;
  unsigned cloudWidth = m_cloud->width;
  unsigned cloudHeight = m_cloud->height;

  std::cout << "Cloud width = " << cloudWidth << std::endl;
  std::cout << "Cloud height = " << cloudHeight << std::endl;
  std::cout << "Crop percentage = " << (cropPercentage * 100) << "%" << std::endl;
  std::cout << "BB xmin = " << m_boundingBox.xmin << std::endl;
  std::cout << "BB xmax = " << m_boundingBox.xmax << std::endl;
  std::cout << "BB ymin = " << m_boundingBox.ymin << std::endl;
  std::cout << "BB ymax = " << m_boundingBox.ymax << std::endl;
  std::cout << "BB xmin, cropped = " << m_boundingBox.xmin + static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
  std::cout << "BB xmax, cropped = " << m_boundingBox.xmax - static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
  std::cout << "BB ymin, cropped = " << m_boundingBox.ymin + static_cast<unsigned>(y_delta * cropPercentage) << std::endl;
  std::cout << "BB ymax, cropped = " << m_boundingBox.ymax - static_cast<unsigned>(y_delta * cropPercentage) << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrop(new pcl::PointCloud<pcl::PointXYZ>);
  // If we have an unstructured cloud (for example, coming from the Gazebo depth cloud), then use the following version of the function
  if (cloudHeight == 1)
  {
    UNL_Robotics::extractFrame<pcl::PointXYZ>(m_cloud, cloudCrop,
                                              m_boundingBox.xmin + static_cast<unsigned>(x_delta * cropPercentage),
                                              m_boundingBox.xmax - static_cast<unsigned>(x_delta * cropPercentage),
                                              m_boundingBox.ymin + static_cast<unsigned>(y_delta * cropPercentage),
                                              m_boundingBox.ymax - static_cast<unsigned>(y_delta * cropPercentage),
                                              CAMERA_NUM_PIXELS_WIDTH, CAMERA_NUM_PIXELS_HEIGHT);
  }
  else
  {
    // Otherwise we have an organized cloud, so use this version
    UNL_Robotics::extractFrame<pcl::PointXYZ>(m_cloud, cloudCrop,
                                              m_boundingBox.xmin + static_cast<unsigned>(x_delta * cropPercentage),
                                              m_boundingBox.xmax - static_cast<unsigned>(x_delta * cropPercentage),
                                              m_boundingBox.ymin + static_cast<unsigned>(y_delta * cropPercentage),
                                              m_boundingBox.ymax - static_cast<unsigned>(y_delta * cropPercentage));
  }
  std::stringstream ss;
  ss << m_baseName << "_step" << printStepCount() << "_extractBBcrop" << std::setprecision(2) << std::fixed << cropPercentage << ".pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloudCrop, false);

  // Copy this into the destination cloud
  copyPointCloud(*cloudCrop, *destination);

  m_pipelineStepCount += 10;
}

//**************not needed right now**********************************************************************************************************
void UNL_Robotics::SegmentationPipeline::removeOutliers(double meanK, double stddevMulThresh)
{
  // Remove outliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removedOutliers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(m_cloud);
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stddevMulThresh);
  sor.filter(*cloud_removedOutliers);

  std::stringstream ss;
  ss << m_baseName << "_step" << printStepCount() << "_removedOutliers.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_removedOutliers, false);

  // Make this is our new working cloud
  copyPointCloud(*cloud_removedOutliers, *m_cloud);

  m_pipelineStepCount += 10;
}

void UNL_Robotics::SegmentationPipeline::performEuclideanExtraction()
{
  // Euclidean Cluster Extraction

  // Start by removing NaNs, if they exist
  pcl::PointCloud<pcl::PointXYZ>::Ptr nanlessCloud(new pcl::PointCloud<pcl::PointXYZ>);
  removeNaNs(m_cloud, nanlessCloud);
  copyPointCloud(*nanlessCloud, *m_cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(m_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  std::cout << "Cluster tolearance set to: " << CLUSTER_TOLERANCE << std::endl;
  ec.setClusterTolerance(CLUSTER_TOLERANCE);

  std::cout << "Minimum cluster size set to: " << MIN_CLUSTER_SIZE << std::endl;
  ec.setMinClusterSize(MIN_CLUSTER_SIZE);

  // This is set based on the bounding box size
  unsigned maxClusterSize = (m_boundingBox.xmax - m_boundingBox.xmin) * (m_boundingBox.ymax - m_boundingBox.ymin);
  ec.setMaxClusterSize(maxClusterSize);
  std::cout << "Maximum cluster size set to: " << maxClusterSize << std::endl;
  ec.setSearchMethod(tree);
  std::cout << "Set search method to tree" << std::endl;
  m_cloud->is_dense = false;
  ec.setInputCloud(m_cloud);
  std::cout << "Set input cloud" << std::endl;
  ec.extract(cluster_indices);
  std::cout << "   " << cluster_indices.size() << " cluster" << (cluster_indices.size() != 1 ? "s" : "")
            << " found in pointcloud." << std::endl;

  // For the file name, set the cluster number width (so that numbers are formatted as  07  if there are more than 10)
  unsigned clusterNumberWidth = floor(log10(cluster_indices.size())) + 1;

  // Loop over all the clusters found
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->push_back((*m_cloud)[*pit]);
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
    std::cout << "    - Cluster extracted with " << cloud_cluster->size() << " data points." << std::endl;
    std::stringstream ss;
    ss << m_baseName << "_step" << printStepCount() << "_euclideanCluster_" << std::setfill('0')
       << std::setw(clusterNumberWidth) << std::distance(cluster_indices.begin(), it) << ".pcd";
    m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
  }

  // Make the biggest cluster (cluster_indices[0]) our new working cloud
  if (cluster_indices.size() > 0)
  {
    copyPointCloud(*clusters[0], *m_cloud);

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

//********************************************************************************************************************************************

std::vector<UNL_Robotics::Point2D> UNL_Robotics::SegmentationPipeline::calcBoundingBoxInWorldCoords(bool visualizeBB, double camera_x, double camera_y, double camera_theta)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = m_cloud;

  // Compute principal directions (equivalent to principal components / PCA)
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);

  // Two methods: compute Eigen vectors, or use a PCA object

  // Method 1
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  // The following line is necessary for proper orientation in some cases.
  // The numbers come out the same without it, but the signs are
  //    different and the box doesn't get correctly oriented in some cases.
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  /*
     //Method 2
     // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PCA<pcl::PointXYZ> pca;
     pca.setInputCloud(cloudSegmented);
     pca.project(*cloudSegmented, *cloudPCAprojection);
     Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
  */

  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // For the transformation back to world coordinates
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); // Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  std::stringstream ss;
  ss << m_baseName << "_step" << printStepCount() << "_PCA.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloudSegmented, false);

  std::cout << "PCA:" << std::endl;
  std::cout << eigenVectorsPCA << std::endl;
  std::cout << "Bounding box transform:" << std::endl;
  std::cout << bboxTransform << std::endl;

  Eigen::Vector3f p1(minPoint.x, minPoint.y, minPoint.z);
  Eigen::Vector3f p2(minPoint.x, minPoint.y, maxPoint.z);
  Eigen::Vector3f p3(maxPoint.x, minPoint.y, maxPoint.z);
  Eigen::Vector3f p4(maxPoint.x, minPoint.y, minPoint.z);
  Eigen::Vector3f p5(minPoint.x, maxPoint.y, minPoint.z);
  Eigen::Vector3f p6(minPoint.x, maxPoint.y, maxPoint.z);
  Eigen::Vector3f p7(maxPoint.x, maxPoint.y, maxPoint.z);
  Eigen::Vector3f p8(maxPoint.x, maxPoint.y, minPoint.z);

  std::cout << "Points - untransformed" << std::endl;
  std::cout << "P1" << std::endl
            << p1 << std::endl;
  std::cout << "P2" << std::endl
            << p2 << std::endl;
  std::cout << "P3" << std::endl
            << p3 << std::endl;
  std::cout << "P4" << std::endl
            << p4 << std::endl;
  std::cout << "P5" << std::endl
            << p5 << std::endl;
  std::cout << "P6" << std::endl
            << p6 << std::endl;
  std::cout << "P7" << std::endl
            << p7 << std::endl;
  std::cout << "P8" << std::endl
            << p8 << std::endl;

  // Transform back to world coordinates
  Eigen::Vector3f pr1 = bboxQuaternion * p1 + bboxTransform;
  Eigen::Vector3f pr2 = bboxQuaternion * p2 + bboxTransform;
  Eigen::Vector3f pr3 = bboxQuaternion * p3 + bboxTransform;
  Eigen::Vector3f pr4 = bboxQuaternion * p4 + bboxTransform;
  Eigen::Vector3f pr5 = bboxQuaternion * p5 + bboxTransform;
  Eigen::Vector3f pr6 = bboxQuaternion * p6 + bboxTransform;
  Eigen::Vector3f pr7 = bboxQuaternion * p7 + bboxTransform;
  Eigen::Vector3f pr8 = bboxQuaternion * p8 + bboxTransform;

  std::cout << std::endl
            << std::endl;
  std::cout << "Points with rotation/translation" << std::endl;
  std::cout << "P1" << std::endl
            << pr1 << std::endl;
  std::cout << "P2" << std::endl
            << pr2 << std::endl;
  std::cout << "P3" << std::endl
            << pr3 << std::endl;
  std::cout << "P4" << std::endl
            << pr4 << std::endl;
  std::cout << "P5" << std::endl
            << pr5 << std::endl;
  std::cout << "P6" << std::endl
            << pr6 << std::endl;
  std::cout << "P7" << std::endl
            << pr7 << std::endl;
  std::cout << "P8" << std::endl
            << pr8 << std::endl;

  // Project onto flat 2D space (in this case, x-z of the camera view)
  std::vector<Point2D> points;
  points.push_back({pr1[0], pr1[2]});
  points.push_back({pr2[0], pr2[2]});
  points.push_back({pr3[0], pr3[2]});
  points.push_back({pr4[0], pr4[2]});
  points.push_back({pr5[0], pr5[2]});
  points.push_back({pr6[0], pr6[2]});
  points.push_back({pr7[0], pr7[2]});
  points.push_back({pr8[0], pr8[2]});
  std::cout << "Input points are: " << std::endl;
  std::for_each(points.begin(), points.end(),
                [](Point2D point)
                { std::cout << point.x << ", " << point.y << std::endl; });

  std::vector<Point2D> resultLocal = findConvexHull(points);
  std::cout << "Boundary points of convex hull (local) are: " << endl;
  std::for_each(resultLocal.begin(), resultLocal.end(),
                [](Point2D point)
                { std::cout << point.x << ", " << point.y << std::endl; });

  // Transform these into global coordinates
  std::vector<Point2D> resultGlobal;
  for (auto pt : resultLocal)
  {
    // Based on where the camera is and where it is looking. Note that camera local coords are:
    //   X = perpendicular to the right, Y = in direction of view (out the front)
    double delta_x = sin(camera_theta) * pt.x + cos(camera_theta) * pt.y;
    double delta_y = -1.0 * cos(camera_theta) * pt.x + sin(camera_theta) * pt.y;
    resultGlobal.push_back(Point2D{camera_x + delta_x, camera_y + delta_y});
  }
  assert(resultLocal.size() == resultGlobal.size());
  std::cout << "Boundary points of convex hull (global) are: " << endl;
  std::for_each(resultGlobal.begin(), resultGlobal.end(),
                [](Point2D point)
                { std::cout << point.x << ", " << point.y << std::endl; });

  // Just for visulizing / debugging
  if (visualizeBB)
  {
    // This viewer has 4 windows, but is only showing images in one of them as written here.
    int argc = 1;
    char **argv;
    pcl::visualization::PCLVisualizer *visu = new pcl::visualization::PCLVisualizer(argc, argv, "PlyViewer");
    int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
    // visu->createViewPort(0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
    // visu->createViewPort(0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
    // visu->createViewPort(0.0, 0, 0.5, 0.5,  mesh_vp_3);
    // visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
    // visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);

    visu->createViewPort(0.0, 0.0, 2.0, 2.0, mesh_vp_3);
    visu->addPointCloud(cloudSegmented, "bboxedCloud", mesh_vp_3);
    visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);

    while (!visu->wasStopped())
    {
      visu->spinOnce();
    }
  }

  return resultGlobal;
}

void UNL_Robotics::SegmentationPipeline::calcBoundingBoxInWorldCoords2(bool visualizeBB, double x, double y, double theta)
{
  typedef pcl::PointXYZ PointType;

  pcl::PointCloud<PointType>::Ptr cloud = m_cloud;

  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloud, pcaCentroid);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); // correct vertical between main directions
  eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
  eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

  std::cout << "Eigenvalue va(3x1):\n"
            << eigenValuesPCA << std::endl;
  std::cout << "Feature vector ve(3x3):\n"
            << eigenVectorsPCA << std::endl;
  std::cout << "centroid point (4x1):\n"
            << pcaCentroid << std::endl;
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
  tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();                                     // R.
  tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>()); //  -R*t
  tm_inv = tm.inverse();

  std::cout << "Transformation matrix tm(4x4):\n"
            << tm << std::endl;
  std::cout << "inverter matrix tm'(4x4):\n"
            << tm_inv << std::endl;

  pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(*cloud, *transformedCloud, tm);

  PointType min_p1, max_p1;
  Eigen::Vector3f c1, c;
  pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
  c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

  std::cout << "Centre c1(3x1):\n"
            << c1 << std::endl;

  Eigen::Affine3f tm_inv_aff(tm_inv);
  pcl::transformPoint(c1, c, tm_inv_aff);

  Eigen::Vector3f whd, whd1;
  whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
  whd = whd1;
  float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3; // The average scale of the point cloud, used to set the size of the main direction arrow

  std::cout << "width1=" << whd1(0) << endl;
  std::cout << "heght1=" << whd1(1) << endl;
  std::cout << "depth1=" << whd1(2) << endl;
  std::cout << "scale1=" << sc1 << endl;

  const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
  const Eigen::Vector3f bboxT1(c1);

  const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
  const Eigen::Vector3f bboxT(c);

  // The main direction of the point cloud transformed to the origin
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

  // The main direction of the initial point cloud
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
  pcl::visualization::PCLVisualizer viewer;

  pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0); // Point cloud related to the origin
  viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
  viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");

  viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
  viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
  viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");

  /*
   pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 0, 0); //The initial point cloud input is related
  viewer.addPointCloud(cloud, color_handler, "cloud");
  viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");

  viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
  viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
  viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
  */

  viewer.addCoordinateSystem(0.5f * sc1);
  viewer.setBackgroundColor(1.0, 1.0, 1.0);
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
  }
}



//***********************************************************************************************************************************************

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentationPCL");
  ROS_INFO("Initializiing Segmentation of PCL");

  std::vector<std::string> args = parseCL(argc, argv);

  // creating threads for computation
  uint32_t numThreads(4);
  ros::AsyncSpinner spinner(numThreads);
  spinner.start();

  // Do subscriptions here
  m_objectDetectionSubscriber = nodeHandle.subscribe(, QUEUE, objDetectionCallback); //bbcord.msg
  m_detectionImageSubscriber = nodeHandle.subscribe(, QUEUE, detectionImageCallback); // H_detection_img

  
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // call functions 1 by 1

  SegmentationPipeline segmenter(ssObjPath.str(), boundingBox, pclCloud);
  segmenter.doPlaneExtraction(normal, normalThreshold);
  segmenter.extractObjectInBoundingBox(cropPercentage);

  // not necessary right now

  // segmenter.removeOutliers(meanK, stddevMulThresh);
  // segmenter.performEuclideanExtraction();
  // bool visualizeBB = false;
  // hullPoints = segmenter.calcBoundingBoxInWorldCoords(visualizeBB,
  //                                                     m_currentPose.x,
  //                                                     m_currentPose.y,
  //                                                     m_currentPose.yaw);

  // Rather than ros::spin(), use waitForShutdown() with the async spinner
  ros::waitForShutdown();

  ROS_INFO("Finalizing the bot manager");
  return EXIT_SUCCESS;

  return 1;
}