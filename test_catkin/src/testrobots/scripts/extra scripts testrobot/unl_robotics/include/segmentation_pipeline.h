#ifndef UNL_ROBOTICS_SEGMENTATION_PIPELINE_H
#define UNL_ROBOTICS_SEGMENTATION_PIPELINE_H

#include "convexHull.h"
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//std
#include <string>

namespace UNL_Robotics {

//apala:create structure for bounding box
  struct BoundingBox {
    unsigned xmin;
    unsigned xmax;
    unsigned ymin;
    unsigned ymax;    
  };

  enum class Normal {eX =0, eY, eZ};
  

  //apala:create class SegmentationPipeline
  class SegmentationPipeline {
  public: //apala: access specifier

  
    SegmentationPipeline(pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloud); 
    //apala:pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloud
    // it is a wrapper around a pointer that manages it's lifetime 
    //shared_ptr is a smart pointer that shares ownership  
    //It is reference counted so it can see when the last copy of it goes out 
    //of scope and then it frees the object managed.

    SegmentationPipeline(const std::string& baseName,
                         const BoundingBox& yoloIdentifiedBoundingBox,
                         pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloud);

    SegmentationPipeline(const std::string& baseName,
                         const BoundingBox& yoloIdentifiedBoundingBox,
                         std::string pcdFilepath);


    //Set or reset the base name - used when you want to continue using this pipeline for another extraction
    void setBaseName(const std::string& newBaseName) {m_baseName = newBaseName;}

    //Set or reset the bounding box - used when you want to continue using this pipeline for another extraction
    void setBoundingBox(const BoundingBox& newBoundingBox) {m_boundingBox = newBoundingBox;}

    //If plane extraction has occurred once already, and you want to restart the pipeline
    // from this point, call this function. The post-plane extraction cloud is automatically
    // saved after called doPlaneExtraction, and so the pipeline can be restarted from there.
    void resetCloudToPostPlaneExtraction();
    void resetCloudToPostPlaneExtractionAndBB(const std::string& newBaseName,
                                              const BoundingBox& newBoundingBox);

   
    //Calculations
    double calculateObjectAngleOffset() const;
    double calculateDepths() const;

    ////////////
    //Extractions

    //Extract a plane matching the normal passed in if the normal meets the threshold
    void doPlaneExtraction(Normal, double minThreshold);
    void doPlaneExtraction(Normal, double minThreshold, pcl::PointCloud<pcl::PointXYZ>::Ptr destination);
    
    //Pass in a percentage to crop from all borders.
    //E.g., passing 0.15 crops 15% off each edge, leaving the middle 70%,
    //      passing 0.0 crops exactly the Yolo-identified bounding box passed to constr
    void extractObjectInBoundingBox(double cropPercentage); 
    void extractObjectInBoundingBox(double cropPercentage, pcl::PointCloud<pcl::PointXYZ>::Ptr destination);
    

    //*********************************************************************************************************************************************
    //Use a statistical outlier remover
    void removeOutliers(double meanK, double stddevMulThresh);

    //Euclidean extractions.  The biggest cluster is set as the new working cloud
    void performEuclideanExtraction();

    //Calculate the 2D convex hull in world coordinates, given the current robot pose
    // and the camera angle theta (in radians)
    std::vector<UNL_Robotics::Point2D> calcBoundingBoxInWorldCoords(bool visualizeBB, double x, double y, double theta);
    void calcBoundingBoxInWorldCoords2(bool visualizeBB, double x, double y, double theta);
    
    //Print the minimum and maximum points of the entire cloud at this point in the pipeline
    void printMinMax();
    void printMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    //***********************************************************************************************************************************************
    
  private:
    pcl::PCDWriter m_writer;
    unsigned m_pipelineStepCount;  //What step are we in the pipeline. Major steps by 10, minor by 1
    std::string m_baseName;      //The base file name we will append and save to
    BoundingBox m_boundingBox;

    //Convenience function to print the pipline step count formatted to 2 digits, padding from the front with a 0 if necessary
    std::string printStepCount() const;
    std::string printStepCount(unsigned addition) const;
    
    //The working point cloud. Starts as the cloud passed in the constructor, but can be reset
    // to the post-plane-extracted cloud to avoid re-extracting the plane for each object
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_postPlaneExtractedCloud;

    void removeNaNs(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest);
  };

}

#endif
