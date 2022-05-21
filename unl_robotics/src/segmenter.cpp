
//This is a stand-alone app used for driving the
// segmentation pipeline.  Mostly used for testing
// and development of the pipeline.


#include "segmentation_pipeline.h"
//std
#include <cstdlib>
#include <string>
#include <vector>
#include <ctime>
#include <iomanip>


//These are data sets that were previously run, with point-cloud files and Yolo recognition
enum class PCDDataSet {
  set_20210608_133349 = 0,    //Simulation | Pureza | Couch
  set_20210610_080831,        //Simulation | Pureza | Couch
  set_162378035574815,        //Real life  | Suonen | 1a)  Bed   w/see through slats
  set_162384299743334,        //Real life  | Suonen | 1b)  Bed   w/slats covered
  set_162378089703168,        //Real life  | Suonen | 2a) Chair
  set_162378128238851,        //Real life  | Suonen | 2b) Chair
  set_162378176853040,        //Real life  | Suonen | 3)  Refrigerator
  numPCDDataSets
};

namespace {

  std::vector<std::string> parseCL(int argc, char** argv)
  {
    return std::vector<std::string>(argv+1, argv + argc);
  }

  std::string timestamp()
  {
    std::time_t now_time_t = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
  }

  //Get the bounding box information specific to a data set
  UNL_Robotics::BoundingBox getBB(PCDDataSet dataSet) {

    unsigned xmin;
    unsigned xmax;
    unsigned ymin;
    unsigned ymax;
    
    if(dataSet == PCDDataSet::set_20210608_133349) {
      xmin = 181;
      xmax = 563;
      ymin = 147;
      ymax = 311;
    }
    else if(dataSet == PCDDataSet::set_20210610_080831) {
      xmin = 168;
      xmax = 507;
      ymin = 178;
      ymax = 292;
    }
    else if(dataSet == PCDDataSet::set_162378035574815) {   //Bed 1a
      xmin = 124;
      xmax = 506;
      ymin = 269;
      ymax = 480;
    }
    else if(dataSet == PCDDataSet::set_162384299743334) {   //Bed 1b
      xmin = 107;
      xmax = 516;
      ymin = 227;
      ymax = 474;
    }
    else if(dataSet == PCDDataSet::set_162378089703168) {   //Chair 2a
      xmin = 252;
      xmax = 445;
      ymin = 133;
      ymax = 431;
    }
    else if(dataSet == PCDDataSet::set_162378128238851) {   //Chair 2b
      xmin = 149;
      xmax = 398;
      ymin = 127;
      ymax = 467;
    }
    else if(dataSet == PCDDataSet::set_162378176853040) {   //Refrigerator
      xmin = 358;
      xmax = 511;
      ymin = 6;
      ymax = 465;
    }
    else {
      assert(false);
      throw("Out of bounds PCDDataSet enum");
    }
  
    return UNL_Robotics::BoundingBox{xmin, xmax, ymin, ymax};
  }
}

///////////////////
  
int main(int argc, char** argv)
{
  std::vector<std::string> args = parseCL(argc, argv);
  if(args.size() != 1) {
    std::cout << "Must include exactly 1 argument on the CL - a file with the timestamp as the first 15 chars. Must exit." << std::endl;
    std::cout << "number of args = " << args.size() << std::endl;
    return EXIT_FAILURE;
  }

  std::string arg1 = args[0];
  if(arg1.length() < 15) {
    std::cout << "Must include 1 CL arg with a file containing the timestamp as the first 15 chars. Must exit." << std::endl;
    std::cout << "arg1 = " << arg1 << std::endl;
    std::cout << "arg1 length = " << arg1.length() << std::endl;
    return EXIT_FAILURE;
  }
  
  std::string timestamp = arg1.substr(0,15);
  std::cout <<  std::endl;
  std::cout << "Working on segmentation for file with timestamp: " << timestamp << std::endl << std::endl;;
  
  //The object type
  std::string objectName = "couch";

  //Set up the bounding box - specific to the object you are segmenting (identified by yolo)
  UNL_Robotics::BoundingBox yoloIdentifiedBoundingBox;
  if(timestamp == "20210608_133349")
     yoloIdentifiedBoundingBox = getBB(PCDDataSet::set_20210608_133349);
  else if(timestamp == "20210610_080831")
     yoloIdentifiedBoundingBox = getBB(PCDDataSet::set_20210610_080831);
  else if(timestamp == "162378035574815")
     yoloIdentifiedBoundingBox = getBB(PCDDataSet::set_162378035574815);
  else if(timestamp == "162384299743334")
     yoloIdentifiedBoundingBox = getBB(PCDDataSet::set_162384299743334);
  else if(timestamp == "162378089703168")
     yoloIdentifiedBoundingBox = getBB(PCDDataSet::set_162378089703168);
  else if(timestamp == "162378128238851")
     yoloIdentifiedBoundingBox = getBB(PCDDataSet::set_162378128238851);
  else if(timestamp == "162378176853040")
     yoloIdentifiedBoundingBox = getBB(PCDDataSet::set_162378176853040);
  else {
    std::cout << "Unknown data set. No bounding box information. Must exit." << std::endl;
    return EXIT_FAILURE;
  }

  //Parameters
  std::string baseName = timestamp + "_";
  std::string pcdFilepath = timestamp + "_fullPointCloud.pcd";

  //The higher the theshold, the more exact the plane must be. In Gazebo, this can be 0.99
  // but not so high in the real world
  double normalThreshold = 0.97;
  UNL_Robotics::Normal normal = UNL_Robotics::Normal::eY;

  //How must to crop around the bounding box
  double cropPercentage = 0.02;        // 0.00  to  0.20

  //Statistical outlier removal parameters
  double meanK = 50.0;                   // 50.0  to  100.0
  double stddevMulThresh = 0.5;           // 0.5  to    1.0
  
  std::cout << "Beginning segmentation" << std::endl;
  std::cout << "======================" << std::endl;
  std::cout  << std::endl;

  UNL_Robotics::SegmentationPipeline segmenter(baseName, yoloIdentifiedBoundingBox, pcdFilepath);

  std::cout << "*) Raw data" << std::endl;
  segmenter.printMinMax();
  std::cout << std::endl;

  std::cout << "*) Extracting floor plane" << std::endl;
  segmenter.doPlaneExtraction(normal, normalThreshold);

  std::cout << "*) Extracting from bounding box, with crop = " << cropPercentage << " ...." << std::endl;  
  segmenter.extractObjectInBoundingBox(cropPercentage);
  
  std::cout << "*) Removing outliers...." << std::endl;
  segmenter.removeOutliers(meanK, stddevMulThresh);
  segmenter.printMinMax();
  std::cout << std::endl;

  std::cout << "*) Performing Euclidean extraction...." << std::endl;
  segmenter.performEuclideanExtraction();
  segmenter.printMinMax();
  std::cout  << std::endl;

  bool visualizeBB = true;
  std::cout << "*) Calculating bounding box...." << std::endl;
  segmenter.calcBoundingBoxInWorldCoords(visualizeBB, 0,0,0);
  segmenter.printMinMax();
  segmenter.calcBoundingBoxInWorldCoords2(visualizeBB, 0,0,0);
  std::cout << std::endl;

  std::cout << "======================" << std::endl;
  std::cout << "Finishing segmentation" << std::endl;
  
  return EXIT_SUCCESS;
}
