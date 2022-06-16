
#include "PeopleLocalizer.h"
//Unix
#include "unistd.h"     //Used to get/print the CWD for debugging purposes
//std
#include <cstdlib>
#include <fstream>

const double PI = 3.141592653589793;

//How many observations stops in a full rotation? If less than 6, the Astra camera will not record
// the whole scene. It has a view angle width of approximately 1.19 radians, so at least 6 positions are required.
const unsigned NUM_OBSERVATION_POINTS_PER_FULL_ROTATION(6);

//Robot seems to underestimate rotation by this offset amount
const double ROTATION_ERROR_OFFSET(0.23);    


namespace {

  std::vector<std::string> parseCL(int argc, char** argv)
  {
    return std::vector<std::string>(argv+1, argv + argc);
  }
  
  std::string add_s(unsigned val)
  {
    if(val == 1)
      return "";
    else
      return "s";
  }

  void pwd()
  {
    //Just using to test the CWD. Print to Debug level log
    char buff[FILENAME_MAX];
    char* result = getcwd( buff, FILENAME_MAX );
    if(result == NULL) {
      std::cout << "Could not determine the CWD" << std::endl;
      ROS_ERROR("Could not determine the current working directory");
    }
    else {
      std::string cwd(buff);
      std::cout << "CWD = " << cwd << std::endl;
      ROS_INFO_STREAM("CWD = " << cwd);
    }
  }
  
}

///// MAIN ////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_observer");
  ROS_INFO("Initializing the people_observer node");

  //Check if there was a command-line arg. If so, it is for re-setting the image topic.
  std::vector<std::string> args = parseCL(argc, argv);
  std::string imageTopic;
  if(args.size() > 0) {
    imageTopic = args[0];
    ROS_INFO_STREAM("Command line request to override the image topic. Setting to " << imageTopic);
  }
    
  //Print the working directory
  pwd();

  uint32_t numThreads(4);
  ros::AsyncSpinner spinner(numThreads);
  spinner.start();

  ros::NodeHandle nodeHandle;
  UNL_Robotics::PeopleLocalizer localizer(nodeHandle, "./", "people.txt", true, false);
  localizer.beginInspection(0, "begin");
  localizer.inspectPosition(0);
  
  //Rather than ros::spin(), use waitForShutdown() with the async spinner 
  ros::waitForShutdown();

  ROS_INFO("Finalizing the people_observer node");
  return EXIT_SUCCESS;
}
