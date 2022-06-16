
#include "bot_manager.h"
#include "segmentation_pipeline.h"
#include "map_writer.h"
//Message headers
#include "unl_smart_robotic_home/move_absolute.h"
#include "unl_smart_robotic_home/move_relative.h"
//Dynamic reconfigure for the Turtlebot speed limit
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
//Unix
#include "unistd.h"     //Used to get/print the CWD for debugging purposes
#include <sys/stat.h>   //For mkdir
#include <sys/types.h>
//std
#include <cstdlib>
#include <fstream>

const double PI = 3.141592653589793;

//How many observations stops in a full rotation? If less than 6, the Astra camera will not record
// the whole scene. It has a view angle width of approximately 1.19 radians, so at least 6 positions are required.
const unsigned NUM_OBSERVATION_POINTS_PER_FULL_ROTATION(5);

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

  bool myMkdir(const std::string& dir)
  {
    return(mkdir(dir.c_str(), 0777) == 0);
  }
  
}

//The filePath contains only the path from the CWD, not including the filename.
//The filename contains the filename stem with no extension (i.e., no ".xml")
UNL_Robotics::bot_manager::bot_manager(ros::NodeHandle& nodeHandle,
                                           const std::string& planFileDirectory,
                                           const std::string& inventoryListPath,
                                           const std::string& inventoryListFilename,
                                           const std::string& imageTopic)
  : m_nodeReady(false), m_planFileDirectory(planFileDirectory),
    m_inventoryClerk(nodeHandle, inventoryListPath, inventoryListFilename, true)  //Start the inventory clerk as "paused"
{
  //If the image topic was set, pass it to the inventory clerk
  if(!imageTopic.empty())
    m_inventoryClerk.setImageTopic(imageTopic);

  //Set up the service callbacks. Since they are member variables, they will stay persistent
  m_nodeReadyServer = nodeHandle.advertiseService("bot_manager/node_ready",
                                                  &bot_manager::node_ready_request,
                                                  this);
  m_nodeReadyServer = nodeHandle.advertiseService("bot_manager/execute_plan",
                                                  &bot_manager::executePlan,
                                                  this);
  ROS_INFO_STREAM("Set up bot_manager node services.");
  
  ////////////////

  ROS_INFO_STREAM("Checking whether worker nodes are ready.");
    
  //Check the move_base_director status
//   ros::ServiceClient moveClientReady = nodeHandle.serviceClient<unl_smart_robotic_home::node_ready>("move_base_director/node_ready");
//   unl_smart_robotic_home::node_ready move_base_director_ready_msg;
//   bool done=false;
//   unsigned second =0;longTermOond" << add_s(second));
//     }
//     else {
//       ++second;
//       if(second == maxSeconds) {
//         ROS_ERROR_STREAM("Waited " << second << " second" << add_s(second) << ", which is the maximum. Cannot continue without the move_base_director. Will now abort");
//         exit(EXIT_FAILURE);
//       }
//       else {
//         ros::Duration(1.0).sleep();  //Sleep for a second
//         ROS_INFO_STREAM("Waiting for move_base_director to be ready. So far waited for " << second << " of " << maxSeconds << " second" << add_s(maxSeconds));
//       }
//     }
//   } while(!done);

//   //Set the maximum robot velocity used. This is done in the planner.
//   double max_vel_x = 0.2;
//   double max_rot_vel = 1.0;
//   dynamic_reconfigure::ReconfigureRequest srv_req;
//   dynamic_reconfigure::ReconfigureResponse srv_resp;
//   dynamic_reconfigure::DoubleParameter vel_x_param;
//   dynamic_reconfigure::DoubleParameter vel_rot_param;
//   dynamic_reconfigure::Config conf;
//   vel_x_param.name = "max_vel_x";
//   vel_x_param.value = max_vel_x;
//   conf.doubles.push_back(vel_x_param);
//   vel_rot_param.name = "max_rot_vel";
//   vel_rot_param.value = max_rot_vel;
//   conf.doubles.push_back(vel_rot_param);
//   srv_req.config = conf;
//   ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
  
//   //Now that the worker nodes are certified ready, set up their client service interfaces
//   m_absoluteMoveClient = nodeHandle.serviceClient<unl_smart_robotic_home::move_absolute>("move_base_director/move_absolute");
//   m_relativeMoveClient = nodeHandle.serviceClient<unl_smart_robotic_home::move_relative>("move_base_director/move_relative");
  
//   ROS_INFO_STREAM("Worker nodes are ready");
//   ROS_INFO_STREAM("The bot manager node is ready");
//   m_nodeReady = true;
// }

// ////  PRIVATE   ////

// bool UNL_Robotics::bot_manager::node_ready_request(unl_smart_robotic_home::node_ready::Request&  request, 
//                                                      unl_smart_robotic_home::node_ready::Response& response)
// {
//   ROS_INFO_STREAM("Responded to node ready request with response: " << std::boolalpha << m_nodeReady);
//   response.ready = m_nodeReady;
//   return true;
// }

// bool UNL_Robotics::bot_manager::executePlan(unl_smart_robotic_home::execute_plan::Request&  request, 
//                                                 unl_smart_robotic_home::execute_plan::Response& response)
// {
//   response.result = true;
//   ROS_INFO_STREAM("Received execute plan request. Starting inspection routine");

//   std::string planFilename = request.plan_filename;
  
//   //Load up the management plan
//   std::string fullPath = m_planFileDirectory + planFilename;
//   std::ifstream in_mp(fullPath.c_str());
//   if(!in_mp) {
//     ROS_ERROR_STREAM("Can't open management plan file with path: " << fullPath << "  -- Consider setting the ROS_HOME environment variable so that the management file can be loaded, or running launch script from the catkin workspace prepended with:   ROS_HOME=`pwd`");
//     ROS_FATAL("No management plan available, so must exit");
//     exit(EXIT_FAILURE);
//   }
//   ROS_INFO_STREAM("Opened management plan file with path: " << fullPath);
  
//   std::tuple<bool, InspectionRoute> routeTuple = parseInspectionRoute(in_mp);
//   bool parseResult = std::get<0>(routeTuple);
//   if(!parseResult) {recordMaxOccupancyResult
//   }
    
//   InspectionRoute route = std::get<1>(routeTuple);
//   ROS_INFO_STREAM("Parsed inspection plan file successfully. Number of rooms = " << route.size());

//   //For debugging!!
//   //Dump the file back out to verify that we're reading what we think we're reading
//   //std::string debugFilePath  = m_planFileDirectory + planFilename + ".DEBUG.txt";
//   //std::ofstream out(debugFilePath.c_str());
//   //if(!out) {
//   //  ROS_ERROR_STREAM("Can't open output debug inspection plan file with path: " << debugFilePath);
//   //}
//   //else {
//   //  out << "# This is a debug file generated as a mirror of the inspection route data read in." << std::endl;
//   //  out << "# It serves to verify that the data is being accurately read from the input file." << std::endl;
//   //  out << std::endl;
// //   //  out << route;
// //   //}
  

//   //Iterate over all the rooms to inspect
//   for(auto roomRoute : route) {

//     ROS_INFO_STREAM("Starting inspection routine for room #" << roomRoute.room().roomID);
  
//     unl_smart_robotic_home::move_absolute centerRoomMoveReq;
//     centerRoomMoveReq.request.x = roomRoute.room().roomCenter.x;
//     centerRoomMoveReq.request.y = roomRoute.room().roomCenter.y;
//     centerRoomMoveReq.request.theta = 0.0;
//     if(m_absoluteMoveClient.call(centerRoomMoveReq))  {
//       ROS_INFO("Move to center of the room successfully executed");
//       response.result = true;
//     }    
//     else {
//       ROS_ERROR("Move to center of the room not successfully executed");
//       response.result = false;
//       return false;
//     }

//     inspect(roomRoute.getNumRotations(), roomRoute.room().roomID,
//             roomRoute.room().roomName, roomRoute.room().maxOccupancy);
//   }
 
//   ROS_INFO_STREAM("Finished inspection routine");
//   return response.result;
// }

void UNL_Robotics::bot_manager::inspect(unsigned numFullRotations, unsigned roomNumber, const std::string& roomName,
                                          unsigned maxOccupancy)
{
  ROS_INFO("Starting room inspection");
  ROS_INFO_STREAM("Number of full rotations for this inspection site is " << numFullRotations);

  //The number of observation points per full rotation
  const double rotationAnglePerObservation(((2.0 * PI) / static_cast<double>(NUM_OBSERVATION_POINTS_PER_FULL_ROTATION)
                                             +  ROTATION_ERROR_OFFSET));   //In radians
  ROS_INFO_STREAM("Number of observations per full rotation is " << NUM_OBSERVATION_POINTS_PER_FULL_ROTATION);
  
  //Set up the move request
  unl_smart_robotic_home::move_relative moveReq;
  moveReq.request.rotate = true;
  moveReq.request.translate = false;
  moveReq.request.theta = rotationAnglePerObservation;   // (rad)
  
  //Request to begin inspection. No images taken. The Inv.Clerk will wait (block) until the image is current
  ROS_INFO_STREAM("Instructing inventory clerk to begin inspection");
  //define roomnumber and roomname as static
  m_inventoryClerk.beginInspection(roomNumber, roomName);

  unsigned numObservationPoints = numFullRotations * NUM_OBSERVATION_POINTS_PER_FULL_ROTATION;

  ROS_INFO_STREAM("Beginning inspection of a total of " << numObservationPoints << " observation points");
  //Rotate through observation points
  for(unsigned obsPoint=1; obsPoint <= numObservationPoints; ++obsPoint) {
  
    //Process an image. This blocks until a new image is received
    ROS_INFO_STREAM("Calling for inspection at observation point " << obsPoint);    
    m_inventoryClerk.inspectPosition(obsPoint);

    //Detect if a long-term object was detected
    ROS_INFO_STREAM("Checking if long-term objects were detected");
    if(m_inventoryClerk.longTermObjectDetected()) {
      
      //Get the object information
      std::vector<InventoryClerk::LongTermObject> ltos = m_inventoryClerk.getLongTermObjects();
      assert(ltos.size() >= 1);
      ROS_INFO_STREAM(ltos.size() << " long-term objects were detected");

      std::string pgmPath("./buildings/pureza/maps");
      std::string yamlFilename("pureza.yaml");
      
      for(auto obj : ltos) {
        ROS_INFO_STREAM("  - long-term object is: " << obj.objectType);
        ROS_INFO_STREAM("  - object convect hull points: ");
        for(auto pt : obj.hullPoints)
          ROS_INFO_STREAM("     (" << pt.x << ", " << pt.y << ")");

        //Copy the current map
        //ROS_INFO_STREAM("Copying the current map");

        //Insert the object into the map
        std::string yamlFilepath(pgmPath + "/" + yamlFilename);
        map_writer writer;
        bool insertResult = writer.insertObject(yamlFilepath, pgmPath, obj.hullPoints);
        if(insertResult)
          ROS_INFO_STREAM("Updated the map with long-term object successfully");
        else
          ROS_ERROR_STREAM("Failed to insert the object into the map");
      }

      //Kill the current map relay node
      ROS_INFO_STREAM("Killing the current map relay node");
      system("rosnode kill map_relay");

      //Start a new map server
      // $UPDATE THIS - would be better to do this through ROS rather than the OS
      ROS_INFO_STREAM("Starting a new map server");
      std::stringstream ssCommand;
      ssCommand << "rosrun map_server map_server map_server:=map_server_001 " << pgmPath << "/" << yamlFilename << " &";
      system(ssCommand.str().c_str());

      //Start the new relay node
      ROS_INFO_STREAM("Starting a new relay node");
      
    }
    else
      ROS_INFO_STREAM("No long-term objects were detected");
    
    //Now move to a new position
    ROS_INFO_STREAM("Calling for a rotation to a new observation position");
    m_relativeMoveClient.call(moveReq);
  }
    
  //Record the occupancy result
  m_inventoryClerk.recordMaxOccupancyResult(maxOccupancy);
  
  ROS_INFO("Room inspection successfully executed");
}
