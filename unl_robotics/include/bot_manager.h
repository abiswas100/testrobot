
////////////////////////////////////
// A manager to control the bot
//
// This is a client of this service:
//
//   move_base_director
//
// Written by:  Matthew Peavy
//              Kyungki Kim
//              
// (c) 2021
//

#ifndef UNL_ROBOTICS_BOT_MANAGER_H
#define UNL_ROBOTICS_BOT_MANAGER_H

#include "inventoryClerk.h"
#include "inspection_plan.h"
// Service messages
#include "unl_smart_robotic_home/node_ready.h"
#include "unl_smart_robotic_home/execute_plan.h"
//ROS
#include "ros/ros.h"
//std
#include <string>

namespace UNL_Robotics {
  
  class bot_manager {
  public:
    bot_manager(ros::NodeHandle& n,
                  const std::string& planFileDirectory,
                  const std::string& inventoryListPath,
                  const std::string& inventoryListFilename,
                  const std::string& imageTopic);
      
  private:
    bool               m_nodeReady;   //Set to true when node is fully ready
    std::string        m_planFileDirectory;
    ros::ServiceClient m_absoluteMoveClient;
    ros::ServiceClient m_relativeMoveClient;
    ros::ServiceServer m_nodeReadyServer;
    ros::ServiceServer m_executePlanServer;
    InventoryClerk     m_inventoryClerk;
    
    bool node_ready_request(unl_smart_robotic_home::node_ready::Request&  request, 
                            unl_smart_robotic_home::node_ready::Response& response);
    
    bool executePlan(unl_smart_robotic_home::execute_plan::Request&  request, 
                     unl_smart_robotic_home::execute_plan::Response& response);
    
    void inspect(unsigned numFullRotations, unsigned roomNumber, const std::string& roomName,
                 unsigned maxOccupancy);
    
  };
}

#endif

