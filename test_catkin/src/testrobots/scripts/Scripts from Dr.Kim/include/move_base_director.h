////////////////////////////////
// A move_base director
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2021
//

#ifndef UNL_ROBOTICS_MOVE_BASE_DIRECTOR
#define UNL_ROBOTICS_MOVE_BASE_DIRECTOR

//ROS
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// Service messages lead to autogenerated header files in the /devel/include/<projName>
// where <projName> is the name of the project within the CMakeLists.txt file.
// The CMakeLists.txt file must include a directive to generate these message/header files.
#include "facility_management_robot/node_ready.h"
#include "facility_management_robot/move_base_request.h"


namespace UNL_Robotics {

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  
  class move_base_director {
  public:
    move_base_director();
    ~move_base_director();
    
  private:
    bool m_nodeReady;   //Set to true when node is fully ready
    ros::ServiceServer m_service_move_request;
    ros::ServiceServer m_service_node_ready;
    MoveBaseClient m_actionClient;
    
    bool node_ready_request(facility_management_robot::node_ready::Request&  request, 
                            facility_management_robot::node_ready::Response& response);
    
    bool move_base_request(facility_management_robot::move_base_request::Request&  request, 
                           facility_management_robot::move_base_request::Response& response);
    
  };
}

#endif
