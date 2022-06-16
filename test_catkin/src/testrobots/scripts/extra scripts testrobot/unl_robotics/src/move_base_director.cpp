
#include "move_base_director.h"
//ROS 
#include <ros/ros.h>
//std
#include <cstdlib>
#include <string>

//The frame IDs. Used for absolute (map) and relative (base_link) movements
const std::string FRAME_ID_MAP = "/map";
const std::string FRAME_ID_BASE_LINK = "/base_link";


UNL_Robotics::move_base_director::move_base_director()
  : m_nodeReady(false),
    m_actionClient("move_base", true)
{
  ros::NodeHandle nh;

  //Adverise services
  m_service_node_ready = nh.advertiseService("move_base_director/node_ready",
                                             &move_base_director::node_ready,
                                             this);
  m_service_move_absolute = nh.advertiseService("move_base_director/move_absolute",
                                                &move_base_director::move_absolute,
                                                this);
  m_service_move_relative = nh.advertiseService("move_base_director/move_relative",
                                                &move_base_director::move_relative,
                                                this);

  //Wait for the move_base server to come up before we can report that this node is ready
  while(!m_actionClient.waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the move_base action server to come up");

  ROS_INFO_STREAM("move_base_director using frame ID:  " << FRAME_ID_MAP << "  for absolute movements");
  ROS_INFO_STREAM("move_base_director using frame ID:  " << FRAME_ID_BASE_LINK << "  for relative movements");
  ROS_INFO("The move_base_director server is ready");
  m_nodeReady = true;
}

UNL_Robotics::move_base_director::~move_base_director()
{
}

/////////////

bool UNL_Robotics::move_base_director::node_ready(unl_smart_robotic_home::node_ready::Request&  request, 
                                                  unl_smart_robotic_home::node_ready::Response& response)
{
  ROS_INFO_STREAM("Responded to node ready request with response: " << std::boolalpha << m_nodeReady);
  response.ready = m_nodeReady;
  return true;
}

bool UNL_Robotics::move_base_director::move_absolute(unl_smart_robotic_home::move_absolute::Request&  request, 
                                                     unl_smart_robotic_home::move_absolute::Response& response)
{
  ROS_INFO("move_base_director absolute move request received");
  
  move_base_msgs::MoveBaseGoal goal;

  //For absolute (map-based) movement, set the frame_id to map
  goal.target_pose.header.frame_id = FRAME_ID_MAP;
  goal.target_pose.header.stamp = ros::Time::now();
  
  double theta = request.theta;  //Final pose angle desired  -- see comments about theta above
  goal.target_pose.pose.position.x = request.x;
  goal.target_pose.pose.position.y = request.y;    
  goal.target_pose.pose.orientation.z = sin(theta/2.0);
  goal.target_pose.pose.orientation.w = cos(theta/2.0);
  
  m_actionClient.sendGoal(goal);
  m_actionClient.waitForResult();
  if(m_actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("The base moved to absolute (map) coords:  x=" << request.x << ",  y=" << request.y << ", z-rot=" << theta << " radians");
  }
  else {
    ROS_ERROR("The base failed to move to absolute (map) coordinates");
    response.move_result = false;
    return false;
  }
  
  response.move_result = true;
  return true;
}

bool UNL_Robotics::move_base_director::move_relative(unl_smart_robotic_home::move_relative::Request&  request, 
                                                     unl_smart_robotic_home::move_relative::Response& response)
{
  ROS_INFO("move_base_director relative move request received");
  
  move_base_msgs::MoveBaseGoal goal;

  //For relative movement, set the frame_id to base_link
  goal.target_pose.header.frame_id = FRAME_ID_BASE_LINK;
  goal.target_pose.header.stamp = ros::Time::now();
  
  //First rotate, then translate (if set in these bools). Use quaternion rotation.
  if(request.rotate) {
    //For a pure rotation about the z-axis (Yaw), the angle desired,
    // theta, must be divided by 2 when put into the quaternion.
    //The rotation direction is the right-hand-rule about the upward
    // pointing z-axis (counter-clockwise when observed above the plane).
    //Set the z-part equal to the sine and w-part equal to the cosine
    // of the 1/2 angle.
    //Note that the quaternion will turn to the shortest direction, so
    // 3 PI/2 in the CCW may be effected as a PI/2 in the clockwise.
    double theta = request.theta;
    goal.target_pose.pose.orientation.z = sin(theta/2.0);
    goal.target_pose.pose.orientation.w = cos(theta/2.0);
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    
    ROS_INFO("Sending rotation goal");
    m_actionClient.sendGoal(goal);
    m_actionClient.waitForResult();
    if(m_actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO_STREAM("The base rotated " << theta << " radians");
    }
    else {
      ROS_ERROR("The base failed to rotate");
      response.move_result = false;
      return false;
    }
  }
  
  if(request.translate) {
    //For a pure translation, set the w-orientation to 1.0
    // and then the x and/or y goals, where x is forward, and
    // y is to the left.
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.position.x = request.x;
    goal.target_pose.pose.position.y = request.y;
    
    ROS_INFO("Sending translation goal");
    m_actionClient.sendGoal(goal);
    m_actionClient.waitForResult();
    if(m_actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO_STREAM("The base translated:  x=" << request.x << ",  y=" << request.y);
    }
    else {
      ROS_ERROR("The base failed to translate");
      response.move_result = false;
      return false;
    }        
  }

  response.move_result = true;
  return true;
}

int main(int argc, char** argv)
{
  ROS_INFO("Initializing the move_base_director server");
  ros::init(argc, argv, "move_base_director");

  uint32_t numThreads(4);
  ros::AsyncSpinner spinner(numThreads);
  spinner.start();
  
  UNL_Robotics::move_base_director director;

  //Rather than ros::spin(), user waitForShutdown() with the async spinner 
  ros::waitForShutdown();
  
  return EXIT_SUCCESS;
}

