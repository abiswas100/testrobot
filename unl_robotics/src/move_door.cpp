
/////////////////
// A very simple pub node that commands the door to move
// Pass in the robot's namespace
//
// Topic:        door_bot_ns/door_joint_position_controller/command
// Message:      std_msgs::Float64
// CL Arguments: The robot namesapce, angle to move to (radians)
// 
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2021
//


#include "ros/ros.h"
#include "std_msgs/Float64.h"
//std
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>


unsigned PUB_FREQ(10);       //The publishing frequency
const unsigned QUEUE(100);   //How many messages to allow to queue before discarding
const double TAU(6.283185);  //For Marshall


std::vector<std::string> parseCL(int argc, char** argv)
{
  return std::vector<std::string>(argv+1, argv + argc);
}

int main(int argc, char **argv)
{
  std::vector<std::string> args = parseCL(argc, argv);
  if(args.size() < 2) {
    std::cout << "2 command line args are required: 1) door namespace  2) door angle (in radians)  -- must exit." << std::endl;
    return EXIT_FAILURE;
  }
  
  std::string robotNSName = args[0];
  double angle;
  std::stringstream ss;
  ss << args[1];
  ss >> angle;
  
  ros::init(argc, argv, "move_door_" + robotNSName);
  ros::NodeHandle n;

  std::stringstream topic_joint;
  topic_joint << robotNSName << "/door_joint_position_controller/command";
  ros::Publisher pub_joint = n.advertise<std_msgs::Float64>(topic_joint.str(), QUEUE);

  std_msgs::Float64 msg;
  msg.data = angle;

  //The tick of our clock
  unsigned long tick =0;
  ros::Rate loop_rate(PUB_FREQ);
  
  while(tick < 10)
  {
    //Publish it
    pub_joint.publish(msg);

    //Wait
    ros::spinOnce();
    loop_rate.sleep();

    ++tick;
  }

  return EXIT_SUCCESS;
}
