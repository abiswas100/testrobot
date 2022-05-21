
/////////////////
// A very simple pub node that commands the door to swing back and forth.
// Pass in the robot's namespace
//
// Topic:        door_bot_ns/door_joint_position_controller/command
// Message:      std_msgs::Float64
// CL Arguments: The robot namesapce
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

const double TAU(6.283185);  //For Marshall

std::vector<std::string> parseCL(int argc, char** argv)
{
  return std::vector<std::string>(argv+1, argv + argc);
}

int main(int argc, char **argv)
{
  std::vector<std::string> args = parseCL(argc, argv);
  std::string robotNSName = "door_bot_ns";   //The default namespace name if no argument is passed
  if(args.size() > 0)
    robotNSName = args[0];
  
  ros::init(argc, argv, "swing_door_" + robotNSName);
  ros::NodeHandle n;

  const unsigned QUEUE(100);   //How many messages to allow to queue before discarding
  std::stringstream topic_joint;
  topic_joint << robotNSName << "/door_joint_position_controller/command";
  ros::Publisher pub_joint = n.advertise<std_msgs::Float64>(topic_joint.str(), QUEUE);

  //The publishing frequency
  unsigned PUB_FREQ(10);
  ros::Rate loop_rate(PUB_FREQ);

  //The tick of our clock
  unsigned long tick =0;

  //The frequency (Hz) of the door swing
  const double f = 0.1;
  
  while (ros::ok())
  {
    //Our message is an angle of rotation for the joint
    std_msgs::Float64 msg;
    msg.data = sin(TAU * f * static_cast<double>(tick) / static_cast<double>(PUB_FREQ));

    //Publish it
    pub_joint.publish(msg);

    //Wait
    ros::spinOnce();
    loop_rate.sleep();

    ++tick;
  }

  return EXIT_SUCCESS;
}
