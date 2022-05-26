#include <ros/ros.h>
#include <std_msgs/String.h>
#include <testrobots/Boundingbox.h>
#include <string>

void chatterCallback (const testrobots::Boundingbox::ConstPtr &msg);
int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/BBox", 10, chatterCallback);

  ros::spin();


  return 0;
}
void chatterCallback (const testrobots::Boundingbox::ConstPtr &msg)
{ 
  ROS_INFO("class: %s", msg->Class.c_str());  
  ROS_INFO("%f", msg->probability);
  ROS_INFO("%ld", msg->xmin);
  ROS_INFO("%ld", msg->xmax);
  ROS_INFO("%ld", msg->ymin);
  ROS_INFO("%ld", msg->xmax);
        // ROS_INFO("recieving");
    // ROS_INFO("I heard: ");
}


// #include "ros/ros.h"
// #include "std_msgs/String.h"


// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }
// // %EndTag(CALLBACK)%

// int main(int argc, char **argv)
// {
  
//   ros::init(argc, argv, "listener");

 
//   ros::NodeHandle n;

  
//   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

//   ros::spin();

//   return 0;
// }