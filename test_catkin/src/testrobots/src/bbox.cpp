#include <ros/ros.h>
#include <std_msgs/String.h>
#include <testrobots/Boundingbox.h>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>

std::ofstream m_out; 

// Camera specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(1920);   // Avhishek - has been updated as image sizes in respect to our Simulation Camera
const unsigned CAMERA_NUM_PIXELS_HEIGHT(1080);
const double CAMERA_HORIZONTAL_VIEW_ANGLE(1.19); // The angle (in radians) of what the camera views , Avhishek - which is an Astra camera 

unsigned xmin = 0;
unsigned xmax = 0;
unsigned ymin = 0;
unsigned ymax = 0;

void BBoxCallback (const testrobots::Boundingbox::ConstPtr &msg);
int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/BBox", 10, BBoxCallback);

  ros::spin();


  return 0;
}
void BBoxCallback (const testrobots::Boundingbox::ConstPtr &msg)
{ 
  ROS_INFO("class: %s", msg->Class.c_str());  
  ROS_INFO("%f", msg->probability);
  ROS_INFO("%ld", msg->xmin);
  ROS_INFO("%ld", msg->xmax);
  ROS_INFO("%ld", msg->ymin);
  ROS_INFO("%ld", msg->xmax);



  std::string objectName = msg->Class.c_str();
  xmin = msg->xmin;
  xmax = msg->xmax;
  ymin = msg->ymin;
  ymax = msg->ymax;
  unsigned x_delta = xmax - xmin;
  unsigned y_delta = ymax - ymin; 
  ROS_INFO_STREAM("  " << objectName  << "  -  Probability " << std::setprecision(4) << (msg->probability*100) << "%" ); // not needed 
  ROS_INFO_STREAM("    " << "BB Min (x,y) = (" << xmin << ", " << ymin << ")" );
  ROS_INFO_STREAM("    " << "BB Max (x,y) = (" << xmax << ", " << ymax << ")" );
  // not needed ************
  std::cout << "*) Object type:  " << objectName << std::endl;
  std::cout << "   Probability  " << std::setprecision(4) << (msg->probability*100.0) << "%" << std::endl;
//*******************

//   // Avhishek - Don't know why  this is being done what is the use of calculating objectAngleOffset

  //Calculate the angle offset of the picture relative to the center of the view port
  unsigned x_centerBB = xmin + static_cast<unsigned>(x_delta/2);
  unsigned y_centerBB = ymin + static_cast<unsigned>(y_delta/2);
  int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH/2) - x_centerBB;   //Can be negative! This orientation assumes CCW=+
  double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

  // Avhishek - m_out is being used for printing and is declared at the top of the file
  m_out << "   " << "Bounding Box (x,y):"
        << "   Min = (" << xmin << ", " << ymin << ")"
        << "   Max = (" << xmax << ", " << ymax << ")"
        << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
  m_out << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;

}

