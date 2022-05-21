/////////////////
// An object recognizing inventory clerk.
// Will listen for Yolo topics of recognized
// images and keep track of all objects that
// it knows with an inventory list.
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2022
//

#ifndef UNL_ROBOTICS_INVENTORYCLERK_H
#define UNL_ROBOTICS_INVENTORYCLERK_H

#include "convexHull.h"
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>                 //Point clouds
//Darknet / Yolo
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/ObjectCount.h"
//PCL
#include <pcl_ros/point_cloud.h>                 //Point clouds
//std
#include <fstream>

namespace UNL_Robotics {
  
  class InventoryClerk {
  public:

    struct Pose {
      double x;
      double y;
      double yaw;
    };

    struct LongTermObject {
      std::string objectType;
      std::vector<UNL_Robotics::Point2D> hullPoints;  //Convex hull points (x,y)
    };

    /////////
    
    InventoryClerk(ros::NodeHandle& nodeHandle,
                   const std::string& workingPath,
                   const std::string& inventoryFilename,
                   bool beginPaused = true,
                   bool useDownSampling = false);


    void setImageTopic(const std::string& imageTopic);  //Overrides default
    void beginInspection(unsigned roomNumber, const std::string& roomName);  //No images taken. Waits until the image is current
    void inspectPosition(unsigned pointNum);
    void recordMaxOccupancyResult(unsigned maxOccupancy);  //Called when a room inspection is finished

    bool longTermObjectDetected() {return m_longTermObjectDetectedAtThisPosition;}
    std::vector<LongTermObject> getLongTermObjects() {return m_longTermObjects;}
    
    
  private:
    std::ofstream m_out;         //Stream for distances datalog
    std::string m_workingPath;   //Path for saving inventory file and images
    std::string m_imageTopic;    //The topic used for reading images to process/pass to Yolo
    bool m_pause;                //Only record items when this is false
    bool m_YOLO_imageReceived;   //Set to true whenever a Yolo image is received during an unpaused period
    bool m_currentlyProcessingObject; //Set to true when we are processing a Yolo-recognized object
                                       //so that we allow the image processing to finish before going on.
    bool m_useDownSampling;      //Whether to use a downsampling filter on the cup
    Pose m_currentPose;          //Store current pose corresponding to the "latest" images/clouds below

    //$REMOVE THIS
    //unsigned m_numPeopleInRoom;  //Keep track of how many people in this room. Reset when beginInspection() called
    std::vector<double> m_peopleInRoom;  //This is the positive angle rotation to people in the room
    
    //Subscribers
    ros::Subscriber m_objectDetectionSubscriber;
    ros::Subscriber m_detectionImageSubscriber;
    ros::Publisher  m_yoloImagePublisher;

    //The latest image/cloud messages
    sensor_msgs::Image m_latestRGBImage;
    sensor_msgs::PointCloud2 m_latestPointCloud;


    //Long-term object info
    bool m_longTermObjectDetectedAtThisPosition;
    std::vector<LongTermObject> m_longTermObjects;
    
    //Callbacks
    void objDetectionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg);

    //Allow a thread to use this to broadcast the latest image
    void broadcastLatestImage();
  };

}

std::ostream& operator<<(std::ostream&, const UNL_Robotics::InventoryClerk::Pose&);

#endif
