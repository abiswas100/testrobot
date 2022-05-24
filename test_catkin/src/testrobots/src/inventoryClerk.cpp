void UNL_Robotics::InventoryClerk::inspectPosition(unsigned positionNumber)
{
  ROS_INFO_STREAM("InventoryClerk - inspecting position #" << positionNumber);
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);

  //Begin by setting the lont-germ object detection flag to false
  m_longTermObjectDetectedAtThisPosition = false;
  m_longTermObjects.clear();
  
  m_out << "-- Observation point # " << positionNumber << " --" << std::endl;
    std::time_t now_time_t = std::time(nullptr);
  m_out << std::put_time(std::localtime(&now_time_t), "%b %d %Y,  %H:%M:%S") << std::endl;
  
  // use this 
  ROS_INFO("Obtaining current robot pose");
  m_currentPose = getCurrentPose();
  ROS_INFO_STREAM("Current robot pose: " << m_currentPose);
  m_out << "Current robot pose (x,y,yaw): " << m_currentPose << std::endl << std::endl;

  //Verify that we are set to pause
  ROS_ASSERT(m_pause == true);

  //Capture and store the latest depth image, RGB image, and point-cloud.
  //This is done here once, and for the remainder of this loop, all 3 of those
  // images will be used together. This guarantees that the images being processed
  // by Yolo match the depth images being used for depth information.
  //The RGB image will be continuously broadcast specially for Yolo consumption

  ros::Duration maxWait(1.0); //Wait up to this amount of time for images/point cloud
  //raw image 
  boost::shared_ptr<sensor_msgs::Image const> rawImagePtr =
    ros::topic::waitForMessage<sensor_msgs::Image>(m_imageTopic, maxWait);
  if(rawImagePtr == NULL)
    ROS_ERROR_STREAM("No raw image messages received on " << m_imageTopic);
  else
    m_latestRGBImage = *rawImagePtr;
  //pointcloud image
  boost::shared_ptr<sensor_msgs::PointCloud2 const> pointCloudPtr =
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD, maxWait);
  if(pointCloudPtr == NULL)
    ROS_ERROR_STREAM("No point clound messages received on " << TOPIC_POINT_CLOUD);
  else
    m_latestPointCloud = *pointCloudPtr;

  ROS_INFO_STREAM("Most recent images captured");

  //Begin broadcasting the most recent RGB image for Yolo to pick up
  //Do this on a thread which will be destroyed at the end of this scope and
  // therefore stop broadcasting
  ROS_INFO("Launching broadcast image thread");
  std::thread broadcastThread(&InventoryClerk::broadcastLatestImage, this);
  ROS_INFO("Broadcast image thread launched");

  //Avhishek - needs to adjusted for Python yolo timing - 4sec on avg

  // **************************************************************FIXED YOLO LAG ***************************************************************
  //Wait until Yolo catches up. Pause is currently true, so any object detection
  // callbacks are ignored during this nap time
  ROS_INFO_STREAM("Beginning nap for a time of " << (YOLO_IMAGE_REFRESH_LAG * NUM_REFRESHES_UNTIL_CURRENT) << " seconds");
  ros::Duration(YOLO_IMAGE_REFRESH_LAG * NUM_REFRESHES_UNTIL_CURRENT).sleep();

  //Now un-pause. We're ready to process whatever Yolo finds
  ROS_INFO_STREAM("Awoke from nap. Setting pause to false");
  m_pause = false;

  // Avhishek - DO this loop if there is no yolo image recieved
  //Loop until the next object detection and processing has occurred
  do {
    ROS_INFO_STREAM("InventoryClerk in loop waiting for image to be processed. Waiting 1 second");
    ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
    ros::spinOnce();
    ros::Duration(1.0).sleep();  //Sleep for a second
  }
  while(!m_YOLO_imageReceived);

  //Wait for the thread to finish
  broadcastThread.join();
  
  //Re-set to pause and image received status
  m_pause = true;
  m_YOLO_imageReceived = false;
  ROS_INFO_STREAM("InventoryClerk - finished inspecting position #" << positionNumber);
}

//****************************************************************************************************************************

void UNL_Robotics::InventoryClerk::objDetectionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  ROS_INFO_STREAM("InventoryClerk - object detected, in callback");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);

  //Set this flag to true to keep main thread from exiting before we are done processing
  // this object. Otherwise, the long-term classification might not be set yet and the
  // thread would continue prematurely
  m_currentlyProcessingObject = true;
  
  if(m_pause) {
    ROS_INFO_STREAM("InventoryClerk - currently paused, so abandoning callback");
    ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
    return;
  }

  ROS_INFO_STREAM("InventoryClerk - not paused so continuing with callback");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);

  
  std::string timeStamp = timestamp();
  unsigned numBoxes = msg->bounding_boxes.size();
  ROS_INFO_STREAM( "Detected " << numBoxes << " object" << (numBoxes > 1 ? "s" : "") );

  //Iterate over all the items that have been identified
  unsigned itemNum = 0;
  for(auto box : msg->bounding_boxes) {

    //Get out the object type and the bounding box information
    std::string objectName = box.Class;
    unsigned xmin = box.xmin;
    unsigned xmax = box.xmax;
    unsigned ymin = box.ymin;
    unsigned ymax = box.ymax;
    unsigned x_delta = xmax - xmin;
    unsigned y_delta = ymax - ymin; 
    ROS_INFO_STREAM("  " << objectName  << "  -  Probability " << std::setprecision(4) << (box.probability*100) << "%" );
    ROS_INFO_STREAM("    " << "BB Min (x,y) = (" << xmin << ", " << ymin << ")" );
    ROS_INFO_STREAM("    " << "BB Max (x,y) = (" << xmax << ", " << ymax << ")" );
    
    m_out << "*) Object type:  " << objectName << std::endl;
    m_out << "   Probability  " << std::setprecision(4) << (box.probability*100.0) << "%" << std::endl;


    // Avhishek - Don't know why  this is being done what is the use of calculating objectAngleOffset

    //Calculate the angle offset of the picture relative to the center of the view port
    unsigned x_centerBB = xmin + static_cast<unsigned>(x_delta/2);
    unsigned y_centerBB = ymin + static_cast<unsigned>(y_delta/2);
    int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH/2) - x_centerBB;   //Can be negative! This orientation assumes CCW=+
    double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

    // Avhishek - m_out is being used for printing 
    m_out << "   " << "Bounding Box (x,y):"
          << "   Min = (" << xmin << ", " << ymin << ")"
          << "   Max = (" << xmax << ", " << ymax << ")"
          << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
    m_out << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;


    // Avhishek - 
    //Convert the most recent ROS point cloud msg into a pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(m_latestPointCloud, *pclCloud);

    //Assert a few things about our point cloud and RGB images. They should match sizes.
    //If not, our bounding box values will not correspond
    // Avhishek - 
    ROS_ASSERT(m_latestRGBImage.height ==  pclCloud->height);
    ROS_ASSERT(m_latestRGBImage.width  ==  pclCloud->width);


    //Do long-term object classification
    if((objectName == "sofa") || (objectName == "bench") || (objectName == "door")) {
      
      m_longTermObjectDetectedAtThisPosition = true;
    
      //Save the RGB image of the object as a jpeg file and the point-cloud as a PCD
      //First create the path based on the object name and its number in this series of bounding boxes
      // (in the case that there is more than 1 object of the same type in the image, this will be unique)
      std::stringstream ssObjName, ssObjPath;
      ssObjName << "item_" << itemNum << "_obj_" << objectName;
      ssObjPath << m_workingPath << timeStamp << "_" << ssObjName.str();
    
      //Call the crop and save function. Save only the object in this loop
      cropAndSaveImage(m_latestRGBImage, ssObjPath.str() + ".jpeg",
                       xmin, ymin, x_delta, y_delta);
    
      //Save the full room point cloud
      std::stringstream ssPCD;
      ssPCD << m_workingPath << timeStamp << "_fullPointCloud.pcd";
      pcl::PCDWriter writer;
      writer.write<pcl::PointXYZ>(ssPCD.str(), *pclCloud, false);

      BoundingBox boundingBox{xmin, xmax, ymin, ymax};
      std::vector<UNL_Robotics::Point2D> hullPoints;
      if(objectName == "door") {

        
        //Note: In order for doors to be recognized, you  **MUST**  use a custom version
        //      of Yolo that is trained for doors.  If this is not installed, door
        //      recognition will not work.

        
        //DoorSegmentation segmenter(ssObjPath.str(), boundingBox, pclCloud);
        //segmenter.doPlaneExtraction(normal, normalThreshold);
        //segmenter.extractObjectInBoundingBox(cropPercentage);
        //segmenter.removeOutliers(meanK, stddevMulThresh);
        //segmenter.performEuclideanExtraction();
        //hullPoints = segmenter.calcBoundingBoxInWorldCoords(m_currentPose.x,
        //                                                    m_currentPose.y,
        //                                                    m_currentPose.yaw);
      }
      else {
        //Extract the object PCL knowing the bounding box
        SegmentationPipeline segmenter(ssObjPath.str(), boundingBox, pclCloud);
        segmenter.doPlaneExtraction(normal, normalThreshold);
        segmenter.extractObjectInBoundingBox(cropPercentage);
        segmenter.removeOutliers(meanK, stddevMulThresh);
        segmenter.performEuclideanExtraction();
        bool visualizeBB = false;
        hullPoints = segmenter.calcBoundingBoxInWorldCoords(visualizeBB,
                                                            m_currentPose.x,
                                                            m_currentPose.y,
                                                            m_currentPose.yaw);
      }
      LongTermObject lto {objectName, hullPoints};
      m_longTermObjects.push_back(lto);
    }
    
    //Increment the item number
    ++itemNum;    
  }
 
  //We're now finished processing this object, so set the processing flag to false
  //This is checked in the main thread to ensure we're not moving on before this is done.
  m_currentlyProcessingObject = false;
  
  ROS_INFO_STREAM("InventoryClerk - finished processing object");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);
}

//****************************************************************************************************************************

/*
  Avhishek - 

*/

void UNL_Robotics::InventoryClerk::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO_STREAM("InventoryClerk - detection image callback called");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);

  if(m_pause) {
    ROS_INFO_STREAM("InventoryClerk - currently paused. Returning");
    return;
  }

  //Save off the full image for this observation
  std::string timeStamp = timestamp();
  std::stringstream ssFullPath;
  ssFullPath << m_workingPath << timeStamp << "_fullImage.jpeg";
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(m_latestRGBImage, sensor_msgs::image_encodings::BGR8);
    imwrite(ssFullPath.str(), cv_ptr->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("cv_bridge exception. What = " << e.what());
    return;
  }
  ROS_INFO_STREAM("Saved full image to file: " << ssFullPath.str());

  //And wait until any object processing (CV) has finished
  ros::Rate loop_rate(10);
  while(m_currentlyProcessingObject)    //Until object being processed is finished
  {
    loop_rate.sleep();
  }
 
  //Set the yolo image received to indicate we just received an image during an un-paused period.
  //As soon as this is set to true, the main thread will continue
  ROS_INFO_STREAM("InventoryClerk - finished detection image callback. Setting image received flag to true");
  m_YOLO_imageReceived = true;
  
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived <<
                  "m_currentlyProcessingObject = " << m_currentlyProcessingObject);
}

//****************************************************************************************************************************
/*
  Avhishek - publishes the latest image 
*/
void UNL_Robotics::InventoryClerk::broadcastLatestImage()
{
  //This function is generally called on a separate thread and allowed to loop until
  // the m_YOLO_imageReceived bool flag is flipped.
  
  double rate(10.0);  //Broadcast at 10 Hz
  ROS_INFO_STREAM("Thread - beginning broadcast of image at a rate of " << rate << " Hz");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);

  ros::Rate loop_rate(rate);
  while(ros::ok()  &&  !m_YOLO_imageReceived)    //Until an image has been received during an un-paused period
  {
    m_yoloImagePublisher.publish(m_latestRGBImage);
    loop_rate.sleep();
  }
  
  ROS_INFO("Thread - finished broadcast of image");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
}


//***************************************************************************************************************************************************

UNL_Robotics::InventoryClerk::InventoryClerk(ros::NodeHandle& nodeHandle,
                                             const std::string& workingPath,
                                             const std::string& inventoryFilename,
                                             bool beginPaused,
                                             bool useDownSampling)
  : m_out((workingPath + inventoryFilename).c_str()),
    m_workingPath(workingPath),
    m_imageTopic(DEFAULT_IMAGE_TOPIC),
    m_pause(beginPaused),
    m_YOLO_imageReceived(false),
    m_currentlyProcessingObject(false),
    m_useDownSampling(useDownSampling)
{
  std::cout << "IC 1" << std::endl;s
  ROS_INFO_STREAM("IC " << 1);
  
  if(!m_out) {
    std::cout << inventoryFilename << " output file not opened successfully. "
              << "Likely you need to set ROS_HOME or launch this node from the unl_smart_robotic_home directory" << std::endl;
    exit(EXIT_FAILURE);
  }
  //Avhishek - this is where the subscriptions are happening (darknet BB, darknet image)
  m_objectDetectionSubscriber = nodeHandle.subscribe(TOPIC_DARKNET_BB, QUEUE, &InventoryClerk::objDetectionCallback, this); //bbcord.msg
  m_detectionImageSubscriber = nodeHandle.subscribe(TOPIC_DARKNET_IMAGE, QUEUE, &InventoryClerk::detectionImageCallback, this); // H_detection_img
  //Avhishek - publisher here that publishes Yolo detected image
  m_yoloImagePublisher = nodeHandle.advertise<sensor_msgs::Image>(TOPIC_YOLO_IMAGE, 1);
    
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
}

// **************************************************************************************************************************************