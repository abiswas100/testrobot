// #include "ros/ros.h"
// #include "std_msgs/String.h"


// #include "segmentation_pipeline.h"
// #include "pclUtils.h"
// #include "gaussKernel.h"
// #include "convexHull.h"
// //OpenCV
// #include <cv_bridge/cv_bridge.h>
// // #include <opencv2/imgproc/imgproc.hpp>
// // #include <opencv2/highgui/highgui.hpp>
// //pcl
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/common/common.h>
// //PCL - PCA
// #include <pcl/common/pca.h>
// #include <pcl/features/moment_of_inertia_estimation.h>
// //PCL - visualization
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/point_cloud_color_handlers.h>
// //std
// #include <sstream>

// //Camera specs
// const unsigned CAMERA_NUM_PIXELS_WIDTH(640);
// const unsigned CAMERA_NUM_PIXELS_HEIGHT(480);
// const double   CAMERA_HORIZONTAL_VIEW_ANGLE(1.19);   //The angle (in radians) of what the camera views

// //For Gaussian convolution filter
// const unsigned KERNEL_RADIUS(5);                    //The Gaussian convolution radius
// const unsigned NUM_GAUSS_SAMPLE_PTS(KERNEL_RADIUS*2 + 1);   //How many sample points for depth sampling

// //PCL plane extraction - a hard threshold, especially for if something goes wrong or lots of small (insignificant) planes
// const unsigned MAX_PLANE_COUNT(8);
// const double PLANE_EXTRACTION_CONTINUE_THRESHOLD(0.30);   // While 30% of the original cloud is still there

// //Euclidean clustering
// double CLUSTER_TOLERANCE(0.10);
// unsigned MIN_CLUSTER_SIZE(10);





/*
I have added this so that it doesn't give error for main in catkin_make
*/

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%