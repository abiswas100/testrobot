#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import matplotlib.pyplot as plt
try:
    import cv2
except ImportError:
    import sys
    ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
    sys.path.remove(ros_path)
    import cv2
    sys.path.append(ros_path)
from cv_bridge import CvBridge


import yolo as Yolo

hello_str = "I Track Humans"
rospy.init_node('Panel_Detection', anonymous = False)
pub_Image = rospy.Publisher('Panel', Image, queue_size=10)
pub = rospy.Publisher('Panel_Detection_', String, queue_size=10)
bridge = CvBridge()  


def image_processing_publish(cv_img):       
    yolo_output = Yolo.Yolo_imp(cv_img)
    print("for left")
    left_output = bridge.cv2_to_imgmsg(yolo_output)
    while not rospy.is_shutdown():
        pub_Image.publish(left_output)
        # will add this here when real implementation happens
        # pub.publish(hello_str)
        rospy.spin()   
        
def image_callback(msg: Image):
    cv_img =  bridge.imgmsg_to_cv2(msg)
    image_processing_publish(cv_img)

        
def main():
    
    ### Depth Camera Input Subscribers
    
    # rospy.Subscriber("/camera", Image, image_callback, queue_size=10)
    
    # rospy.Subscriber()
    try:
        rospy.Subscriber()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()