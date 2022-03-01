#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
# import matplotlib.pyplot as plt
# try:
#     import cv2
# except ImportError:
#     import sys
#     ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
#     sys.path.remove(ros_path)
#     import cv2
#     sys.path.append(ros_path)
from cv_bridge import CvBridge

from robot.msg  import H_detection
# import yolo as Yolo


rospy.init_node('Human_Detection', anonymous = True)

# pub_Image = rospy.Publisher('Human', Image, queue_size=10)

msg_pub = rospy.Publisher("H_Detection_msg", H_detection, queue_size=1)



bridge = CvBridge()  
msg = H_detection()
human_flag = 1
if human_flag == 1:
    rospy.loginfo_once("Human Detected on Camera")
    msg.signal = 1 
    msg_pub.publish(msg)

# def yolo_processing(cv_img):       
#     ''' yolo processing node computes detection 
#         and returns new image with detection and 
#         human_flag which turns true if the human is detected
#     '''
#     yolo_output = Yolo.Yolo_imp(cv_img)
#     output = bridge.cv2_to_imgmsg(yolo_output)
#     human_flag = 0
#     '''
#     Add a custom msg called Human Detected -
#     It is published if a human is detected 
#     '''

#     while not rospy.is_shutdown():
#         pub_Image.publish(output)
#         # if human_flag is 1:
#         #     rospy.loginfo_once("Human Detected on Camera")
#         #     msg.signal = 1 
#         #     msg_pub.publish(msg)

        
# def image_callback(msg: Image):
#     cv_img =  bridge.imgmsg_to_cv2(msg)
#     yolo_processing(cv_img)

        
def main():
    rospy.loginfo("I am human detection node.")
    ### Depth Camera Input Subscribers
    # rospt.Subscriber("/camera", Image, image_callback)
    

    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass
    
    rospy.spin()