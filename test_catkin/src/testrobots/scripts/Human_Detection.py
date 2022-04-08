#!/usr/bin/env python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2 as pc2
import csv
import cv2
import pyrealsense2
import pcl_ros 
import pcl
from cv_bridge import CvBridge
import yolo as Yolo
import numpy as np
from testrobots.msg import H_detection  
bridge = CvBridge()  

center_pixel = []


pub = rospy.Publisher("H_Detection_image", Image, queue_size=10)    
msg_pub = rospy.Publisher("H_Detection_msg", H_detection, queue_size=1)

def yolo_processing(cv_img):       
    ''' yolo processing node computes detection 
        and returns new image with detection and 
        human_flag which turns true if the human is detected
    '''
    msg = H_detection()
    msg.signal = -1
    yolo_output, object_label, center_pixels = Yolo.Yolo_imp(cv_img)
    print("image dimensions")
    try:
        center_pixel = center_pixels[0]
        print("center_pixel - ", center_pixel)
    except IndexError: 
        print("no center pixel ...")
        
    output = bridge.cv2_to_imgmsg(yolo_output)
    
    '''
    Add a custom msg called Human Detected -
    It is published if a human is detected 
    '''
    if(object_label == 'person'):
        rospy.logwarn("Human Detected on Camera")
        msg.signal = 1 
        
    rospy.logwarn(msg.signal)
    
    #publish the message and the image
    msg_pub.publish(msg)
    pub.publish(output)
    
        
def image_callback(data):
    # print("here in callbaack")
    cv_img =  bridge.imgmsg_to_cv2(data)
    # print("Dimensions of camera img- ",cv_img.shape)
    yolo_processing(cv_img)

def DepthCamSub(data):
    depth_cv_img =  bridge.imgmsg_to_cv2(data)
    print("Dimensions - ",depth_cv_img.shape)
    # print(depth_cv_img[400][235])
    print(center_pixel[1],center_pixel[0])
    # print(depth_cv_img(center_pixel[0],center_pixel[1]))    
    print("---------------------------------------------------------------------------------")

def Depthcloud(msg ):
    points_list = []
    for data in msg.data:
        # print(data)
        points_list.append(data)
    
    # print(len(points_list))

    #set the data into an array of 61440, 1080
    a = np.array(points_list)
    points_array = np.reshape(a,(61440,1080))
    # print(np.shape(points_array))
    # print(points_array)
    
    
    
    
def main():
    rospy.init_node('Human_Detection', anonymous=False)
    
    ### Depth Camera Input Subscribers
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback,queue_size=10)
    # rospy.Subscriber("/camera/depth/image_raw", Image, DepthCamSub)
    # rospy.Subscriber("/camera/depth/points",pc2, Depthcloud, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()
if __name__ == "__main__":
    # try:
    #     talker()
    # except rospy.ROSInterruptException: pass

    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    




# bridge = CvBridge()  

        
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
