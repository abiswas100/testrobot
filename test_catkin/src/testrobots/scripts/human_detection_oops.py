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



class Detection(object):

    def __init__(self):
        self.central_pixel = 0
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback,queue_size=10)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.DepthCamSub, queue_size=1)
        # rospy.Subscriber("/camera/depth/points",pc2, Depthcloud, queue_size=1)

        # publishing topics
        self.pub = rospy.Publisher("H_Detection_image", Image, queue_size=10)    
        self.msg_pub = rospy.Publisher("H_Detection_msg", H_detection, queue_size=1)
    


    def yolo_processing(self,cv_img):       
        ''' 
        yolo processing node computes detection 
        and returns new image with detection and 
        human_flag which turns true if the human is detected
        '''
        msg = H_detection()
        msg.signal = -1
        yolo_output, object_label, center_pixels = Yolo.Yolo_imp(cv_img)
        
        try:
            self.center_pixel = center_pixels[0]
            print("center_pixel in yolo processing- ", self.center_pixel)
        except IndexError or AttributeError: 
            print("no center pixel in yolo_processing...")
            
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
        self.msg_pub.publish(msg)
        self.pub.publish(output)
    
        
    def image_callback(self,data):
        # print("here in callbaack")
        cv_img =  bridge.imgmsg_to_cv2(data)
        # print("Dimensions of camera img- ",cv_img.shape)
        self.yolo_processing(cv_img)

    def DepthCamSub(self,depth_data):
        depth_cv_img =  bridge.imgmsg_to_cv2(depth_data)
        # print("Dimensions - ",depth_cv_img.shape)
        # print("depth at -",depth_cv_img[400][235])
        try:
            print("center pixel in depthcam",self.center_pixel)
            print("distance of human in depthcam", depth_cv_img[self.center_pixel[1]][self.center_pixel[0]])
            rospy.sleep(0.5)
            # print()
        except AttributeError:
            print("no centers in depth")
def main():
    rospy.init_node('Human_Detection', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()