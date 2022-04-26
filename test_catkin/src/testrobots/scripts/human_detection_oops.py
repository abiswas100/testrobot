#!/usr/bin/env python
#!/usr/bin/env python3


from numpy import NaN
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2 as pc2
import csv
import cv2
from cv_bridge import CvBridge
import yolo as Yolo
import numpy as np
import os
import math

from testrobots.msg import H_detection
from testrobots.msg import stop  

bridge = CvBridge() 
class Detection(object):

    def __init__(self):
        self.corner_queue = []
        self.queue_center = []
        
        self.center_pixel = [] 
        self.corners = 0   # list containing lists of corners for current timestamp - recieved from 
        
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback,queue_size=1)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.DepthCamSub, queue_size=1)
        # rospy.Subscriber("/camera/depth/points",pc2, Depthcloud, queue_size=1)

        # publishing topics
        self.pub = rospy.Publisher("H_Detection_image", Image, queue_size=1)    
        self.msg_pub = rospy.Publisher("H_Detection_msg", H_detection, queue_size=1)
        self.stop_msg =  rospy.Publisher("Stop_msg", stop, queue_size=1)
        self.vector_pub = rospy.Publisher("H_Vector", Image, queue_size=1)
        
        #initialize csv file
        self.path = os.getcwd()
        self.path = self.path+"/human_motion.csv"
        # open the file in the write mode
        
        header = ['center_x', 'center_y', 'Distance']
        self.csv_file = open(self.path, 'w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(header)
        
    def image_callback(self,data):
        # print("here in callbaack")
        cv_img =  bridge.imgmsg_to_cv2(data)
        # print("Dimensions of camera img- ",cv_img.shape)
        tracking_img = cv_img

        self.yolo_processing(cv_img)
        
        self.human_motion_tracking(tracking_img)
        

    def yolo_processing(self,cv_img):       
        ''' 
        yolo processing node computes detection 
        and returns new image with detection and 
        human_flag which turns true if the human is detected
        '''
        msg = H_detection()
        msg.signal = -1
        
        #yolo returning center and corners
        yolo_output, object_label, center_pixels, self.corners = Yolo.Yolo_imp(cv_img)
        
        
        # checking center_pixels and setting center_pixel to 0         
        if len(center_pixels) == 0: 
            self.center_pixel = []
            print("no center pixel in yolo_processing...",self.center_pixel)
        else:
            self.center_pixel = center_pixels[0]
            

        #making the yolo output into a ros image version        
        output = bridge.cv2_to_imgmsg(yolo_output)
        
        '''
        Add a custom msg called Human Detected -
        It is published if a human is detected 
        '''
        # changing the msg value only if the label is == person
        if(object_label == 'person'):
            rospy.logwarn("Human Detected on Camera")
            msg.signal = 1 
            
        rospy.logwarn(msg.signal)
        
        #publish the message and the image
        self.msg_pub.publish(msg)
        self.pub.publish(output)

        # checking if center_pixels is empty and then setting the past center
        if len(self.center_pixel) == 0: pass
        else:
            self.queue_center.append(self.center_pixel)
            self.corner_queue.append(self.corners[0])
        
            
    def human_motion_tracking(self, tracking_img):
        
        if len(self.center_pixel) == 0: 
            output = bridge.cv2_to_imgmsg(tracking_img)
            self.vector_pub.publish(output)

        else:
            print("current center pixel in human tracking",self.center_pixel)    
            center_x = self.center_pixel[1]
            center_y = self.center_pixel[0]
            
            # check if stack is empty 
            if len(self.queue_center) == 0: pass
            else: 
                past = self.queue_center[0]
                # print("past center value", past)
                
                print("")
                
                for i in self.corner_queue:
                    print("self. corner_queue in Human Tracking",i)
                    
                ''' Getting the last last corner value    
                    this is done when the queue is just 
                    starting and there is no penaltimate value
                '''
                
                if len(self.corner_queue) > 2 or len(self.corner_queue) == 2:  
                    past_corner = self.corner_queue[len(self.corner_queue) - 2] 
                else:
                    past_corner = self.corner_queue[0]
                
                current_corner = self.corners[0]
                
                # print("")
                # print("past corner value", past_corner)
                
                past_leftbottom_corner = past_corner[0]
                past_rightbottom_corner = past_corner[1]
                past_lefttop_corner = past_corner[2]
                past_righttop_corner = past_corner[3]                        
                
                # past_width = past_leftbottom_corner[0] - past_rightbottom_corner[0]
            
                
                # Get the current corners from the current_corner                 
                
                current_leftbottom_corner = current_corner[0]
                current_rightbottom_corner = current_corner[1]
                current_lefttop_corner = current_corner[2]
                current_righttop_corner = current_corner[3]                        

                current_width = int(current_leftbottom_corner[0]) - int(current_rightbottom_corner[0])
                print("current width w1", math.ceil(current_width))
                

            past_center_x = past[1]
            past_center_y = past[0]
            
            
            # Computer Vision Part - draw the arrow on the image            
            start_point = (center_x,center_y)
        
            end_point = (past_center_x, past_center_y)
            
            # color and thickness in BGR for Computer Vision
            bbcenter = (0,0,0)
            color = (0, 255 , 0)
            # colors = [(235,14,202),(67,232,25), (232,25,25), (14, 235, 235),(37,33,255)]
            thickness = 20
            
            #pop the previous corner and center values
            if len(self.corner_queue) > 5:     
                popped_corner = self.corner_queue.pop(0) # the value is used so now  deleteing the last value
                popper_center = self.queue_center.pop(0)
                print("")
            
            # image = cv2.arrowedLine(tracking_img, end_point, start_point,
            #                             color, thickness)
            
            # drawing a circle at the center
            image = cv2.circle(tracking_img,start_point, 10, bbcenter, thickness)

            # drawing the past 5 bounding boxes
            for i in range(len(self.corner_queue) - 1):
                corner = self.corner_queue[i]
                leftbottom_corner = corner[0]
                righttop_corner = corner[3]
                
                print(leftbottom_corner,righttop_corner)
                
                # color = colors[i]
                image = cv2.rectangle(image,leftbottom_corner, righttop_corner, color, 15)    
                

            #converting to ROS format and publishing 
            output = bridge.cv2_to_imgmsg(image)
                        
            self.vector_pub.publish(output)


    def DepthCamSub(self,depth_data):
        depth_cv_img =  bridge.imgmsg_to_cv2(depth_data)

        if len(self.center_pixel) == 0:
            print("no centers in depth")
            rospy.sleep(0.5)
            
        else:
            print("center_pixel in depth- ", self.center_pixel, "length of center pixel", len(self.center_pixel))
            # print("center pixel in depthcam",self.center_pixel)
            center_x = self.center_pixel[1]
            center_y = self.center_pixel[0]

            depth = depth_cv_img[self.center_pixel[1]][self.center_pixel[0]]
            
            msg = stop()
            msg.stop = -1            
            
            #changing Nan Values to 0
            if depth == "nan":
                depth = 0
            
            
            data_to_write = [center_x,center_y,depth]

            self.writer.writerow(data_to_write)
            

            print("distance of human in depthcam - ", depth_cv_img[self.center_pixel[1]][self.center_pixel[0]])
            
            if depth <= 1.5 : 
                rospy.logfatal("Human too close ... Stop Immediately")
                msg.stop = 1
                rospy.logwarn(msg.stop)            
            
            self.stop_msg.publish(msg)            
            print("stop signal value", msg.stop)
            rospy.sleep(0.5)
            
            
def main():
    rospy.init_node('Human_Detection', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()