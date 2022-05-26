#!/usr/bin/env python
#!/usr/bin/env python3


from numpy import NaN
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2 as pc2

import tf, tf2_ros
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
import csv
import cv2
from cv_bridge import CvBridge
import yolo as Yolo
import numpy as np
import os
import math
import pcl
import pcl_ros
import ros_numpy
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh


from testrobots.msg import H_detection
from testrobots.msg import stop  
from testrobots.msg import BoundingBox
from testrobots.msg import BoundBoxes


bridge = CvBridge() 
class Detection(object):

    def __init__(self):
        self.corner_queue = []
        self.queue_center = []
        
        self.center_pixel = [] 
        self.corners = 0   # list containing lists of corners for current timestamp - recieved from 
        
        self.center_depth_point = Point()
        self.depth = 0
        self.confidence = 0.0
        
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback,queue_size=1)
        # rospy.Subscriber("/camera/depth/image_raw", Image, self.DepthCamSub, queue_size=1)
        # rospy.Subscriber("/camera/depth/points",pc2, self.Depthcloud, queue_size=1)
        # rospy.Subscriber("/map",OccupancyGrid,self.Occupancy, queue_size=1)
        # publishing topics
        self.pub = rospy.Publisher("H_Detection_image", Image, queue_size=1)    
        self.msg_pub = rospy.Publisher("H_Detection_msg", H_detection, queue_size=1)
        self.stop_msg =  rospy.Publisher("Stop_msg", stop, queue_size=1)
        self.vector_pub = rospy.Publisher("H_Vector", Image, queue_size=1)
        # self.point_pub = rospy.Publisher('/center_point', PointStamped, queue_size=1)
        self.boundingboxes = rospy.Publisher("BBox", BoundBoxes, queue_size=1)
        
        #initialize csv file
        self.path = os.getcwd()
        self.path = self.path+"/human_motion.csv"
        # open the file in the write mode
        
        header = ['center_x', 'center_y', 'Distance']
        self.csv_file = open(self.path, 'w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(header)
    
    '''
        This is a callback for PointCloud message
    '''    

    def Depthcloud(self,msg):
        # points_list = []
        # for data in msg.data:
        #     # print(data)
        #     points_list.append(data)
        # print(len(points_list))
        # print(len(points_list))

        # pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)  # remove_nans=True
        # print(np.shape(pc_np))
        # # print(pc_np)
        # o3dpc = orh.rospc_to_o3dpc(msg) 
        pass
              
        
    '''
        This is a callback for Occupancy Grid message
    '''
    def Occupancy(self,msg):
        # occupancy  =  ros_numpy.occupancy_grid.occupancygrid_to_numpy(msg)
        # print(occupancy)
        # print(np.shape(o
        # ccupancy))
        # print(msg.MapMetaData)
        pass

    
    '''
        This is a callback for RGB Image message
    '''

    def image_callback(self,data):
        # print("here in callbaack")
        cv_img =  bridge.imgmsg_to_cv2(data)
        # print("Dimensions of camera img- ",cv_img.shape)
        tracking_img = cv_img

        self.yolo_processing(cv_img)
        
        # self.human_motion_tracking(tracking_img)
        

    def yolo_processing(self,cv_img):       
        ''' 
        yolo processing node computes detection 
        and returns new image with detection and 
        human_flag which turns true if the human is detected
        '''
        # defining msgs for publishing
        msg = H_detection()
        msg.signal = -1
        
        #defining message to create message for BoundingBox in Cpp        
        bbcordmsg = BoundingBox()
        bbcordmsgs = BoundBoxes()
        #yolo returning center and corners
        yolo_output, object_label, center_pixels, self.corners, self.confidence = Yolo.Yolo_imp(cv_img)
        
       
        print(self.corners)
        
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
            # if human is detected find the transform and the point
            
            '''
            change made for the time being
            '''
            # open the corner pixel and get the xmin,xmax, ymin, max this is only done and published when there is a human   
            
            corner = self.corners[0]
            leftbottom_corner = corner[0]
            rightbottom_corner = corner[1]
            lefttop_corner = corner[2]
            righttop_corner = corner[3]  
            
            xmin = leftbottom_corner[0]
            xmax = rightbottom_corner[0]
            ymin = lefttop_corner[1]
            ymax = righttop_corner[1]
            
            #complete the bbcord message now
            
            bbcordmsg.Class = object_label
            bbcordmsg.probability = self.confidence
            bbcordmsg.xmin = xmin
            bbcordmsg.xmax = xmax
            bbcordmsg.ymin = ymin
            bbcordmsg.ymax = ymax
              
            bbcordmsgs.bounding_boxes = bbcordmsg
            # self.transform_depth() # this is to call the transform function 
            msg.signal = 1 
            
            
        rospy.logwarn(msg.signal)
        
        #publish the message and the image
        self.msg_pub.publish(msg)
        self.pub.publish(output)
        self.boundingboxes.publish(bbcordmsgs)
        
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
                
                # for i in self.corner_queue:
                #     print("self. corner_queue in Human Tracking",i)
                    
                ''' Getting the last last corner value    
                    this is done when the queue is just 
                    starting and there is no penaltimate value
                '''
                
                if len(self.corner_queue) > 2 or len(self.corner_queue) == 2:  
                    past_corner = self.corner_queue[len(self.corner_queue) - 2] 
                else:
                    past_corner = self.corner_queue[0]
                
                current_corner = self.corners[0]
                
                
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
                print("current width w1", abs(current_width))
                

            past_center_x = past[1]
            past_center_y = past[0]
            
            
            # Computer Vision Part - draw the arrow on the image            
            start_point = (center_x,center_y)
        
            end_point = (past_center_x, past_center_y)
            
            # color and thickness in BGR for Computer Vision
            bbcenter = (0,0,0)
            color = (0, 255 , 0)
            # colors = [(235,14,202),(67,232,25), (232,25,25), (14, 235, 235),(37,33,255)]
            colors = [(0, 255 , 0),(0, 215 , 0),(0, 185 , 0),(0, 145 , 0),(0, 145, 0)]
            thickness = 20
            
            #pop the previous corner and center values
            if len(self.corner_queue) > 5:     
                popped_corner = self.corner_queue.pop(0) # the value is used so now  deleteing the last value
                popper_center = self.queue_center.pop(0)
                print("")
            
            # image = cv2.arrowedLine(tracking_img, end_point, start_point,
            #                             color, thickness)
            
            # drawing a circle at the center
            image = cv2.circle(tracking_img,start_point, 10, bbcenter, 1)

            # drawing the past 5 bounding boxes
            for i in range(len(self.corner_queue) - 1):
                corner = self.corner_queue[i]
                leftbottom_corner = corner[0]
                righttop_corner = corner[3]
                
                
                color = colors[i]
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

            self.depth = depth_cv_img[self.center_pixel[1]][self.center_pixel[0]]
            
            msg = stop()
            msg.stop = -1            
            
            #changing Nan Values to 0 
            if self.depth == "nan":
                self.depth = 0
            if (math.isnan(self.depth)):self.depth = 0
                                    
            data_to_write = [center_x,center_y,self.depth]

            self.writer.writerow(data_to_write)
            
            
            print("distance of human in depthcam - ", self.depth)
            
            if self.depth <= 1.5 : 
                rospy.logfatal("Human too close ... Stop Immediately")
                msg.stop = 1
                rospy.logwarn(msg.stop)            
            
            self.stop_msg.publish(msg)            
            rospy.sleep(0.5)
    
    def transform_depth(self):       
        listener = tf.TransformListener()
        try:
            listener.waitForTransform("map","camera_depth_optical_frame", rospy.Time(10), rospy.Duration(1.0))
        except (tf2_ros.TransformException,tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as  e:
                print("transform error in waitfortransform",e)
                
                        
        rate = rospy.Rate(10)
        self.center_depth_point.x = self.depth
        # while not rospy.is_shutdown():
            
            # self.distance_point = PointStamped()
            # self.distance_point.header.frame_id = "/map"
            # self.distance_point.header.stamp = rospy.Time(0)
            # self.distance_point.point = self.center_depth_point
            
        point_msg = PointStamped()
        point_msg.header.frame_id = "camera_depth_optical_frame"
        point_msg.header.stamp = rospy.Time(0)
        point_msg.point = self.center_depth_point
        
        
        
        try:
            p = listener.transformPoint('map',point_msg) 
            print("Point message transformed",p)
            self.point_pub.publish(p)    
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("transform error",e)
            pass
                
        
    
    
             
            
def main():
    rospy.init_node('Human_Detection', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()