#!/usr/bin/env python
#!/usr/bin/env python3

from numpy import NaN
import rospy
import ros_numpy

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Header
from sensor_msgs import point_cloud2


from geometry_msgs.msg import Point, PointStamped

import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError
import yolo as Yolo
import numpy as np
import time

from testrobots.msg import newBoundingbox


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
        
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback,queue_size=5)        
        rospy.Subscriber("camera/depth/points", pc2, self.pointcallback, queue_size=1)
        
        # publishing topics
        self.pub = rospy.Publisher("H_Detection_image", Image, queue_size=1)    

        self.croppedpcl = rospy.Publisher("cropedPCL", pc2, queue_size=1)

        self.boundingbox = rospy.Publisher("Box_values", newBoundingbox, queue_size=100, latch=True)

    def pointcallback(self,data):
        print("Here in Pointcloud bounding Box extraction ---- ")
        print("")
        #use the same header and data as input message
        header = data.header
        #use this to generate finished pointcloud
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),
            ]
        
        # convert ROS_pointcloud into numpy array
        pcl_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False)  # remove_nans=True
        
    
        if len(self.center_pixel) == 0:
            print("no bounding box to extract")
            
        else:    
            
            # get all the corners of the box here 
            corner = self.corners[0]
            leftbottom_corner = corner[0]
            righttop_corner = corner[3]  
            
            
            # from the corners cut out the xmin, xmax, ymin,max values
            xmin = leftbottom_corner[0]
            xmax = righttop_corner[0]
            ymin = leftbottom_corner[1]
            ymax = righttop_corner[1]

            points = []
            no_of_points = 0
            
                        
            #  creating bounding box message for apala
            box = newBoundingbox()

            '''
                Part 1 - iterate over the pointcloud and cut the points of the box and make all the rest of the point NaN
                Part 2 - extract the point for the corners of the BB and publish it
                
                you can run any one of them or both of them
            '''
  
            
            start = time.time()
            for row_no in range(0,1079):
                    for col_no in range(0,1919):
                        row = pcl_np[row_no]
                        value = row[col_no]
                        
                        
                        if (row_no >= ymin and row_no <= ymax) and (col_no >= xmin and col_no <= xmax):
                            no_of_points += 1
                    
                            # print("row, col, value",row_no, col_no, value)       
                        else:
                            pcl_np[row_no][col_no] = NaN
                        
                        
                        # getting the values of each x,y,z point and creating their corresponding rgb values
                        x = value[0]
                        y = value[1]
                        z = value[2]
                    
                        try:
                            r = abs(int(x*255.0))
                            g = abs(int(x*255.0))
                            b = abs(int(x*255.0))
                        except ValueError:
                            r = 1 
                            g = 1
                            b = 1   
                        
                        a = 255
                        # print(r,g,b,a)
                        if (r >= 255 or b >= 255 or g >= 255):
                            r = 255 
                            g = 255
                            b =255
                        #creating a pointcloud sensor message type array with rgb values
                        try:
                            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                            # print (hex(rgb))
                        except struct.error:
                            print("error",r,g,b)
                        pt = [x, y, z, rgb]
                        points.append(pt)
                        
                        if(row_no == ymin and col_no == xmin):
                            box.A_x = x
                            box.A_y = y
                            box.A_z = z
                        if(row_no == ymax and col_no == xmin):
                            box.B_x = x
                            box.B_y = y
                            box.B_z = z
                        if(row_no == ymin and col_no == xmax):
                            box.C_x = x
                            box.C_y = y
                            box.C_z = z
                        if(row_no == ymax and col_no == xmax):
                            box.D_x = x
                            box.D_y = y
                            box.D_z = z
            
            seconds = time.time() - start
            # print('Time Taken:', time.strftime("%H:%M:%S",time.gmtime(seconds)))  # get the time of working for the pointcloud process
            #publishing pointcloud and bounding box points
            print("")
            print(box)
            print("")
            pc2 = point_cloud2.create_cloud(header, fields, points)
            
            self.croppedpcl.publish(pc2)
            self.boundingbox.publish(box)  
            

            

 
    '''
        This is a callback for RGB Image message
    '''       
    def image_callback(self,data):
        # print("here in callbaack")
        cv_img =  bridge.imgmsg_to_cv2(data)
        # print("Dimensions of camera img- ",cv_img.shape)
        tracking_img = cv_img

        self.yolo_processing(cv_img)

        
    '''
        This function calls Yolo and Yolo related data and other
    '''
    def yolo_processing(self,cv_img):       
        ''' 
        yolo processing node computes detection 
        and returns new image with detection and 
        human_flag which turns true if the human is detected
        '''

        #yolo returning center and corners
        yolo_output, object_label, center_pixels, self.corners, self.confidence = Yolo.Yolo_imp(cv_img)
        

        
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
            # rospy.logwarn("Human Detected on Camera")
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
            xmax = righttop_corner[0]
            ymin = leftbottom_corner[1]
            ymax = righttop_corner[1]
            
        
        #publish the message and the image
        self.pub.publish(output)
        
        # checking if center_pixels is empty and then setting the past center
        if len(self.center_pixel) == 0: pass
        else:
            self.queue_center.append(self.center_pixel)
            

    # def DepthCamSub(self,depth_data):
        
       
    #     # depth_cv_img =  bridge.imgmsg_to_cv2(depth_data, '32FC1')
        
    #     depth_image = bridge.imgmsg_to_cv2(depth_data,desired_encoding='passthrough' )
        
    
    #     depth_array = np.array(depth_image, dtype=np.float32)
        
    #     # rospy.loginfo(depth_array)
        
    #     # depth_array = depth_array.astype(np.uint8)
    #     cv2.imwrite("depth_img.png", depth_array)
        
    #     if len(self.center_pixel) == 0:
    #          pass
            
    #     else:    
            
    #         print("Corners in Depth Camera Bounding Box",self.corners[0])
    #         # get all the corners of the box here 
    #         corner = self.corners[0]
    #         leftbottom_corner = corner[0]
    #         rightbottom_corner = corner[1]
    #         lefttop_corner = corner[2]
    #         righttop_corner = corner[3]  
            
            
    #         # from the corners cut out the xmin, xmax, ymin,max values
    #         xmin = leftbottom_corner[0]
    #         xmax = righttop_corner[0]
    #         ymin = leftbottom_corner[1]
    #         ymax = righttop_corner[1]

    #         # get the width and height of the box
    #         w = xmax - xmin
    #         h = ymax - ymin
        
    #         i = 0
    #         print(depth_array.shape) 
    #         # print(depth_img_cpy)
    #         totalrows, totalcols = depth_array.shape   ## 1080*1920
            
    #         for row_no in range(0,1079):
    #             for col_no in range(0,1919):
    #                 row = depth_array[row_no]
    #                 value = row[col_no]
                    
                    
    #                 if (row_no >= ymin and row_no <= ymax) and (col_no >= xmin and col_no <= xmax):
    #                     i += 1
    #                     # print("row, col, value",row_no, col_no, value)        
    #                     # depth_array[row_no][col_no] = 2.00  
    #                     pass
    #                 else:
    #                     depth_array[row_no][col_no] = NaN
    #         print("no of points in the bbox in depth_image",i)
            
    #         print(depth_array) 
    #         cv2.imwrite('depth_with_BBox.jpeg', depth_array)
    #         depth_bbox = bridge.cv2_to_imgmsg(depth_array)
    #         self.depth_with_BB.publish(depth_bbox)
            
    #         #creating PCL value using open3d
                    
    #         # pcl = o3d.geometry.PointCloud()
    #         # pcl.points = o3d.utility.Vector3dVector(np.random.randn(500,3))
    #         # o3d.visualization.draw_geometries([pcl])
    
 
   


def main():
    rospy.init_node('Depth_Image_Extract', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()