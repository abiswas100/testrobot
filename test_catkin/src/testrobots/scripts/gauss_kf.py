#!/usr/bin/env python
#!/usr/bin/env python3

#imports 
from array import array
from cmath import sqrt
from os import device_encoding
from cv2 import HOUGH_MULTI_SCALE
from numpy import NaN, cov
# from torch import uint8
import rospy
import ros_numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import seaborn as sns
from sklearn.cluster import DBSCAN
from numpy import random

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Polygon, PolygonStamped
# will try later
from visualization_msgs.msg import Marker
# from geometry_msgs import PoseWithCovarianceStamped
from testrobots.msg import Meannn
from testrobots.msg import Deviation
from testrobots.msg import Velocity
from kf import KF



from geometry_msgs.msg import Point, PointStamped, Point32

import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError
import yolo as Yolo
import numpy as np
import time

bridge = CvBridge() 
class Detection(object):

    def __init__(self):
        
        self.mean = Point()
        self.cov_mat = []
        self.std_dev = Point()
        
        
        self.pointcloud_queue = []

        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=1)
        
                
        # publishing topics
        self.pub_mean_x = rospy.Publisher("mean_x", Meannn, queue_size=1)   
        self.pub_mean_y = rospy.Publisher("mean_y", Meannn, queue_size=1)  
        self.pub_mean_z = rospy.Publisher("mean_z", Meannn, queue_size=1)  
        
        self.pub_vel_x = rospy.Publisher("vel_x",Velocity,queue_size=1)
        self.pub_vel_y = rospy.Publisher("vel_y",Velocity,queue_size=1)
        self.pub_vel_z = rospy.Publisher("vel_z",Velocity,queue_size=1)
        
        self.publish_std_dev = rospy.Publisher("deviation", Deviation, queue_size=1)
        
        # self.publish_covar = rospy.Publisher("covar", pc2, queue_size=1) # Covariance matrix needs to be published as a covariance array
        self.human_polygon = rospy.Publisher("Human_polygon", PolygonStamped, queue_size=1)
        self.human_marker = rospy.Publisher("Human_marker", Marker, queue_size=2)
    
    def cloud_callback(self,data):
        print("Here in Pointcloud callback.........................................")
        header = data.header
        
        # creating the mean and covariance messages to publish 
        mean = Meannn()
        mean_x = Meannn()
        mean_y = Meannn()
        mean_z = Meannn()
        std_dev = Deviation()
        dev_x = Deviation()
        dev_y = Deviation()
        dev_z = Deviation()
        vel_x = Velocity()
        vel_y = Velocity()
        vel_z = Velocity()
        
       
        Human_Marker_cube = Marker()
        
        
        pcl_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False) 
        
    
        #create a 2D array out of this pcl_np without the y values which are 0 after projection
        xzarray = []
        height, width = pcl_np.shape
        x_values = []
        z_values = []
        for i in range(0,height):
                point = pcl_np[i]
                x = point[0]
                z = point[2]
                x_values.append(x)
                z_values.append(z)
                value = [x,z]
                xzarray.append(value)
        
        x_np_array = np.array(x_values)
        z_np_array = np.array(z_values)

        mean_x = x_np_array.mean(axis=0)
        mean_z = z_np_array.mean(axis=0)
    
       
        
        mean_x_array = abs(x_np_array.mean(axis=0))
        mean_z_array = z_np_array.mean(axis=0)        
        cov_x = np.cov(x_np_array)
        cov_z = np.cov(z_np_array)
        
        # print("mean x", mean_x_array, "", "mean_z", mean_z_array)
        # print("Covariance x", cov_x, "", "Covariance_z", cov_z)
        
        
        # find mean for the XZ array
        xz_np_array = np.array(xzarray)
        mean2D = xz_np_array.mean(axis=1)   # axis  = 0 is for column, axis 1 for row and abs is to keep everything positive
        cov_xz = np.cov(xz_np_array)
        # cov_xz = np.cov(x_np_array,z_np_array)
        # print(mean2D, cov_xz)
        # Plot Gaussian 
        
        Y,Z= np.random.multivariate_normal(mean2D,cov_xz, 2)
        # print(X)
        # print(Y)
        # print(Z)
        plt.plot(Z)
        plt.xlabel('Density')
        plt.draw()
        plt.pause(10)
        
        
        # compute DBSCAN - change eps and min_samples as required, eps- min distance between points
        # learn more from - https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
        DBSCAN_cluster = DBSCAN(eps=0.5, min_samples=30).fit(xz_np_array)
        labels = DBSCAN_cluster.labels_
        components = DBSCAN_cluster.components_ #copy of each core sample found by training
        feature = DBSCAN_cluster.n_features_in_ #number of features seen during fit
        
        # useful cluster has human with label 0, might change if more humans are added, x and y are points needed for plotting        useful_cluster = []
        useful_cluster = []
        x = []
        z = []
        for i in range(len(components)):
            if labels[i] == 0 :
                useful_cluster.append(components[i])
                point = components[i]
                x.append(point[0])
                z.append(point[1])
                
        #publish mean 
        mean_y = 0.0
        self.pub_mean_x.publish(np.mean(x))
        self.pub_mean_y.publish(mean_y)
        self.pub_mean_z.publish(np.mean(z))

        
        if np.mean(x) == NaN or np.mean(z) == NaN:
            print("no human in view")
            pass
        else:
            
            meanx = np.mean(x)
            meanz = np.mean(z)
            Human_Marker_cube.header.frame_id = "camera_rgb_optical_frame"
            Human_Marker_cube.header.stamp = rospy.Time.now()
            Human_Marker_cube.ns = "basic_shapes"
            Human_Marker_cube.id = 1
            Human_Marker_cube.type = 1
            Human_Marker_cube.pose.position.x =  meanx
            Human_Marker_cube.pose.position.y = 0.0
            Human_Marker_cube.pose.position.z = meanz
            Human_Marker_cube.pose.orientation.x = 1.0
            Human_Marker_cube.pose.orientation.y = 1.0
            Human_Marker_cube.pose.orientation.z = 0.0
            Human_Marker_cube.pose.orientation.w = 0.0
            Human_Marker_cube.scale.x = 0.8
            Human_Marker_cube.scale.y = 0.8
            Human_Marker_cube.scale.z = 0.8
            Human_Marker_cube.color.a = 1.0
            Human_Marker_cube.color.r = 1.0
            Human_Marker_cube.color.g = 0.0
            Human_Marker_cube.color.b = 0.0
            
            
            # while not rospy.is_shutdown():
            self.human_marker.publish(Human_Marker_cube)
            
        #calculate velocity x,y,z
        
        vel_y = 0.0
        vel_x
        vel_z
        
        
        #publish velocity
        self.pub_vel_x.publish(vel_x)
        self.pub_vel_y.publish(vel_y)
        self.pub_vel_z.publish(vel_z)
        
            
    
            
           
                

def main():
    rospy.init_node('Gaussian_DBSCAN', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
    
 