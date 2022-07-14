#!/usr/bin/env python
#!/usr/bin/env python3

#imports 
from array import array
from cmath import sqrt
from os import device_encoding
# from cv2 import HOUGH_MULTI_SCALE
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
from kf import KF



from geometry_msgs.msg import Point, PointStamped, Point32

import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError
import yolo as Yolo
import numpy as np
import time
import math

bridge = CvBridge() 
class Detection(object):

    def __init__(self):
        
        self.mean = Point()
        self.cov_mat = []
        self.std_dev = Point()
        self.pos_mean_queue = []
        # t_now = rospy.Time.now()
        t_now = rospy.get_time()
        print(t_now)
        pos_mean = [0.00, 0.00,t_now]
        self.pos_mean_queue.append(pos_mean)
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=1)
        
                
        # publishing topics
        self.publish_mean = rospy.Publisher("mean", Meannn, queue_size=1)   
        self.publish_std_dev = rospy.Publisher("deviation", Deviation, queue_size=1)
        # self.publish_covar = rospy.Publisher("covar", pc2, queue_size=1) # Covariance matrix needs to be published as a covariance array
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
        
        # polygon = Polygon()
        # poly_stamped = PolygonStamped()
        Human_Marker_cube = Marker()
        
        
        pcl_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False) 
        
    
        #create a 2D array out of this pcl_np without the y values which are 0 after projection
        xzarray = []
        height, width = pcl_np.shape
        x_values = []
        z_values = []
        for i in range(0,height):
                point = pcl_np[i]
                x_values.append(point[0])
                z_values.append(point[2])
                value = [point[0], point[2]]
                xzarray.append(value)
        
        x_np_array = np.array(x_values)
        z_np_array = np.array(z_values)

        #use these 4 lines to plot X and Z points with ROS
        
        # plt.plot(x_np_array, z_np_array, 'x')
        # plt.axis("equal")
        # plt.draw()
        # plt.pause(0.0000000001)   
        
        # find mean for the XZ array
        xz_np_array = np.array(xzarray)
        mean2D = xz_np_array.mean(axis=1)   # axis  = 0 is for column, axis 1 for row and abs is to keep everything positive
        cov_xz = np.cov(xz_np_array)
        # print(mean2D, cov_xz)

        # Plot Gaussian 
        
        '''    Uncomment lines below to see gaussian plots
        '''
        # Y,Z= np.random.multivariate_normal(mean2D,cov_xz, 2)
        # plt.plot(Z)
        # plt.xlabel('Density')
        # plt.draw()
        # plt.pause(10)
        
        
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

        #plot the human's movement
        # plt.scatter(x,y)
        # plt.draw()
        # plt.pause(0.00001)
        
        
        # Publish a Cube Marker for the human
        
        if np.mean(x) == NaN or np.mean(z) == NaN:
            print("no human in view")
            
        else:
            
            meanx = round(np.mean(x),2)
            meanz = round(np.mean(z),2)
            t_now = rospy.get_time()
    
            pos_mean_now = [meanx, meanz,t_now]
            self.pos_mean_queue.append(pos_mean_now)
           
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
            '''
                find the velocity in x and z plane
            '''
            pos_mean_last = self.pos_mean_queue.pop(0)
            meanx_last, meanz_last,t_last = pos_mean_last[0],pos_mean_last[1],pos_mean_last[2]
            # print("Past meanx, meanz and time",meanx_last, meanz_last,t_last)
            
            # print("popped value",self.pos_mean_queue.pop(0))
            print("")
            dist_x, dist_z, time_diff = round(math.dist([meanx],[meanx_last]),2) , round(math.dist([meanz],[meanz_last]),2) , t_now - t_last  #time in seconds
            vx, vz = round((dist_x/time_diff),2), round((dist_z/time_diff),2)  # speed in m/sec
            
            print("Distance travelled in x and z and time_diff", dist_x, dist_z, time_diff)
            print("speed in x and z", vx, vz)  
            
        #******************   added by apala for kf   **************************************************
        
        # kf = KF(initial_x=0.0,initial_v=1.0,acc_variance=0.1)

        # DT = 0.2
        # NUM_STEPS = 1000
        # meas_every_step = 20

        # mus = []
        # covs = []

        # real_x = meanx #0.0 this should have our mean of cluster
        # real_v = 0.9
        # mea_variance = cov_xz #0.1 ** 2 //not sure if this is the covariance of cluster ; should be single value
        # real_xs = []
        # real_vs = []


        # for step in range(NUM_STEPS):
        #     covs.append(kf.cov)
        #     mus.append(kf.mean)
            
        #     real_x = real_x + DT * real_v
            
        #     kf.predict(dt=DT)
            
            
        #     if step != 0 and step % meas_every_step == 0:
        #         kf.update(meas_value=real_x + np.random.randn() * np.sqrt(mea_variance), 
        #                 meas_variance=mea_variance)
                
        #     real_xs.append(real_x)
        #     real_vs.append(real_v)
            
        #     print("something is printing",real_xs)
                    
                # real_xs.append(real_x)
                # real_vs.append(real_v)
                
                # print("something is printing",real_xs)
                        
        #***************************************************************************************        
                

def main():
    rospy.init_node('Gaussian_DBSCAN', anonymous=False)
    sn = Detection()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
##################### Apala code #################################

        # arr = np.array(pcl_np)
        # mean.mean_value = np.mean(arr)
        # blahh = arr.mean(axis=0)  # axis  = 0 is for column, axis 1 for row 
        # # blah has 3 values , xmean, ymean and z mean
        # # mean_x.mean_value = blahh[0]
        # # mean_y.mean_value = blahh[1]
        # # mean_z.mean_value = blahh[2]
        
        # # print(blahh)
        # print("mean size:", len(blahh))
        
        # # print("mean_x: ", mean_x.mean_value, "mean_y: " , mean_y.mean_value, "mean_z: ", mean_z.mean_value)
        
        # # print(arr)
        # print("size of original array is: " , len(arr),"X",len(arr[0]))
        
        # cov_mat = np.cov(arr)
        
        # std_dev.std_dev_value = np.std(arr)
        # duhh = arr.std(axis=0)
        # dev_x.std_dev_value = duhh[0]
        # dev_y.std_dev_value = duhh[1]
        # dev_z.std_dev_value = duhh[2]
        
        # print(duhh)
        
        # # print("dev_x: ",dev_x.std_dev_value, "dev_y: " , dev_y.std_dev_value, "dev_z: ", dev_z.std_dev_value)
        

        # # print("covariance matrix is ::", cov_mat)
       
        # print("size of cov_mat is: ", len(cov_mat),"X",len(cov_mat[0])) 
        # print("std deviation is: ", std_dev.std_dev_value)          
        # print("mean is: ", mean.mean_value)
        # # print()

        # #plot the gaussian function
        # x,y = np.random.multivariate_normal(blahh,cov_mat,2)#not taking more than 2, but example showed 5000
        # plt.plot(x,y,'x')
        # plt.axis('equal')
        # plt.show()
        
        
        # # s = np.random.normal(mean.mean_value,std_dev.std_dev_value,1000)
        # # sns.distplot(random.normal(size=1000), hist=False)
        # # plt.show()
        # # plt.close()
        
        
        
        
        # self.publish_mean.publish(mean)
        # self.publish_std_dev.publish(std_dev)
        # # self.publish_covar.publish(cov_mat)   

    