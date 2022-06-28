#!/usr/bin/env python
#!/usr/bin/env python3

from array import array
from cmath import sqrt
from os import device_encoding
from numpy import NaN, cov
import rospy
import ros_numpy
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.cluster import DBSCAN
from numpy import random

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import PointField
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from testrobots.msg import Meannn
from testrobots.msg import Deviation



from geometry_msgs.msg import Point, PointStamped

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
       
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=1)
        
                
        # publishing topics
        self.publish_mean = rospy.Publisher("mean", Meannn, queue_size=1)   
        self.publish_std_dev = rospy.Publisher("deviation", Deviation, queue_size=1)
        # self.publish_covar = rospy.Publisher("covar", pc2, queue_size=1) # Covariance matrix needs to be published as a covariance array

    def cloud_callback(self,data):
        print("Here in Pointcloud callback.........................................")
        
        
        # creating the mean and covariance messages to publish 
        mean = Meannn()
        mean_x = Meannn()
        mean_y = Meannn()
        mean_z = Meannn()
        std_dev = Deviation()
        dev_x = Deviation()
        dev_y = Deviation()
        dev_z = Deviation()
        
        
        pcl_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False) 
        
    
        #create a 2D array out of this pcl_np
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
    
        #use these 4 lines to plot X and Z points with ROS
        
        # plt.plot(x_np_array, z_np_array, 'x')
        # plt.axis("equal")
        # plt.draw()
        # plt.pause(0.0000000001)   
        
        mean_x_array = abs(x_np_array.mean(axis=0))
        mean_z_array = z_np_array.mean(axis=0)        
        cov_x = np.cov(x_np_array)
        cov_z = np.cov(z_np_array)
        
        # print("mean x", mean_x_array, "", "mean_z", mean_z_array)
        # print("Covariance x", cov_x, "", "Covariance_z", cov_z)
        
        
        # find mean for the XZ array
        xz_np_array = np.array(xzarray)
        mean2D = abs(xz_np_array.mean(axis=1))   # axis  = 0 is for column, axis 1 for row and abs is to keep everything positive
        cov_xz = np.cov(xz_np_array)
        # cov_xz = np.cov(x_np_array,z_np_array)
        print("mean", mean2D.shape)
        print(cov_xz.shape)
        # print("Mean of 2D array",mean2D, "" , "Covariance ", cov_xz)
        
        # Y= np.random.multivariate_normal(mean2D,cov_xz, 3)
        # # print(X)
        # # print(Y)
        # # print(Z)
        # plt.plot(Y)
        # plt.draw()
        # plt.pause(10)
        
        
        DBSCAN_cluster = DBSCAN(eps=0.5, min_samples=30).fit(xz_np_array)
        print(DBSCAN_cluster.labels_)
        # print(DBSCAN_cluster.components_)
        labels = DBSCAN_cluster.labels_
        components = DBSCAN_cluster.components_ #copy of each core sample found by training
        feature = DBSCAN_cluster.n_features_in_ #number of features seen during fit
        
        
        useful_cluster = []
        x = []
        y = []
        for i in range(len(components)):
            # print(components[i])
            # print(labels[i])
            
            if labels[i] == 0 :
                useful_cluster.append(components[i])
                point = components[i]
                x.append(point[0])
                y.append(point[1])
        plt.scatter(x,y)
        plt.draw()
        plt.pause(0.00001)
        
def main():
    rospy.init_node('Compute_Gaussian', anonymous=False)
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

    